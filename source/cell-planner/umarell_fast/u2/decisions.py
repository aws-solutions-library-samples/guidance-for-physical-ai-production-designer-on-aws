"""Decision router of U2: every discrete choice the orchestrator makes goes through here and is logged.

Sources, in the order a mode consults them:

* ``rule``    -- a fixed rule written before any run (the rule baselines of docs/uf_decider_study.md §1, plus
                 "try the candidates concurrently and keep the first that works" for recoveries).
* ``decider`` -- strands-decider served locally (umarell_fast.u2.decider); its answer is used when its confidence
                 is >= ``threshold`` (0.9, the vendor's calibration point).
* ``llm``     -- Opus 5 on Bedrock with a compact state and a prompt-cached system prompt (umarell_fast.u2.llm).

Modes (``--decisions``):

* ``decider`` (the U2 specification): decider for the discrete types; below the confidence threshold, or for a type
  the decider cannot answer (generative), Opus; the rule only when Opus is unavailable or over budget.
* ``rules``: the rule for every discrete type; Opus only for generative questions (park poses) when code search fails.
* ``llm``: Opus for every decision (reference point for cost / latency).

Every decision is appended to ``<run>/decisions.jsonl`` with its type, options, the rule's answer, every consulted
source's answer + confidence + latency, the final choice and its source. The router never blocks the planner: the
orchestrator launches the candidate work for all options concurrently and asks the router in parallel; the router's
answer only selects among the candidates (``decide`` may be called from a worker thread).
"""
from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

DISCRETE_TYPES = ("recovery", "step_mode", "home_at_end", "accept_tree", "assemble_reaction", "start_delay",
                  "reallocate")
GENERATIVE_TYPES = ("park_pose",)
MODES = ("decider", "rules", "llm")


@dataclass
class Question:
    dtype: str
    question: str
    options: list            # [(name, description), ...]
    rule: str                # the rule's answer (an option name)
    kind: str = "choice"     # choice | noul
    qid: str = ""


@dataclass
class Decision:
    dtype: str
    qid: str
    choice: str
    source: str
    confidence: float | None
    rule: str
    detail: dict = field(default_factory=dict)


def _llm_prompt(state: str, q: Question) -> str:
    opts = "\n".join(f"  - {n}: {d}" for n, d in q.options)
    kind = "yes/no question (answer \"true\" or \"false\")" if q.kind == "noul" else "choice"
    return (f"STATE\n{state}\n\nQUESTION ({q.dtype}, {kind})\n{q.question}\nOPTIONS\n{opts}\n\n"
            "Answer with one JSON object as specified by the U2 decision protocol.")


class Router:
    def __init__(self, mode: str, run_dir: Path, decider=None, llm=None, threshold: float = 0.9):
        if mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}")
        self.mode = mode
        self.decider = decider
        self.llm = llm
        self.threshold = threshold
        self.path = Path(run_dir) / "decisions.jsonl"
        self._lock = threading.Lock()
        self._n = 0
        self.records: list[dict] = []

    # -- public ------------------------------------------------------------------------------------------------
    def decide(self, state: str, q: Question) -> Decision:
        return self.decide_many(state, [q])[0]

    def decide_many(self, state: str, qs: list[Question]) -> list[Decision]:
        """Answer several questions over one state (one decider request for all of them)."""
        t0 = time.perf_counter()
        for q in qs:
            if not q.qid:
                with self._lock:
                    self._n += 1
                    q.qid = f"d{self._n:03d}"
        dec: dict[str, Decision] = {}
        details: dict[str, dict] = {q.qid: {} for q in qs}
        names = {q.qid: [n for n, _ in q.options] for q in qs}

        if self.mode == "rules":
            for q in qs:
                dec[q.qid] = Decision(q.dtype, q.qid, q.rule, "rule", None, q.rule)
        else:
            pending = list(qs)
            if self.mode == "decider" and self.decider is not None:
                ask = {q.qid: {"kind": q.kind, "question": q.question, "options": q.options} for q in pending
                       if q.dtype in DISCRETE_TYPES}
                if ask:
                    ans, info = self.decider.ask(state, ask)
                    for q in pending:
                        a = ans.get(q.qid)
                        if q.qid in ask:
                            details[q.qid]["decider"] = {**(a or {}), **{k: v for k, v in info.items()
                                                                          if k != "usage"}}
                        if a and a.get("choice") in names[q.qid] and a["confidence"] >= self.threshold:
                            dec[q.qid] = Decision(q.dtype, q.qid, a["choice"], "decider", a["confidence"], q.rule)
                    pending = [q for q in pending if q.qid not in dec]
            # Opus for the rest (below the threshold, decider unavailable, or mode == "llm"): all questions of the
            # batch concurrently -- each is an independent short request
            def _ask(q):
                return q, self.llm.ask(_llm_prompt(state, q), purpose=f"{q.dtype}:{q.qid}", effort="low",
                                       max_tokens=4000)
            answers = {}
            if self.llm is not None and pending:
                from concurrent.futures import ThreadPoolExecutor
                with ThreadPoolExecutor(max_workers=min(8, len(pending))) as ex:
                    for q, (parsed, info) in ex.map(_ask, pending):
                        answers[q.qid] = (parsed, info)
            for q in pending:
                if q.qid in answers:
                    parsed, info = answers[q.qid]
                    details[q.qid]["llm"] = {k: info.get(k) for k in ("input_tokens", "output_tokens",
                                                                       "cache_read_tokens", "cache_write_tokens",
                                                                       "cost_usd", "latency_s", "error")}
                    ch = str((parsed or {}).get("choice", "")).strip()
                    if ch in names[q.qid]:
                        details[q.qid]["llm"]["why"] = str((parsed or {}).get("why", ""))[:300]
                        dec[q.qid] = Decision(q.dtype, q.qid, ch, "llm", None, q.rule)
                        continue
                    details[q.qid]["llm"]["unparsed"] = (info.get("text") or "")[:300]
                dec[q.qid] = Decision(q.dtype, q.qid, q.rule, "rule", None, q.rule,
                                      {"fallback": "llm unavailable / over budget / unparsable"})
        lat = time.perf_counter() - t0
        out = []
        batch = qs[0].qid if qs else ""
        for q in qs:
            d = dec[q.qid]
            d.detail.update(details[q.qid])
            self._log(state, q, d, lat, batch)
            out.append(d)
        return out

    def note_override(self, d: Decision, final: str, why: str) -> None:
        """The chosen option turned out infeasible (its candidate failed): record what was used instead."""
        rec = {"t": time.time(), "qid": d.qid, "type": d.dtype, "override": True, "chosen": d.choice,
               "final": final, "why": why}
        self._write(rec)

    def summary(self) -> dict:
        with self._lock:
            recs = [r for r in self.records if not r.get("override")]
        by_src: dict[str, int] = {}
        by_type: dict[str, dict] = {}
        for r in recs:
            by_src[r["source"]] = by_src.get(r["source"], 0) + 1
            t = by_type.setdefault(r["type"], {"n": 0, "sources": {}, "agree_rule": 0})
            t["n"] += 1
            t["sources"][r["source"]] = t["sources"].get(r["source"], 0) + 1
            t["agree_rule"] += int(r["choice"] == r["rule"])
        dec = [r["decider"] for r in recs if r.get("decider")]
        batches = {}
        for r in recs:
            if r.get("decider"):
                batches[r.get("batch")] = r["decider"].get("latency_s") or 0
        return {"decisions": len(recs), "by_source": by_src, "by_type": by_type,
                "decider_asked": len(dec),
                "decider_confident": sum(1 for x in dec if (x.get("confidence") or 0) >= self.threshold),
                "decider_requests": len(batches), "decider_latency_s": round(sum(batches.values()), 3),
                "llm_calls": sum(1 for r in recs if r.get("llm")),
                "wait_s": round(sum(b for b in {r.get("batch"): r.get("latency_s") or 0 for r in recs}.values()), 3)}

    # -- internals ---------------------------------------------------------------------------------------------
    def _log(self, state: str, q: Question, d: Decision, lat: float, batch: str = "") -> None:
        rec = {"t": time.time(), "qid": q.qid, "batch": batch, "type": q.dtype, "kind": q.kind, "question": q.question,
               "options": [n for n, _ in q.options], "rule": q.rule, "choice": d.choice, "source": d.source,
               "confidence": d.confidence, "mode": self.mode, "latency_s": round(lat, 3),
               "state": state[:4000], **d.detail}
        self._write(rec)

    def _write(self, rec: dict) -> None:
        with self._lock:
            self.records.append(rec)
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.path, "a") as f:
                f.write(json.dumps(rec, default=str) + "\n")

    # -- generative --------------------------------------------------------------------------------------------
    def generate_points(self, state: str, question: str, qid_hint: str = "park") -> tuple[list, dict]:
        """Generated Cartesian candidates (park / stage points) from Opus. Logged as a ``park_pose`` decision."""
        if self.llm is None:
            return [], {"error": "no llm"}
        with self._lock:
            self._n += 1
            qid = f"g{self._n:03d}"
        prompt = f"STATE\n{state}\n\nQUESTION (park_pose, coordinates)\n{question}\n\nAnswer with one JSON object."
        parsed, info = self.llm.ask(prompt, purpose=f"park_pose:{qid}", effort="medium", max_tokens=8000)
        pts = []
        for p in (parsed or {}).get("points") or []:
            try:
                pts.append({"x": float(p["x"]), "y": float(p["y"]), "z": float(p["z"])})
            except (KeyError, TypeError, ValueError):
                continue
        rec = {"t": time.time(), "qid": qid, "type": "park_pose", "kind": "generate", "question": question,
               "choice": json.dumps(pts), "source": "llm", "confidence": None, "rule": None, "mode": self.mode,
               "state": state[:4000], "llm": {k: info.get(k) for k in ("input_tokens", "output_tokens",
                                                                       "cache_read_tokens", "cache_write_tokens",
                                                                       "cost_usd", "latency_s", "error")},
               "why": str((parsed or {}).get("why", ""))[:300]}
        self._write(rec)
        return pts[:3], info
