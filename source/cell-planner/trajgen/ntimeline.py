"""N-robot timeline (any number of robots, any robot ids), compatible with ``player.timeline``.

    tl = NTimeline.from_legs(legs, robot_ids=("r1", "r2", "r3"), dt=0.05, home=homes)
    tl.q                   # (N_t, R, 6) on the grid tl.t
    tl.robot_ids           # robot order of the second axis
    tl.sample(ts)          # (..., R, 6), umarell sampling semantics (player.timeline.sample_leg per leg)
    tl.save("timeline.npz"); NTimeline.load("timeline.npz")

File format (``format = "trajgen_ntimeline_v1"``): the keys of ``player.timeline.Timeline.save`` (t, q, home,
legs_json, weld_events_json, meta_json) plus ``robot_ids_json`` and ``format``. A file with R = 2 and the ids
robot1/robot2 is therefore also a valid ``player.timeline.Timeline`` file; ``to_player()`` gives the two-robot object
for any two ids (ids robot1/robot2 in any order: columns reordered to player order; other ids renamed to
robot1/robot2 in order, the mapping in ``meta["robot_map"]``). ``load`` also reads
plain player files (robot ids robot1/robot2).
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from player.timeline import Leg, sample_leg

FORMAT = "trajgen_ntimeline_v1"
PLAYER_IDS = ("robot1", "robot2")


@dataclass
class NTimeline:
    t: np.ndarray
    q: np.ndarray                                   # (N, R, 6)
    robot_ids: tuple
    legs: list = field(default_factory=list)
    weld_events: list = field(default_factory=list)  # [(robot, weld_id, t_start, t_end)]
    meta: dict = field(default_factory=dict)
    home: np.ndarray | None = None                   # (R, 6)

    @classmethod
    def from_legs(cls, legs: Sequence[Leg], *, robot_ids, dt: float = 0.05, t_end: Optional[float] = None,
                  home: Optional[np.ndarray] = None, meta: Optional[dict] = None) -> "NTimeline":
        robot_ids = tuple(robot_ids)
        R = len(robot_ids)
        home = np.zeros((R, 6)) if home is None else np.asarray(home, float).reshape(R, 6)
        legs = sorted(legs, key=lambda l: (l.t0, robot_ids.index(l.robot)))
        if t_end is None:
            t_end = max([max(l.t_done, l.t1) for l in legs] + [0.0])
        n = int(np.floor(t_end / dt + 1e-9)) + 1
        t = np.round(np.arange(n) * dt, 10)
        tl = cls(t=t, q=np.zeros((n, R, 6)), robot_ids=robot_ids, legs=list(legs), meta=dict(meta or {}), home=home)
        tl.q = tl.sample(t)
        tl.weld_events = sorted([(l.robot, l.op_id, round(l.t1, 6), round(l.t1 + l.hold_s, 6)) for l in legs
                                 if l.kind == "weld"], key=lambda e: e[2])
        return tl

    def robot_legs(self, robot: str) -> list:
        return sorted([l for l in self.legs if l.robot == robot], key=lambda l: l.t0)

    def sample(self, t) -> np.ndarray:
        t = np.asarray(t, float)
        flat = np.atleast_1d(t).ravel()
        R = len(self.robot_ids)
        out = np.zeros((flat.size, R, 6))
        for r, rid in enumerate(self.robot_ids):
            q = np.repeat(self.home[r][None], flat.size, axis=0)
            for leg in self.robot_legs(rid):
                m = flat >= leg.t0
                if m.any():
                    q[m] = sample_leg(leg, flat[m])
            out[:, r] = q
        return out.reshape(t.shape + (R, 6))

    @property
    def makespan(self) -> float:
        return max(l.t1 for l in self.legs) if self.legs else 0.0

    @property
    def executor_end(self) -> float:
        return max(l.t_done for l in self.legs) if self.legs else 0.0

    def save(self, path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            path, t=self.t, q=self.q, home=self.home,
            legs_json=np.array(json.dumps([l.to_json() for l in self.legs])),
            weld_events_json=np.array(json.dumps([list(e) for e in self.weld_events])),
            meta_json=np.array(json.dumps(self.meta, default=str)),
            robot_ids_json=np.array(json.dumps(list(self.robot_ids))), format=np.array(FORMAT))
        return path

    @classmethod
    def load(cls, path) -> "NTimeline":
        z = np.load(path, allow_pickle=False)
        legs = [Leg.from_json(d) for d in json.loads(str(z["legs_json"]))]
        ev = [tuple(e) for e in json.loads(str(z["weld_events_json"]))]
        rids = tuple(json.loads(str(z["robot_ids_json"]))) if "robot_ids_json" in z.files else PLAYER_IDS
        return cls(t=z["t"], q=z["q"], robot_ids=rids, legs=legs, weld_events=ev,
                   meta=json.loads(str(z["meta_json"])), home=z["home"])

    def to_player(self):
        """``player.timeline.Timeline`` (R = 2 only). If the ids are robot1/robot2 in any order, the columns are put in
        player order (column 0 = robot1), so the player's robot1 is the real robot1 (round-3 finding: a scene listing
        robot2 first got its robots swapped); other ids are renamed to robot1/robot2 in order (``meta["robot_map"]``)."""
        from player.timeline import Timeline

        if len(self.robot_ids) != 2:
            raise ValueError("player.timeline.Timeline holds exactly two robots")
        ids = tuple(self.robot_ids)
        meta = dict(self.meta)
        if set(ids) == set(PLAYER_IDS):
            perm = [ids.index(r) for r in PLAYER_IDS]
            mp = {r: r for r in ids}
            if ids != PLAYER_IDS:
                meta["column_order"] = {"source": list(ids), "player": list(PLAYER_IDS)}
        else:
            perm = [0, 1]
            mp = dict(zip(ids, PLAYER_IDS))
            meta["robot_map"] = mp
        legs = [replace(l, robot=mp[l.robot]) for l in self.legs]
        ev = [(mp[e[0]],) + tuple(e[1:]) for e in self.weld_events]
        return Timeline(t=self.t.copy(), q=self.q[:, perm].copy(), legs=legs, weld_events=ev, meta=meta,
                        home=self.home[perm].copy())


def as_ntimeline(tl, robot_ids=None) -> NTimeline:
    """Wrap a player.timeline.Timeline (robot1/robot2) or pass an NTimeline through."""
    if isinstance(tl, NTimeline):
        return tl
    rids = tuple(robot_ids) if robot_ids is not None else PLAYER_IDS
    return NTimeline(t=tl.t, q=tl.q, robot_ids=rids, legs=list(tl.legs), weld_events=list(tl.weld_events),
                     meta=dict(tl.meta), home=np.asarray(tl.home, float))
