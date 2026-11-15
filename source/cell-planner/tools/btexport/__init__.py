"""tools.btexport: ONE exporter from every tool of this project to umarell's behaviour-tree plan format
(docs/p3_btexport.md).

    PYTHONPATH=. .venv/bin/python -m tools.btexport --from <spec> --cell <cell> --out plan.json [--xml plan.xml]
        [--profile umarell|full] [--validate]

    from tools.btexport import export_spec
    plan, report = export_spec("ppo:runs/ppo_v3h/selected_snap500.pt", cell="welding_cell_2_small", out="p.json")

Modules: sources (inputs -> Source: controller rollouts, umarell plans, trajgen runs, timelines, execute.py results),
core (Source -> plan JSON + XML with the unified rules and checks), retime (speed profiles; ``full`` = URDF velocity
limits with acceleration ramps, weld operations slow), limits (URDF joint limits + acceleration override),
validate / umarell_check (umarell's own loaders, validators and executor on a scratch repo copy).
"""
from __future__ import annotations


def export_spec(spec: str, *, cell=None, profile: str = "umarell", out=None, xml=None, tree_id=None, **kw):
    from tools.btexport import core, sources
    src = sources.resolve(spec, cell=cell, profile=profile)
    return core.export(src, profile=profile, out=out, xml=xml, tree_id=tree_id, **kw)
