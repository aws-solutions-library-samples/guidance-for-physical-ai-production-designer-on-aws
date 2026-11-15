"""U3: the umarell-fast orchestrator with every discrete choice made by rules -- no decider, no LLM.

The no-model lower bound of the umarell-fast study: the same orchestrator as U2 (umarell_fast.orchestrator), with
the base :class:`~umarell_fast.orchestrator.Policy` (rules) for every decision. Run it with
``python -m umarell_fast.u3 --cell welding_cell_2_small`` (see ``umarell_fast/u3/__main__.py``).
"""
from umarell_fast.orchestrator import Params, Policy


class RulePolicy(Policy):
    """U3's policy: exactly the base rules (kept as its own class so U3's choices are named in reports)."""
    name = "u3_rules"


__all__ = ["RulePolicy", "Params"]
