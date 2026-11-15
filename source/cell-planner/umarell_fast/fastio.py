"""Parse-once cache for umarell's YAML cell inputs (umarell HEAD eccb1bc).

``utils.cell_store.read_yaml`` re-parses robots.yaml / obstacles.yaml with pure-Python PyYAML on every call; in the
replays it is called thousands of times per plan (welding_cell_2t's obstacles.yaml, 152 kB / 738 boxes, costs
265 ms per parse). This keeps umarell's read path (``read_text``: local file or the cell-data service) and only
memoises the parse by the text's content hash, returning a deep copy so callers that mutate the mapping cannot
corrupt the cache. Install into a scratch umarell copy with :func:`install`.
"""
from __future__ import annotations

import copy
import hashlib

_PARSED: dict = {}
STATS = {"hits": 0, "misses": 0}


def install() -> None:
    from utils import cell_store
    if getattr(cell_store.read_yaml, "_uf_cached", False):
        return
    orig_read_text = cell_store.read_text

    def read_yaml(cell_id, kind, name, *, cell_base=None):
        text = orig_read_text(cell_id, kind, name, cell_base=cell_base)
        if text is None:
            return None
        key = hashlib.blake2b(text.encode("utf-8", "surrogatepass"), digest_size=16).digest()
        hit = _PARSED.get(key)
        if hit is None:
            import yaml
            loader = getattr(yaml, "CSafeLoader", yaml.SafeLoader)
            data = yaml.load(text, Loader=loader)
            hit = data if isinstance(data, dict) else None
            _PARSED[key] = hit
            STATS["misses"] += 1
        else:
            STATS["hits"] += 1
        return copy.deepcopy(hit)

    read_yaml._uf_cached = True
    cell_store.read_yaml = read_yaml
