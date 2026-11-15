"""Check path membership for motions certified safe under independent arm clocks."""
from collections import OrderedDict
import math

import numpy as np
from scipy.optimize import minimize_scalar
from scipy.spatial import cKDTree

from .trajectory import Trajectory

_PATHS = OrderedDict()


def path_error(points, actual, status):
    key = id(points)
    found = _PATHS.get(key)
    if found is None or found[0] is not points:
        path = Trajectory(points)
        knots = cKDTree(np.asarray([p["positions"] for p in points]))
        found = points, path, knots
        _PATHS[key] = found
        if len(_PATHS) > 64:
            _PATHS.popitem(last=False)
    _, path, knots = found
    q = np.asarray(actual, dtype=float)
    if q.shape != (path.n,) or not np.all(np.isfinite(q)):
        return math.inf
    # A controller-reported progress time is a candidate only: measured joint
    # feedback still has to agree with that point of the certified curve.
    progress = (status or {}).get("trajectory_time_s")
    if isinstance(progress, (float, int)) and math.isfinite(progress):
        err = float(np.max(abs(q - path.sample(float(progress)))))
        if err <= 1e-7:
            return err
    _, nearest = knots.query(q, k=min(6, len(points)))
    segments = {min(max(int(i) + offset, 0), len(points)-2)
                for i in np.atleast_1d(nearest) for offset in (-1, 0)}
    error = math.inf
    for i in segments:
        a, b = path.times[i:i+2]
        objective = lambda t: float(np.sum((q - path.sample(float(t)))**2))
        solution = minimize_scalar(objective, bounds=(a, b), method="bounded",
                                   options={"xatol": 1e-12})
        for t in (a, b, solution.x):
            error = min(error, float(np.max(abs(q - path.sample(float(t))))))
        if error <= 1e-7:
            break
    return error
