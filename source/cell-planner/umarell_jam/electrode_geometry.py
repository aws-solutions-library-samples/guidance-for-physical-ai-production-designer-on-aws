"""Refine boundary-crossing electrode triangles without changing the mesh surface."""
import hashlib
from pathlib import Path

import hppfcl as fcl
import numpy as np
import pinocchio as pin

from .geometry import Geometry


def revision():
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


class RefinedElectrodeGeometry(Geometry):
    """Same 20 mm electrode region and native contact radius; finer conservative cover."""
    def __init__(self, cell):
        super().__init__(cell)
        self.refinement_counts = {}
        for rid, arm in self.arms.items():
            cfg, model = cell.robots[rid], arm["m"]
            for index, go in enumerate(arm["visual"].geometryObjects):
                if not go.meshPath.endswith("/tip.obj"):
                    continue
                tcp = model.frames[model.getFrameId(cfg["tip_link"])]
                offset = pin.SE3(np.eye(3), np.asarray(cfg.get("tcp_offset", [0., 0., 0.])))
                center = (go.placement.inverse() * tcp.placement * offset).translation
                vertices = np.asarray(go.geometry.vertices())
                pending = [vertices[[go.geometry.tri_indices(i)[j] for j in range(3)]]
                           for i in range(go.geometry.num_tris)]
                accepted, ordinary = [], []
                while pending:
                    triangle = pending.pop()
                    radius = np.linalg.norm(triangle - center, axis=1)
                    edges = [np.linalg.norm(triangle[i] - triangle[j]) for i, j in ((0, 1), (1, 2), (2, 0))]
                    longest = max(edges)
                    if max(radius) <= .02:
                        accepted.append(triangle)
                    elif min(radius) - longest >= .02 or longest <= .002:
                        ordinary.append(triangle)
                    else:
                        i, j = ((0, 1), (1, 2), (2, 0))[int(np.argmax(edges))]
                        k = 3 - i - j
                        middle = (triangle[i] + triangle[j]) / 2
                        # The two triangles exactly cover the original planar triangle.
                        pending.extend([np.array([triangle[i], middle, triangle[k]]),
                                        np.array([middle, triangle[j], triangle[k]])])
                parts = []
                for triangles, electrode in ((accepted, True), (ordinary, False)):
                    if not triangles:
                        continue
                    shape = fcl.BVHModelOBBRSS()
                    shape.beginModel(len(triangles), 3 * len(triangles))
                    vv, ff = fcl.StdVec_Vec3f(), fcl.StdVec_Triangle()
                    for number, triangle in enumerate(triangles):
                        for point in triangle:
                            vv.append(point)
                        ff.append(fcl.Triangle(3*number, 3*number+1, 3*number+2))
                    shape.addSubModel(vv, ff)
                    shape.endModel()
                    parts.append((shape, electrode))
                arm["tool_parts"][index] = parts
                self.refinement_counts[rid] = {
                    "original_triangles": go.geometry.num_tris,
                    "contact_triangles": len(accepted), "ordinary_triangles": len(ordinary),
                    "electrode_local_radius_m": .02, "boundary_resolution_m": .002}
