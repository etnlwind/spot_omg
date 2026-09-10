"""Offline triangle-mesh clearance audit, not MuJoCo's primitive contact proxies.

Requires requirements-stow.txt. Never drives hardware or changes live qpos.
"""
import hashlib
import itertools
import json
from pathlib import Path

import fcl
import mujoco
import numpy as np

from cad_gait import CAD
from stow_policy import LANDING, FOLDED

# Original design endpoint retained ONLY for offline collision-limit measurement.
UNRESTRICTED_FOLDED = [0., -265., 0.] * 2 + [0., -85., 0.] * 2
DESIGN_CLEARANCE_M = .018
MIN_DYNAMIC_CLEARANCE_M = .005


class LegClearance:
    """FCL BVH triangle distances for all 54 different-leg link pairs.

Includes the imported motor housings. Same-leg articulated mating surfaces,
    chassis, cables and unmodelled parts are outside this inter-leg audit.
    """
    def __init__(self, model):
        self.model = model
        self.objects = {}
        for leg in ('fl', 'fr', 'rl', 'rr'):
            for j in (1, 2, 3):
                name = f'{leg}_j{j}'
                g = model.geom(name).id
                mesh = model.geom_dataid[g]
                va, fa = model.mesh_vertadr[mesh], model.mesh_faceadr[mesh]
                v = model.mesh_vert[va:va+model.mesh_vertnum[mesh]].astype(float)
                faces = model.mesh_face[fa:fa+model.mesh_facenum[mesh]]
                bvh = fcl.BVHModel()
                bvh.beginModel(len(v), len(faces))
                bvh.addSubModel(v, faces)
                bvh.endModel()
                corners = np.array(list(itertools.product(*zip(v.min(0), v.max(0)))))
                self.objects[name] = (g, fcl.CollisionObject(bvh), corners)
        self.pairs = [(a,b) for a,b in itertools.combinations(self.objects, 2)
                      if a[:2] != b[:2]]

    def measure(self, data, ceiling=float('inf')):
        """Return minimum distance/pair, or ceiling/None if all farther away."""
        bounds = {}
        for name, (g, obj, corners) in self.objects.items():
            rotation = data.geom_xmat[g].reshape(3,3)
            position = data.geom_xpos[g]
            obj.setTransform(fcl.Transform(rotation, position))
            world = corners @ rotation.T + position
            bounds[name] = (world.min(0), world.max(0))
        candidates = []
        for a,b in self.pairs:
            al,ah = bounds[a]; bl,bh = bounds[b]
            lower = float(np.linalg.norm(np.maximum(0, np.maximum(al-bh, bl-ah))))
            candidates.append((lower,a,b))
        best, pair = ceiling, None
        for lower,a,b in sorted(candidates):
            if lower >= best: break
            distance = fcl.distance(self.objects[a][1], self.objects[b][1],
                                    fcl.DistanceRequest(), fcl.DistanceResult())
            if distance < best:
                best, pair = max(0., float(distance)), (a,b)
            if best == 0: break
        return best, pair


def audit_design():
    from stow_preview import make_plant
    plant = make_plant(True, True)
    checker = LegClearance(plant.model)
    start, delta = np.array(LANDING), np.array(UNRESTRICTED_FOLDED)-LANDING
    def sample(fraction):
        plant.data.qpos[plant.q] = np.radians(start+delta*fraction)
        mujoco.mj_forward(plant.model, plant.data)
        distance,pair = checker.measure(plant.data)
        return dict(fraction=float(fraction), clearance_mm=distance*1000,
                    pair=pair, angles_deg=(start+delta*fraction).tolist())
    samples = []
    # At most 0.305 degrees of front J2 between samples; refine first crossings.
    boundaries = {}
    for fraction in np.linspace(0,1,1001):
        row = sample(fraction); samples.append(row)
        for label,threshold in [('margin', DESIGN_CLEARANCE_M*1000), ('contact', .001)]:
            if label not in boundaries and row['clearance_mm'] <= threshold:
                lo = max(0, fraction-.001); hi = fraction
                for _ in range(14):
                    mid = (lo+hi)/2
                    if sample(mid)['clearance_mm'] <= threshold: hi = mid
                    else: lo = mid
                boundaries[label] = sample(hi)
        if len(boundaries) == 2: break
    # Round down along the path, leaving at least the design clearance.
    safe_fraction = np.floor(boundaries['margin']['fraction']*1000)/1000
    selected = sample(safe_fraction)
    return dict(method='FCL BVH distances on full imported CAD triangles; no convex hulls/proxies',
                pairs=len(checker.pairs), design_clearance_mm=DESIGN_CLEARANCE_M*1000,
                dynamic_clearance_required_mm=MIN_DYNAMIC_CLEARANCE_M*1000,
                boundaries=boundaries, selected=selected,
                policy_matches=bool(np.allclose(selected['angles_deg'],FOLDED)),
                mesh_sha256={name:hashlib.sha256((CAD/(name+'.stl')).read_bytes()).hexdigest()
                             for name in checker.objects}, samples=samples,
                limitations=['Provisional CAD joint axes and zero poses.',
                             'Inter-leg surfaces only; excludes same-leg mating surfaces, chassis and wiring.',
                             'Finite path sampling; physical assembly and actuator tolerances unmeasured.'])


if __name__ == '__main__':
    result = audit_design()
    path = Path(__file__).parent/'diagnostics/stow/clearance.json'
    path.write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in ('samples','mesh_sha256')},indent=2))
    raise SystemExit(0 if result['policy_matches'] else 1)
