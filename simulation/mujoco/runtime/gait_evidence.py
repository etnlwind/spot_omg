"""Contact/clearance evidence, separate from no-fall and final-pose checks."""
import numpy as np
import mujoco

LEGS=('fl','fr','rl','rr')

def foot_loads(model,data):
    floor=model.geom('floor').id
    ids=[model.geom(leg+'_foot').id for leg in LEGS]
    loads=[0.]*4
    for index in range(data.ncon):
        c=data.contact[index]
        if floor not in (c.geom1,c.geom2):continue
        other=c.geom2 if c.geom1==floor else c.geom1
        if other not in ids:continue
        force=np.zeros(6);mujoco.mj_contactForce(model,data,index,force)
        loads[ids.index(other)]+=max(0.,float(force[0]))
    return loads

def swing_quality(rows):
    """Audit middle 60% of swing: >0.5N load or <=2mm gap fails.

    Diagnostic thresholds, not a hardware certification. Empty evidence fails.
    Caller supplies actual command phase and excludes stopping transitions.
    """
    result={}
    for i,leg in enumerate(LEGS):
        mid=[r for r in rows if .6<=(r['leg_phase'][i] if r.get('leg_phase') is not None else (r['phase']+(.5 if i in (0,3) else 0))%1)<=.9]
        result[leg]=dict(samples=len(mid),
            loaded_fraction=sum(r['loads_n'][i]>.5 for r in mid)/len(mid) if mid else None,
            min_clearance_mm=min((r['clearance_mm'][i] for r in mid),default=None))
    passed=all(v['samples'] and v['loaded_fraction']==0 and v['min_clearance_mm']>2 for v in result.values())
    return dict(passed=bool(passed),criteria='middle swing: load <=0.5N and clearance >2mm for all four legs; no evidence is not pass',legs=result)
