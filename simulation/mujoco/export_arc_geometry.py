"""Export zero-angle CAD screw axes and measured cushion mesh for embedded IK."""
import json
from pathlib import Path
import numpy as np
import mujoco
from cad_physics import build
from search_gait_profiles import physics
ROOT=Path(__file__).resolve().parents[2]
p,_=physics();p['foot_cushion']=json.loads(Path(__file__).with_name('foot_cushion_10mm.json').read_text())
xml,p=build(p,write_scene=False);m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m)
d.qpos[:]=m.qpos0;d.qpos[:3]=0;d.qpos[3:7]=[1,0,0,0]
for leg in ('fl','fr','rl','rr'):
 for j in (1,2,3):d.qpos[m.jnt_qposadr[m.joint(f'{leg}_j{j}').id]]=0
mujoco.mj_forward(m,d)
def array(v):
 if isinstance(v,(list,np.ndarray)):return '{'+','.join(array(x) for x in v)+'}'
 return f'{float(v):.9f}f'
axes=[];anchors=[];centers=[];verts=[]
for leg in ('fl','fr','rl','rr'):
 ids=[m.joint(f'{leg}_j{j}').id for j in (1,2,3)]
 axes.append(d.xaxis[ids]);anchors.append(d.xanchor[ids])
 g=m.geom(f'{leg}_foot').id;centers.append(d.geom_xpos[g].copy())
 mesh=m.geom_dataid[g];a=m.mesh_vertadr[mesh];n=m.mesh_vertnum[mesh]
 verts.append(m.mesh_vert[a:a+n]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g])

from support_shift import SupportShift
from gait_profiles import foot_targets
control=SupportShift(m);params=[1.44,.5,.08,.02,.20175,-.01,.75]
neutral=foot_targets(params,0,0).reshape(-1)
control.plan(params,0,0,0,0,dict(lateral_m=0,lower_m=0))
gids=[m.geom(f'{leg}_foot').id for leg in ('fl','fr','rl','rr')]
mesh=m.geom_dataid[gids[0]];a=m.mesh_vertadr[mesh];n=m.mesh_vertnum[mesh]
local=m.mesh_vert[a:a+n].copy()
for g in gids:
 other=m.geom_dataid[g];start=m.mesh_vertadr[other];num=m.mesh_vertnum[other]
 assert np.array_equal(local,m.mesh_vert[start:start+num])
rows=['/* Generated from CAD + measured cushion. Canonical radians; no servo ticks. */','#ifndef ARC_GEOMETRY_H','#define ARC_GEOMETRY_H']
for name,v,shape in [('axes',axes,'[4][3][3]'),('anchors',anchors,'[4][3][3]'),('centers',centers,'[4][3]'),('orientations',d.geom_xmat[gids].reshape(4,3,3),'[4][3][3]'),('vertices',local,f'[{n}][3]'),('neutral',neutral,'[12]'),('reference',control.reference,'[4][3]'),('center',control.center,'[3]')]:
 rows.append(f'static const float arc_{name}{shape}='+array(v)+';')
# Balanced spatial tree for exact minimum projection, including flat faces.
nodes=[];order=[]
def tree(ids):
 index=len(nodes);points=local[ids];lo=points.min(axis=0);hi=points.max(axis=0)
 nodes.append(None)
 if len(ids)<=8:
  start=len(order);order.extend(map(int,ids));meta=[0,0,start,len(ids)]
 else:
  axis=int(np.argmax(hi-lo));ids=ids[np.argsort(points[:,axis],kind='stable')];middle=len(ids)//2
  left=tree(ids[:middle]);right=tree(ids[middle:]);meta=[left,right,0,0]
 nodes[index]=(np.r_[lo,hi],meta)
 return index
tree(np.arange(n))
rows.append(f'static const float arc_bounds[{len(nodes)}][6]='+array([v[0] for v in nodes])+';')
rows.append(f'static const uint16_t arc_nodes[{len(nodes)}][4]={{'+','.join('{'+','.join(map(str,v[1]))+'}' for v in nodes)+'};')
rows.append(f'static const uint16_t arc_order[{len(order)}]={{'+','.join(map(str,order))+'};')
# Offline Cartesian seed grid: initialization only, never a commanded pose.
# Runtime IK still checks the original target with the full cushion geometry.
seed_origin=np.array([-.06,-.03,0.]);seed_step=np.array([.015,.015,.01])
seeds=np.empty((4,9,5,3,3));worst_seed_residual=0.
for ix in range(9):
 for iy in range(5):
  for iz in range(3):
   target=control.reference+seed_origin+seed_step*np.array([ix,iy,iz])
   angles,residual=control.solve(target,neutral,iterations=32)
   seeds[:,ix,iy,iz,:]=angles.reshape(4,3)
   worst_seed_residual=max(worst_seed_residual,residual)
assert np.isfinite(seeds).all()
rows.append('static const float arc_seed_origin[3]='+array(seed_origin)+';')
rows.append('static const float arc_seed_step[3]='+array(seed_step)+';')
rows.append('static const float arc_seeds[4][9][5][3][3]='+array(seeds)+';')
rows.append('#define ARC_HAS_SEEDS 1')
print('seed grid worst residual (not a motion target)',worst_seed_residual)
rows += [f'#define ARC_VERTEX_COUNT {n}',f'#define ARC_NODE_COUNT {len(nodes)}','#endif','']
(ROOT/'firmware/stm32-learning/Inc/arc_geometry.h').write_text('\n'.join(rows))
print('shared vertices',n)
