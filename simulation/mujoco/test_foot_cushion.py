import json
from pathlib import Path
import mujoco
import numpy as np
import pytest
from cad_physics import build, Simulation, foot_clearance, rounded_cap_vertices
from search_gait_profiles import physics

PAD=json.loads(Path(__file__).with_name('foot_cushion_10mm.json').read_text())

def models(pad=PAD):
    p,bare=physics()
    p['foot_cushion']=pad
    xml,p=build(p,write_scene=False)
    padded=mujoco.MjModel.from_xml_string(xml)
    return p,bare,padded

@pytest.mark.parametrize('thickness',[.01,.02])
def test_pad_extends_toe_and_adds_mass(thickness):
    pad=dict(PAD,align_to_triangle_base=False,thickness_m=thickness,mass_kg=.005*thickness/.01)
    p,bare,padded=models(pad)
    assert padded.body_mass.sum()-bare.body_mass.sum()==pytest.approx(4*pad['mass_kg'])
    for leg in ('fl','fr','rl','rr'):
        old=bare.geom(leg+'_foot').id
        foot=padded.geom(leg+'_foot').id
        child=padded.geom_bodyid[foot]
        bare_bottom=bare.geom_pos[old,2]-bare.geom_size[old,0]
        mesh=padded.geom_dataid[foot]
        start=padded.mesh_vertadr[mesh];count=padded.mesh_vertnum[mesh]
        rot=np.empty(9);mujoco.mju_quat2Mat(rot,padded.geom_quat[foot])
        verts=padded.mesh_vert[start:start+count]@rot.reshape(3,3).T+padded.geom_pos[foot]
        pad_bottom=padded.body_pos[child,2]+verts[:,2].min()
        assert bare_bottom-pad_bottom==pytest.approx(thickness)
        assert padded.geom_type[foot]==mujoco.mjtGeom.mjGEOM_MESH
        assert np.ptp(verts[:,2])==pytest.approx(.027)
        assert padded.geom_solref[foot,0]==pytest.approx(.02)

def test_rotated_pad_clearance_and_initial_ground_placement():
    p,_,m=models();s=Simulation(p,m)
    values=[]
    for leg in ('fl','fr','rl','rr'):
        f=m.geom(leg+'_foot').id
        mesh=m.geom_dataid[f];start=m.mesh_vertadr[mesh];count=m.mesh_vertnum[mesh]
        world=m.mesh_vert[start:start+count]@s.data.geom_xmat[f].reshape(3,3).T+s.data.geom_xpos[f]
        extent=s.data.geom_xpos[f,2]-world[:,2].min()
        assert foot_clearance(m,s.data,f)==pytest.approx(s.data.geom_xpos[f,2]-extent)
        values.append(foot_clearance(m,s.data,f))
    assert min(values)==pytest.approx(.001)

def test_rounded_cap_preserves_bounds_but_removes_sharp_corners():
    size=np.array([.015,.018,.0135]);r=.004
    vertices=rounded_cap_vertices(size,r)
    assert np.max(vertices,axis=0)==pytest.approx(size)
    assert np.min(vertices,axis=0)==pytest.approx(-size)
    assert np.max(vertices@np.ones(3)) < sum(size)-.002

def test_cap_base_parallel_to_cad_triangle_and_length_27mm():
    from cad_physics import vertices,toe_base_frame
    _,_,m=models()
    for leg in ('fl','fr','rl','rr'):
        frame,anchor=toe_base_frame(vertices(leg+'_j3'))
        b=m.body(leg+'_cushion').id
        rot=np.empty(9);mujoco.mju_quat2Mat(rot,m.body_quat[b])
        assert rot.reshape(3,3)[:,2]==pytest.approx(frame[:,2],abs=1e-7)
        foot=m.geom(leg+'_foot').id
        mesh=m.geom_dataid[foot];a=m.mesh_vertadr[mesh];n=m.mesh_vertnum[mesh]
        g=np.empty(9);mujoco.mju_quat2Mat(g,m.geom_quat[foot])
        local=m.mesh_vert[a:a+n]@g.reshape(3,3).T+m.geom_pos[foot]
        assert np.ptp(local[:,2])==pytest.approx(.027)

def test_elastic_cap_has_closed_rounded_outline_and_exact_length():
    from cad_physics import elastic_cap_vertices
    size=np.array([.018,.02,.0135]);v=elastic_cap_vertices(size)
    assert v.max(0)==pytest.approx(size)
    assert v.min(0)==pytest.approx(-size)
    assert np.max(v[:,0]/size[0]+v[:,1]/size[1]) < 1.6
    assert np.min(v[:,2])==pytest.approx(-.0135)

def test_measured_sole_is_circle_37_3mm():
    from cad_physics import elastic_cap_vertices
    radius=.0373/2
    v=elastic_cap_vertices([radius,radius,.027/2],circular=True)
    equator=v[np.abs(v[:,2])<1e-9]
    assert np.linalg.norm(equator[:,:2],axis=1)==pytest.approx(radius,abs=1e-9)
    _,_,m=models()
    for leg in ('fl','fr','rl','rr'):
        f=m.geom(leg+'_foot').id;mesh=m.geom_dataid[f]
        a=m.mesh_vertadr[mesh];n=m.mesh_vertnum[mesh]
        rot=np.empty(9);mujoco.mju_quat2Mat(rot,m.geom_quat[f])
        local=m.mesh_vert[a:a+n]@rot.reshape(3,3).T+m.geom_pos[f]
        assert np.ptp(local,axis=0)==pytest.approx([.0373,.0373,.027],abs=1e-7)
