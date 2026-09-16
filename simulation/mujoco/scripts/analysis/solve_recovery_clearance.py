"""Calculate minimum knee flexion from the full rotated cushion mesh.

Offline geometry only: no hardware commands, no servo acceleration identification.
Recorded body pose may be used by the evaluator, never as hidden gait feedback.
"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
import mujoco
import numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import RobotController, load_parameters, parse_args
from simulation.mujoco.scripts.analysis.analyze_knee_liftoff import tuck_experiment
from simulation.mujoco.scripts.analysis.analyze_j1_steady_hold import j1_experiment

LEGS = ('FL', 'FR', 'RL', 'RR')


class CushionClearance:
    def __init__(self, plant):
        self.model = plant.model
        self.data = mujoco.MjData(self.model)
        self.q = plant.q.copy()
        self.feet = [self.model.geom(leg.lower()+'_foot').id for leg in LEGS]
        self.vertices = []
        for geom in self.feet:
            mesh = self.model.geom_dataid[geom]
            start, count = self.model.mesh_vertadr[mesh], self.model.mesh_vertnum[mesh]
            self.vertices.append(self.model.mesh_vert[start:start+count].copy())
        self.reference = plant.data.qpos.copy()
        lowest = min(self.point(self.reference, leg)[2] for leg in range(4))
        self.reference[2] -= lowest  # S on z=0, before gravity/compliance settlement.
        self.walking_reference = self.reference.copy()
        self.walking_reference[self.q[::3]] -= np.radians(9.)

    def point(self, qpos, leg):
        """Actual lowest mesh point; no spherical/point-foot approximation."""
        self.data.qpos[:] = qpos
        mujoco.mj_kinematics(self.model, self.data)
        geom = self.feet[leg]
        world = (self.vertices[leg] @ self.data.geom_xmat[geom].reshape(3, 3).T
                 + self.data.geom_xpos[geom])
        return world[world[:, 2] <= world[:, 2].min()+1e-9].mean(axis=0)

    def solve(self, qpos, leg, clearance_mm, j2_deg=None, lower_deg=76., upper_deg=120.):
        """First feasible angle in the folded recovery branch, not a global joint limit."""
        pose = np.asarray(qpos).copy()
        if j2_deg is not None:
            pose[self.q[leg*3+1]] = np.radians(j2_deg)
        def evaluate(j3):
            pose[self.q[leg*3+2]] = np.radians(j3)
            return float(self.point(pose, leg)[2]*1000)
        previous = lower_deg
        for angle in np.linspace(lower_deg, upper_deg, 177):
            if evaluate(angle) >= clearance_mm:
                low, high = previous, angle
                for _ in range(22):
                    middle = (low+high)/2
                    if evaluate(middle) >= clearance_mm:
                        high = middle
                    else:
                        low = middle
                height = evaluate(high)
                point = self.point(pose, leg)
                j2_axis = self.data.xanchor[self.model.joint(LEGS[leg].lower()+'_j2').id]
                return dict(j3_deg=float(high), clearance_mm=height,
                            lowest_point_world_mm=(point*1000).tolist(),
                            foot_x_from_j2_world_mm=float((point[0]-j2_axis[0])*1000),
                            lower_search_bound_active=bool(high <= lower_deg+1e-6))
            previous = angle
        return None

    def tilted_reference(self, roll_deg, pitch_deg, sink_mm):
        pose = self.walking_reference.copy()
        self.data.qpos[:] = self.reference
        mujoco.mj_forward(self.model, self.data)
        pivot = self.data.xipos[self.model.body('cad_base').id].copy()
        r, p = np.radians([roll_deg, pitch_deg])/2
        quat = np.array([np.cos(r)*np.cos(p), np.sin(r)*np.cos(p),
                         np.cos(r)*np.sin(p), -np.sin(r)*np.sin(p)])
        matrix = np.zeros(9)
        mujoco.mju_quat2Mat(matrix, quat)
        pose[:3] = pivot+matrix.reshape(3, 3)@(pose[:3]-pivot)
        pose[2] -= sink_mm/1000
        pose[3:7] = quat
        return pose


def recorded_event_checks(plant, solver, config_path):
    config = json.loads(config_path.read_text(encoding='utf-8'))
    robot = RobotController(plant)
    robot.select_profile('s_native_v6_2_6')
    ticks = {146: 3, 258: 2, 293: 3}  # Known r3 drag: 2.92 RR, 5.16 RL, 5.86 RR.
    results = []
    with tuck_experiment(**config['trajectory']), j1_experiment(config['j1_mode']):
        for tick in range(max(ticks)+1):
            time = tick*.02
            if tick == 100:
                robot.command('drive 1000 0 1', time)
            elif tick > 100 and tick % 10 == 0:
                robot.command(f'@D {tick} 1000 0', time)
            robot.tick(time)
            if tick not in ticks:
                continue
            leg = ticks[tick]
            pose = plant.data.qpos.copy()
            actual = np.degrees(pose[plant.q])
            solution = solver.solve(pose, leg, 5.)
            results.append(dict(time_s=time, leg=LEGS[leg],
                actual_j2_deg=float(actual[leg*3+1]), actual_j3_deg=float(actual[leg*3+2]),
                commanded_j3_deg=float(robot.command_target[leg*3+2]),
                exact_mesh_clearance_mm=float(solver.point(pose, leg)[2]*1000),
                required_at_same_body_pose_for_5mm=solution))
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=Path('config/experiments/post_push_j2_j3_monotonic.json'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding='utf-8'))
    parameters = load_parameters(parse_args([]))
    parameters['pack_open_circuit_voltage'] = config.get('pack_open_circuit_voltage', 11.1)
    plant = Simulation(parameters)
    solver = CushionClearance(plant)
    args.output.mkdir(parents=True, exist_ok=True)
    table = []
    for j2 in (85., 80., 75., 70., 67.8, 65., 60., 55., 50., 45., 42.8):
        row = dict(j2_deg=j2)
        for clearance in (0., 5., 10.):
            row[f'clearance_{int(clearance)}mm'] = {
                leg: solver.solve(solver.walking_reference, i, clearance, j2)
                for i, leg in enumerate(LEGS)}
        table.append(row)
    # Explicit sensitivity envelope; not a measured bound on body motion.
    robust = []
    for j2 in (80., 70., 65., 60., 55., 50., 45.):
        cases = []
        for roll in (-5., 0., 5.):
            for pitch in (-3., 0., 3.):
                pose = solver.tilted_reference(roll, pitch, 5.)
                for i, leg in enumerate(LEGS):
                    result = solver.solve(pose, i, 5., j2)
                    cases.append(dict(roll_deg=roll, pitch_deg=pitch, leg=leg, solution=result))
        worst = max((r for r in cases if r['solution']), key=lambda r:r['solution']['j3_deg'])
        robust.append(dict(j2_deg=j2, infeasible_cases=sum(r['solution'] is None for r in cases), worst=worst))
    events = recorded_event_checks(plant, solver, args.config)
    examples=[]
    for label,j2,j3 in [('Rearward lift',85.,104.),('Lower and open',65.,102.),('Forward and open',45.,100.)]:
        pose=solver.walking_reference.copy()
        pose[solver.q[10]]=np.radians(j2)
        pose[solver.q[11]]=np.radians(j3)
        point=solver.point(pose,3)
        hip=solver.data.xanchor[solver.model.joint('rr_j2').id].copy()
        knee=solver.data.xanchor[solver.model.joint('rr_j3').id].copy()
        geom=solver.feet[3]
        cushion=(solver.vertices[3]@solver.data.geom_xmat[geom].reshape(3,3).T
                 +solver.data.geom_xpos[geom])
        examples.append(dict(label=label,j2_deg=j2,j3_deg=j3,clearance_mm=float(point[2]*1000),
            hip_world_mm=(hip*1000).tolist(),knee_world_mm=(knee*1000).tolist(),
            cushion_center_world_mm=(solver.data.geom_xpos[geom]*1000).tolist(),
            cushion_vertices_world_mm=(cushion*1000).tolist()))
    result = dict(
        basis='Full CAD cushion mesh, J1=S-9 deg, S body height, horizontal floor. CAD J3 positive is flexion.',
        search_branch_deg=[76., 120.],
        warning='Geometric minima, not servo commands with guaranteed dynamic tracking or calibrated physical stops.',
        table=table, sensitivity=dict(roll_deg=5., pitch_deg=3., body_sink_mm=5., rows=robust),
        recorded_events=events,geometric_lowering_example=examples)
    (args.output/'clearance.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    j2s = np.linspace(42.8, 85., 161)
    fig, ax = plt.subplots(figsize=(10, 6))
    for clearance in (0., 5., 10.):
        needed = [solver.solve(solver.walking_reference, 3, clearance, j)['j3_deg'] for j in j2s]
        ax.plot(j2s, needed, label=f'Min J3 for {clearance:g} mm clearance')
    ax.plot([r['j2_deg'] for r in robust],
            [r['worst']['solution']['j3_deg'] for r in robust], '--',
            label='5 mm + assumed body tilt/sink envelope')
    ax.invert_xaxis()
    ax.set(xlabel='J2 angle (deg): rearward lift on left, lowering / forward swing to right',
           ylabel='Minimum CAD J3 flexion (deg)',
           title='Cushion-to-floor constraint during recovery\nGeometry only; actual J3 must reach the curve before J2 lowers')
    ax.grid(alpha=.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.output/'clearance-envelope.png', dpi=150)
    plt.close(fig)
    from scipy.spatial import ConvexHull
    fig,axes=plt.subplots(1,3,figsize=(12,5),sharey=True)
    for ax,row in zip(axes,examples):
        hip=np.array(row['hip_world_mm'])[[0,2]]
        knee=np.array(row['knee_world_mm'])[[0,2]]
        centre=np.array(row['cushion_center_world_mm'])[[0,2]]
        vertices=np.array(row['cushion_vertices_world_mm'])[:,[0,2]]
        offset=np.array([hip[0],0.])
        joints=np.array([hip,knee,centre])-offset
        vertices-=offset
        boundary=ConvexHull(vertices).vertices
        ax.fill(vertices[boundary,0],vertices[boundary,1],color='#eebd49',alpha=.8)
        ax.plot(joints[:,0],joints[:,1],'-o',lw=6,color='#297c96',markersize=7)
        ax.axhline(0,color='#313942',lw=2)
        ax.axhline(5,color='#c46536',ls='--',lw=1)
        ax.annotate(f"{row['clearance_mm']:.1f} mm",(vertices[:,0].mean(),row['clearance_mm']),
                    xytext=(vertices[:,0].mean()+35,25),arrowprops=dict(arrowstyle='->'))
        ax.set(title=f"{row['label']}\nJ2 {row['j2_deg']:g} / J3 {row['j3_deg']:g} deg",
               xlabel='X from J2 (mm); forward right',xlim=(-175,70),ylim=(-12,270))
        ax.set_aspect('equal')
        ax.grid(alpha=.2)
    axes[0].set_ylabel('Height above floor (mm)')
    fig.suptitle('RR CAD example: J2 lowers as J3 gradually opens\nLevel body at S height; 5 mm dashed line; no timing or whole-gait guarantee')
    fig.tight_layout()
    fig.savefig(args.output/'lowering-example.png',dpi=150)
    plt.close(fig)
    print(json.dumps(dict(table=[{'j2':r['j2_deg'], 'j3_5mm':r['clearance_5mm']['RR']['j3_deg'],
                                  'j3_10mm':r['clearance_10mm']['RR']['j3_deg']} for r in table],
                          sensitivity=robust, events=events), indent=2))


if __name__ == '__main__':
    main()
