"""Single-owner virtual robot console. Shared firmware C trajectories, estimated plant.

Run with mjpython for --viewer. No hardware transport is imported or opened.
The protocol adapter is intentionally separate from the physics plant.
"""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
from simulation.mujoco.runtime.stow_policy import LANDING, FOLDED, FOLD_SECONDS, RELEASE_TORQUE_AT_STOW, direct_unfold, duration as stow_duration, folded_geometry, frame as stow_frame, attitude_ok as stow_attitude_ok, pose_frame as shared_pose_frame
import argparse
import collections
import json
import math
import selectors
import socket
import subprocess
import time
import tempfile
import sys
from pathlib import Path

import numpy as np
import mujoco
from simulation.mujoco.runtime.cad_gait import CAD, KEYS
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.realtime_pacer import PhysicsPacer
from simulation.mujoco.runtime.balance_controller import BalanceController
from simulation.mujoco.runtime.heading_controller import HeadingController
from simulation.mujoco.runtime.drive_controller import step as shared_drive_step
import copy
from simulation.mujoco.runtime.bno055_emulator import BNO055Config, BNO055Emulator, FirmwareAttitudeFilter
from simulation.mujoco.runtime.gait_profiles import foot_targets, load_profiles, PROFILE_FILE


class RobotController:
    """Console/session adapter; C gait equations are shared, STM32 HAL is not emulated."""
    def __init__(self, plant):
        self.plant = plant
        self.target = np.degrees(plant.desired).copy()
        self.stand_target=getattr(plant,'stand_target',self.target).copy()
        self.pose = 'stand'
        self.torque = True
        self.safety = 'ok'
        self.motion = None
        self.transition = None
        self.stow_prepared = False
        self.stow_path = False
        self.stow_queue = []
        self.phase = 0.
        self.turn_assist = 0.
        self.linear = self.yaw = 0.
        self.request = (0., 0.)
        self.sequence = 0
        self.last_packet = 0.
        self.elapsed = 0.
        self.out = []
        self.profiles = load_profiles() if PROFILE_FILE.exists() else {}
        self.deployed_profiles = copy.deepcopy(self.profiles)
        from simulation.mujoco.runtime.s_native_gait import NAME, PROFILES
        if plant.p.get('aligned_standing_pose') and plant.p.get('foot_cushion'):
            self.profiles.update(copy.deepcopy(PROFILES))
        from simulation.mujoco.runtime.support_j1 import SupportJ1
        self.support_j1=SupportJ1()
        from simulation.mujoco.runtime.position_wbc import PositionWBC, EncoderChannel
        self.position_wbc=PositionWBC(plant.model)
        self.encoder_channel=EncoderChannel()
        from simulation.mujoco.runtime.gait_tracking import GaitTracking
        self.tracking=GaitTracking(plant.policy)
        self.tracking_enabled=plant.p.get('tracking_feedback_enabled',False)
        self.support_shift=None
        self.footstep_tracker=None
        self.profile = NAME if NAME in self.profiles else json.loads(PROFILE_FILE.read_text()).get('default','legacy')
        self.stopping_reason = None
        self.pending_profile = None
        self.imu = BNO055Emulator(BNO055Config(**plant.p.get("bno055", {})))
        self.attitude_filter = FirmwareAttitudeFilter()
        self.imu_reading = None
        self.balance = BalanceController(plant.policy)
        self.heading = HeadingController(plant.policy)
        self.command_target = self.target.copy()
        self.flight_frames = collections.deque(maxlen=1500)
        self.flight_commands = collections.deque(maxlen=512)
        self.incident_directory = None
        self.last_incident = None
        self.realtime_factor = None
        self.plant.sensor_observer = self.observe_imu

    def load_experimental_profiles(self, path):
        """Opt-in simulator experiments; never replace deployed profile names."""
        profiles=json.loads(Path(path).read_text())['profiles']
        for name, profile in profiles.items():
            if name == 'legacy' or name in self.profiles:
                raise ValueError('Experimental profile must have a new name: '+name)
            if profile.get('balance_base') not in self.deployed_profiles:
                raise ValueError('Experimental profile needs a deployed balance_base')
            for phase in np.linspace(0,1,101):
                foot_targets(profile['params'],phase*profile['params'][0],1,profile['family'])
                if profile.get('measured_swing'):
                    from simulation.mujoco.runtime.measured_swing import targets as measured_targets
                    for linear,yaw in ((1,0),(0,-1),(0,1)):
                        params=profile.get('turn_reverse_params',profile['params']) if yaw else profile['params']
                        measured_targets(params,phase,1,linear,yaw,profile['measured_swing'])
        self.profiles.update(profiles)

    def observe_imu(self, model, data):
        r = data.xmat[model.body('robot').id].reshape(3,3)
        velocity = np.zeros(6)
        # XBODY uses the robot frame, BODY would use its inertia principal axes.
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY,
                                model.body('robot').id, velocity, 1)
        self.imu.advance(float(data.time),
            math.degrees(math.atan2(r[2,1],r[2,2])),
            math.degrees(math.asin(float(np.clip(-r[2,0],-1,1)))),
            math.degrees(math.atan2(r[1,0],r[0,0])), gyro_body_rad_s=velocity[:3])

    def imu_diagnostic(self):
        return dict(sensor='BNO055', mode='IMUPLUS', emulated=True,
                    feedback='active' if self.balance.applied else 'suspended', balance=self.balance.diagnostic(), heading=self.heading.diagnostic(), reading=self.imu_reading,
                    filtered_tenths=self.attitude_filter.filtered,
                    filtered_rate_tenths_s=self.attitude_filter.rate,
                    failures=self.attitude_filter.failures,
                    config=vars(self.imu.config),last_incident=self.last_incident,realtime_factor=self.realtime_factor)

    def reply(self, text='OK'):
        self.out.append(text + '\r\n# ')

    def balance_state(self):
        if self.profile=='attitudepd':
            if not self.body_stabilizer.enabled:return 'off'
            return 'active' if self.body_stabilizer.diagnostic.get('status')=='active' else 'suspended'
        return 'active' if self.balance.applied else 'suspended' if self.balance.enabled else 'off'

    def drain(self):
        result = ''.join(self.out).encode()
        self.out.clear()
        return result

    def blend(self, target, completion, duration=1.):
        self.transition = (self.target.copy(), np.array(target), 0., duration, completion)
        self.pose = 'custom'

    def blend_pose(self,target,completion):
        self.capture_current_target()
        _,duration=shared_pose_frame(self.target,target)
        self.blend(target,completion,max(.02,duration))

    def capture_current_target(self):
        """Restart from the measured pose without restoring an old queued goal."""
        held=self.plant.data.qpos[self.plant.q].copy()
        self.target=np.degrees(held)
        self.command_target=self.target.copy()
        self.plant.desired=held.copy();self.plant.filtered=held.copy()
        self.plant.target_velocity[:]=0
        self.plant.delay=collections.deque([held.copy() for _ in range(len(self.plant.delay))])

    def stop(self, reason='requested'):
        if self.stow_path or (self.motion is None and self.transition is not None):
            self.stow_queue.clear(); self.transition=None
            # Flush queued servo targets, not physical state/velocity. Subsequent
            # physics dissipates motion while the position controller holds here.
            self.capture_current_target()
            self.pose='stow-paused' if self.stow_path else 'custom'
            self.reply('STOPPED: '+reason)
            return
        if self.motion is None or self.stopping_reason:
            return
        if (self.motion[0] in ('drive','profile') and not self.transition and
                (self.elapsed > .1 or self.profiles.get(self.profile,{}).get('stop_on_next_placement'))):
            self.stopping_reason = reason
            self.request = (0.,0.)
            if self.profiles.get(self.profile,{}).get('entry_sequence')=='fr_double':
                self.s_native_gait.begin_stop()
            return
        self.finish_stop(reason)

    def finish_stop(self, reason):
        drive = self.motion[0] == 'drive'
        if self.profile=='attitudepd':
            self.target=self.command_target.copy()
            self.body_stabilizer.reset()
        if self.profile=='arcturn' and any(value is not None and value is not False
                for key,value in self.plant.p.items() if key.startswith('arc_')):
            # Experimental residuals are applied after nominal IK. Blend from
            # the last commanded pose, never jump back to uncorrected nominal.
            self.target=self.command_target.copy()
        self.motion = None
        self.stopping_reason = None
        self.request = (0., 0.)
        completion = f'$SPOTDRIVE stopped reason={reason}\r\nOK' if drive else ('OK' if reason=='complete' else 'STOPPED: '+reason)
        self.blend(self.stand_target, completion)

    def limited_yaw(self, value):
        profile=self.profiles.get(self.profile,{})
        limit=float(profile.get('support_shift',{}).get('turn_input_limit',profile.get('turn_input_limit',.5)))
        if not math.isfinite(limit) or not .5<=limit<=1:
            raise ValueError('Invalid experimental turn input limit')
        return float(np.clip(value,-limit,limit))

    def limited_linear(self, value):
        # Fast trot / high-step full reverse strides lost stability in the 60 s matrix.
        # Its validated reverse envelope is 60%; forward remains unrestricted.
        return max(-.6,value) if self.profile in ('trot','highstep','lift','imu','level','level15','joint','jointfast','jointsport') else value

    def select_profile(self, name):
        if name != 'legacy' and name not in self.profiles:
            raise ValueError('unknown profile: '+name)
        if self.motion or self.transition:
            raise ValueError('stop before changing profile')
        self.profile = name
        if name=='attitudepd':
            from simulation.mujoco.runtime.body_stabilizer import BodyStabilizer
            if not hasattr(self,'body_stabilizer'):
                self.body_stabilizer=BodyStabilizer(self.plant.p.get('body_stabilizer'))
            self.body_stabilizer.reset()
        self.balance.integral[:]=0;self.balance.correction[:]=0
        self.support_j1.reset()
        self.position_wbc.delta[:]=0;self.position_wbc.confidence[:]=0;self.position_wbc.scale=1.
        self.encoder_channel.queue.clear()
        if self.support_shift:self.support_shift.reset()
        if self.footstep_tracker:self.footstep_tracker.reset()

    def disconnected(self):
        self.stop('disconnect')
        # Completion output belongs to the old session, never the next owner.
        self.out.clear()

    def command(self, line, now):
        self.flight_commands.append((float(self.plant.data.time),line))
        words = line.split()
        if not words:
            return
        cmd = words[0]
        try:
            if cmd == 'identity':
                self.reply('$SPOTBACKEND backend=sim protocol=1 physics=estimated controller=sim-profiles adapter=python')
            elif cmd in ('echo', 'log'):
                if line not in ('echo off', 'echo on') and not (len(words)==3 and words[:2]==['log','time'] and words[2].isdigit()):
                    raise ValueError('unsupported simulator command')
                self.reply()
            elif cmd=='stabilize' or cmd=='@B':
                modes={'0':'off','1':'on','2':'status'} if cmd=='@B' else {'on':'on','off':'off','status':'status'}
                if len(words)!=2 or words[1] not in modes:raise ValueError('stabilize on|off|status')
                if not hasattr(self,'body_stabilizer'):
                    from simulation.mujoco.runtime.body_stabilizer import BodyStabilizer
                    self.body_stabilizer=BodyStabilizer(self.plant.p.get('body_stabilizer'))
                action=modes[words[1]]
                if action!='status':self.body_stabilizer.enabled=action=='on'
                status=self.body_stabilizer.diagnostic.get('status','idle')
                self.reply(f'$STABILIZE enabled={int(self.body_stabilizer.enabled)} status={status} policy={self.profile} rate_hz=50')
            elif cmd == 'syncstate':
                error = round(float(np.max(np.abs(self.command_target-np.degrees(self.plant.data.qpos[self.plant.q]))))*4096/360)
                self.reply(f'$SPOTSTATE pose={self.pose} error={error} torque={"on" if self.torque else "off"} safety={self.safety} balance={self.balance_state()} heading={"on" if self.heading.enabled else "off"} rev=s-native-v6-1-v62-sim caps=trot5,simprofiles,gaitprofiles,{",".join(name for name, profile in self.profiles.items() if profile.get("s_native")) + "," if "s_native_v1" in self.profiles else ""}arcsupport,centerpivot,attitudepd,bno055emu,simbalance,balancecontrol,headinghold,stow imu=bno055-emulated backend=sim physics=estimated fall_test={"on" if self.plant.p.get("sim_allow_fall") else "off"} profile={self.profile} reverse_limit={600 if self.profile in ("trot","highstep","lift","imu","level","level15","joint","jointfast","jointsport") else 1000}')
            elif cmd == 'read' and words == ['read', '1']:
                self.reply(f'ID 1 voltage={round(self.plant.voltage*1000)}mV source=simulated')
            elif cmd == 'tracking':
                if len(words)!=2 or words[1] not in ('on','off') or self.motion or self.transition:
                    raise ValueError('tracking on|off requires idle')
                self.tracking_enabled=words[1]=='on';self.reply('OK tracking experimental')
            elif cmd == 'trackingdiag':
                d=self.tracking.diagnostic
                self.reply(f"$TRACKING enabled={int(self.tracking_enabled)} rate={round(self.tracking.rate*1000)} error10={round(d.get('peak_error_deg',0)*10)} oldest_ms={int(d.get('oldest_ms',0))} blocked_ms={int(d.get('blocked_ms',0))} fault={d.get('fault',0)} contact=unobserved")
            elif cmd == 'locomotiondiag':
                h=self.heading.diagnostic();reading=self.imu_reading
                self.reply(f"$LOCOMOTION profile={self.profile} heading={'on' if h['enabled'] else 'off'} yaw10={reading['yaw_tenths'] if reading else 0} valid={int(reading is not None)} error10={round(h['error_deg']*10)} correction={round(h['correction']*1000)} late=0 fault={int(self.safety!='ok')}")
            elif cmd == 'imudiag':
                self.reply('$SIMIMU '+json.dumps(self.imu_diagnostic(), separators=(',', ':')))
            elif cmd in ('gaitdiag', 'baldiag', 'targets'):
                self.reply('$SIMSTATE '+json.dumps(dict(self.plant.row(), imu=self.imu_diagnostic()), separators=(',', ':')))
            elif cmd == '@D':
                seq, linear, yaw = map(int, words[1:])
                if not 0 <= seq <= 0xffffffff or max(abs(linear),abs(yaw)) > 1000:
                    raise ValueError('invalid drive packet')
                delta = (seq-self.sequence) & 0xffffffff
                if self.motion and self.motion[0]=='drive' and not self.stopping_reason and 0 < delta < 0x80000000:
                    self.sequence=seq; self.last_packet=now
                    self.request=(self.limited_linear(linear/1000), self.limited_yaw(yaw/1000))
            elif cmd == '@S':
                seq = int(words[1])
                if len(words)!=2 or not 0 <= seq <= 0xffffffff:
                    raise ValueError('invalid stop packet')
                if 0 < ((seq-self.sequence)&0xffffffff) < 0x80000000:
                    self.sequence=seq
                    if self.motion: self.stop()
                    elif not self.transition: self.reply('$SPOTDRIVE stopped reason=already-stopped\r\nOK')
            elif cmd == '\x03':
                self.stop('interrupt')
            elif cmd=='hold' and self.stow_path:
                self.stop('hold')
            elif self.motion or self.transition:
                raise ValueError('busy; stop and wait for prompt')
            elif self.stow_path and cmd not in ('stow','landing','recover','relax'):
                raise ValueError('unfold with landing before other commands')
            elif cmd=='stowcheck' and len(words)==1:
                self.reply('STOWCHECK: shared signed STS3250 encoder; simulated mode=0 resolution=1; no motion')
            elif cmd=='stow' and len(words)==1:
                self.safety='ok'  # Fresh command retries; active IMU checks still stop faults.
                already_stow=self.stow_path
                if already_stow and not self.torque:self.capture_current_target()
                self.stow_path=True;self.plant.stow_active=True;self.torque=True
                self.balance.correction[:]=0;self.balance.integral[:]=0
                if self.pose=='landing' or already_stow:
                    self.blend(FOLDED,'OK stow',FOLD_SECONDS)
                else:
                    self.stow_queue=[(FOLDED,'OK stow',FOLD_SECONDS)]
                    self.blend_pose(LANDING,'STOW landing')
            elif cmd=='landing' and len(words)==1 and (self.stow_path or folded_geometry(np.degrees(self.plant.data.qpos[self.plant.q]))):
                self.capture_current_target()
                self.torque=True
                self.stow_path=True;self.plant.stow_active=True
                self.stow_prepared=False
                self.blend(LANDING,'OK landing',stow_duration(self.target))
            elif cmd == 'heading' and len(words)==2 and words[1] in ('on','off'):
                self.heading.enabled=words[1]=='on'; self.heading.update(None,0,0,0); self.reply()
            elif cmd in ('balance','simbalance') and len(words)==2 and words[1] in ('on','off'):
                if self.profile=='attitudepd':self.body_stabilizer.enabled=words[1]=='on'
                else:self.balance.enabled=words[1]=='on'
                self.reply()
            elif cmd in ('gaitprofile','simprofile') and len(words)==2:
                self.select_profile(words[1]); self.reply('OK profile='+self.profile)
            elif cmd in ('gaitprofiles','simprofiles') and len(words)==1:
                self.reply(('$GAITPROFILES ' if cmd=='gaitprofiles' else '$SIMPROFILES ')+','.join(['legacy',*self.profiles]))
            elif cmd == 'simwalk' and len(words)==2:
                duration=float(words[1])
                if not math.isfinite(duration) or not 1<=duration<=60:
                    raise ValueError('duration must be 1..60 seconds')
                period=1.8 if self.profile=='legacy' else self.profiles[self.profile]['params'][0]
                self.begin(('profile',duration,period,1),now)
                self.request=(1.,0.)
            elif cmd == 'relax' and len(words)==1:
                self.torque=False; self.pose='custom'; self.reply()
            elif cmd in ('stand', 'stand11', 'landing', 'hold', 'recover') and len(words)==1:
                if cmd=='recover':
                    self.safety='ok'; self.reply(); return
                if self.safety!='ok':
                    raise ValueError('safety latched; recover first')
                self.torque=True
                target = {'stand':self.stand_target,'stand11':[0,0,0]*4,'landing':[0,40,130]*4,
                          'hold':np.degrees(self.plant.data.qpos[self.plant.q])}[cmd]
                if cmd in ('stand','stand11','landing'):self.blend_pose(target,'OK '+cmd)
                else:self.blend(target,'OK '+cmd)
            elif cmd == 'drive':
                linear,yaw,seq=map(int,words[1:])
                if max(abs(linear),abs(yaw))>1000 or not 0<=seq<=0xffffffff:
                    raise ValueError('invalid drive input')
                self.begin(('drive',), now)
                self.sequence=seq; self.last_packet=now
                self.request=(self.limited_linear(linear/1000),self.limited_yaw(yaw/1000))
                self.out.append(f'$SPOTDRIVE started seq={seq} watchdog=800ms\r\n')
            elif cmd in ('trot5','trot4','trot4back','turn','crab'):
                direction=1
                if cmd in ('turn','crab'):
                    if words[1] not in ('left','right'): raise ValueError('invalid direction')
                    direction=1 if words[1]=='left' else -1
                    cycles,period=map(int,words[2:])
                else:
                    cycles,period=map(int,words[1:])
                if not 1<=cycles<=100 or not 400<=period<=10000:
                    raise ValueError('cycles 1..100, period 400..10000 ms required')
                self.begin((cmd, cycles*period/1000, period/1000, direction), now)
            else:
                raise ValueError('unsupported simulator command: '+cmd)
        except (ValueError, IndexError, TypeError) as exc:
            self.reply('ERROR: '+str(exc))

    def active_profile_params(self):
        profile=self.profiles[self.profile]
        forward=np.asarray(profile['params'],dtype=float)
        if 'turn_reverse_params' in profile:
            # Blend continuously through low forward input; phase is never reset.
            weight=self.plant.policy.smootherstep(float(np.clip(self.linear/.5,0,1)))
            base=np.asarray(profile['turn_reverse_params'],dtype=float)
            forward=base+(forward-base)*weight
        config=profile.get('support_shift',{})
        if 'turn_period_s' in config:
            period=float(config['turn_period_s'])
            if not math.isfinite(period) or not .6<=period<=forward[0]:
                raise ValueError('Turn period must be 0.6s or longer and no slower than forward')
            fraction=abs(self.yaw)/max(abs(self.linear)+abs(self.yaw),1e-9)
            forward=forward.copy();forward[0]+=fraction*(period-forward[0])
        return forward

    def gait(self, phase, amplitude):
        policy=self.plant.policy; kind=self.motion[0]
        if kind in ('drive','profile') and self.profile != 'legacy':
            profile=self.profiles[self.profile]
            if profile.get('s_native'):
                self.s_native_gait.prepare_support(*self.request)
                return dict(zip(KEYS,self.s_native_gait.targets(phase,amplitude,self.linear,self.yaw)))
            params=self.active_profile_params().copy()
            if profile.get('measured_swing'):
                from simulation.mujoco.runtime.measured_swing import targets as measured_targets
                values=measured_targets(params,phase,amplitude,self.linear,self.yaw,profile['measured_swing'])
                if profile.get('pivot_turn'):
                    from simulation.mujoco.runtime.pivot_turn import PivotTurn
                    weight=policy.smootherstep(float(np.clip(1-abs(self.linear)/.6,0,1)))
                    if not profile['pivot_turn'].get('retain_entry_foot_xy',False):weight*=policy.smootherstep(min(1.,abs(self.yaw)/.25))
                    if weight>0:
                        if not hasattr(self,'pivot_turn'):self.pivot_turn=PivotTurn(self.plant.model)
                        pivot=self.pivot_turn.plan(params,phase,amplitude,self.yaw,profile['pivot_turn'],nominal=values)
                        values+=weight*(pivot-values)
                return dict(zip(KEYS,values))
            if profile.get('footstep_tracking'):
                if self.footstep_tracker is None:
                    from simulation.mujoco.runtime.footstep_tracker import FootstepTracker
                    self.footstep_tracker=FootstepTracker(self.plant.model)
                values=self.footstep_tracker.plan(params,phase,amplitude,self.linear,self.yaw,profile['footstep_tracking'])
                return dict(zip(KEYS,values))
            if profile.get('support_shift'):
                if self.support_shift is None:
                    from simulation.mujoco.runtime.support_shift import SupportShift
                    self.support_shift=SupportShift(self.plant.model)
                values=self.support_shift.plan(params,phase,amplitude,self.linear,self.yaw,profile['support_shift'])
                return dict(zip(KEYS,values))
            # Reduce stride without collapsing swing height and causing toe drag.
            adaptive=self.position_wbc.scale if profile.get('position_wbc') else self.support_j1.scale if profile.get('support_j1') else 1.
            params[2]*=adaptive
            values=foot_targets(params,phase*params[0],amplitude,
                                profile['family'],self.linear,self.yaw)
            return dict(zip(KEYS,values))
        if kind in ('drive','profile'):
            return policy.drive_walk_targets(phase,amplitude,self.linear,self.yaw)[0]
        if kind=='trot5': return policy.trot5_targets(phase,amplitude)[0]
        if kind in ('trot4','trot4back'):
            return policy.trot4_direction_targets(phase,amplitude,1 if kind=='trot4' else -1)[0]
        return getattr(policy,kind+'_targets')(phase,amplitude,self.motion[3])[0]

    def begin(self, motion, now):
        if self.stow_path:raise ValueError('unfold with landing before gait')
        if self.safety!='ok' or not self.torque:
            raise ValueError('stand/recover required before gait')
        self.attitude_filter = FirmwareAttitudeFilter()
        self.balance.integral[:]=0; self.balance.correction[:]=0
        self.balance.saturated=False
        if self.profile=='attitudepd':self.body_stabilizer.reset()
        for name in ('arc_frame','arc_attitude','arc_preload','arc_transfer','arc_dynamic','arc_support_state','arc_shift_state',
                     'arc_com_state','arc_tripod_state','arc_observer','arc_sensor','arc_sensor_available'):
            if hasattr(self,name):delattr(self,name)
        if self.profiles.get(self.profile,{}).get('pivot_turn',{}).get('retain_entry_foot_xy',False):
            from simulation.mujoco.runtime.pivot_turn import PivotTurn
            self.pivot_turn=PivotTurn(self.plant.model);self.pivot_turn.set_entry(self.command_target)
        self.motion=motion; self.elapsed=self.phase=self.linear=self.yaw=0.
        self.s_gait_frame=None
        if self.profiles.get(self.profile,{}).get('s_native'):
            from simulation.mujoco.runtime.s_native_gait import SNativeGait
            self.s_native_gait=SNativeGait(self.plant.model,self.stand_target,self.profiles[self.profile])
            self.phase=self.profiles[self.profile].get('start_phase',.25)
        if self.plant.p.get('aligned_standing_pose',False):
            from simulation.mujoco.runtime.standing_pose import StandingGaitFrame
            self.s_gait_frame=StandingGaitFrame(self.plant.model,self.stand_target)
        self.turn_assist=0.
        self.stopping_reason=None
        if self.s_gait_frame is None:
            neutral=self.gait(0,0)
            self.blend([neutral[k] for k in KEYS], None)
        elif not np.allclose(self.target,self.stand_target,atol=.5):
            # Non-standing poses still require a controlled return to S.
            self.blend(self.stand_target,None)
        else:
            self.transition=None

    def rebase_gait_on_s(self):
        if self.profiles.get(self.profile,{}).get('s_native'):return
        if not getattr(self,'s_gait_frame',None) or not self.motion:return
        if self.motion[0]=='drive' and (self.profile=='legacy' or
                self.profiles.get(self.profile)==self.deployed_profiles.get(self.profile)):
            import ctypes
            from simulation.mujoco.runtime.drive_controller import NAMES
            out=(ctypes.c_float*12)()
            fn=self.plant.policy._library.spot_locomotion_targets
            fn.argtypes=(ctypes.c_int,*([ctypes.c_float]*4),ctypes.POINTER(ctypes.c_float))
            fn.restype=ctypes.c_int
            if not fn(NAMES.index(self.profile),self.nominal_phase,0.,self.linear,self.yaw,out):
                raise ValueError('Cannot obtain S gait reference')
            neutral=np.array(out)
        else:
            result=self.gait(self.nominal_phase,0.)
            neutral=np.array([result[k] for k in KEYS])
        self.target=self.s_gait_frame.targets(self.target,neutral)

    def tick(self, now):
        if self.motion and self.motion[0]=='drive' and not self.stopping_reason and now-self.last_packet>.8:
            self.stop('watchdog')
        if self.motion is None or self.transition:
            self.heading.update(None,0,0,0)
        if self.transition:
            start,end,elapsed,duration,completion=self.transition
            elapsed=min(duration,elapsed+.02)
            t=self.plant.policy.smootherstep(elapsed/duration)
            if self.stow_path and completion in ('OK stow','OK landing'):
                if completion=='OK landing' and duration>FOLD_SECONDS and elapsed>FOLD_SECONDS:
                    ready=np.asarray(stow_frame(start,False,FOLD_SECONDS)[0])
                    measured=np.degrees(self.plant.data.qpos[self.plant.q]).copy()
                    if np.max(abs(measured-ready))>5:
                        self.stop('unfold preparation tracking error')
                        return
                    start=measured;duration=FOLD_SECONDS;elapsed=.02
                    self.stow_prepared=True
                self.target=np.asarray((direct_unfold(start,elapsed) if completion=='OK landing' and self.stow_prepared else stow_frame(start,completion=='OK stow',elapsed))[0])
            elif completion in ('OK stand','OK stand11','OK landing','STOW landing'):
                self.target=np.asarray(shared_pose_frame(start,end,elapsed)[0])
            else:
                self.target=start+(end-start)*t
            self.transition=(start,end,elapsed,duration,completion)
            if elapsed>=duration:
                self.transition=None
                if self.stow_queue:
                    self.blend(*self.stow_queue.pop(0))
                elif completion:
                    if self.stow_path and completion in ('OK stow','OK landing'):
                        error=float(np.max(abs(np.degrees(self.plant.data.qpos[self.plant.q])-end)))
                        if error>5:
                            self.capture_current_target()
                            if completion=='OK stow':self.torque=False
                            completion=f'ERROR: Stow posture target not reached ({error:.1f} deg)'
                    if self.stow_path and completion=='OK landing':
                        self.stow_path=False;self.plant.stow_active=False
                    if completion=='OK stow' and RELEASE_TORQUE_AT_STOW:
                        # Do not keep pushing toward a position after the legs
                        # settle into contact. Gravity/contact now support them.
                        self.torque=False
                    self.pose=completion[3:] if completion in ('OK stand','OK stand11','OK landing','OK stow') else ('stand' if np.allclose(end,self.stand_target) else 'custom')
                    if self.stow_path and completion.startswith('ERROR:'):
                        self.pose='stow-paused'
                    self.reply(completion)
        elif self.motion:
            self.nominal_phase=self.phase  # target time, before the gait advances
            if self.motion[0]=='drive' and (self.profile=='legacy' or (self.profile in self.deployed_profiles and self.profiles[self.profile]==self.deployed_profiles[self.profile])):
                self.target=shared_drive_step(self)
            else:
                self.elapsed+=.02
                self.linear=self.plant.policy.drive_slew(round(self.linear*1000),round(self.request[0]*1000))/1000
                correction=self.heading.update(self.imu_reading,self.request[0],self.request[1],self.yaw,
                    permitted=self.motion[0]=='drive' and not self.stopping_reason and self.safety=='ok')
                self.yaw=self.plant.policy.drive_slew(round(self.yaw*1000),round((self.request[1]+correction)*1000))/1000
                result=self.gait(self.phase,self.plant.policy.smootherstep(min(1,self.elapsed)))
                self.target=np.array([result[k] for k in KEYS])
                period=self.plant.policy.drive_period_ms(round(self.linear*1000),round(self.yaw*1000))/1000 if self.motion[0]=='drive' else self.motion[2]
                if self.motion[0] in ('drive','profile') and self.profile != 'legacy':
                    period=self.active_profile_params()[0]*(1.35-.35*min(1,abs(self.linear)+abs(self.yaw)))
                self.phase=(self.phase+.02/period)%1
            try:
                self.rebase_gait_on_s()
            except ValueError as error:
                self.capture_current_target()
                self.motion=None;self.transition=None;self.request=(0.,0.)
                self.safety='tracking';self.pose='custom'
                self.reply('ERROR: '+str(error))
            native_stop_ready=(self.profiles.get(self.profile,{}).get('entry_sequence')!='fr_double'
                               or self.s_native_gait.stop_ready)
            if self.motion and self.stopping_reason and max(abs(self.linear),abs(self.yaw))<=.008 and native_stop_ready:
                self.finish_stop(self.stopping_reason)
            elif self.motion and self.motion[0]!='drive' and self.elapsed>=self.motion[1]:
                if self.motion[0]=='profile': self.stop('complete')
                else: self.motion=None; self.blend(self.stand_target,'OK')
        if self.pending_profile and not self.motion and not self.transition:
            self.select_profile(self.pending_profile); self.pending_profile=None
        returning_stand = self.transition is not None and np.allclose(self.transition[1],self.stand_target)
        permitted = self.torque and self.safety=='ok' and (self.pose=='stand' or self.motion is not None or returning_stand)
        if self.stow_path or (self.transition and self.transition[4] in ('OK landing','OK stand11','OK hold')):
            permitted=False
        # Delayed angle derivatives amplified full-speed gait oscillations.
        # Use bounded slow PI during all motion/pose transitions, D only at rest.
        moving = self.motion is not None or self.transition is not None
        self.balance.kp,self.balance.kd,self.balance.ki = (.1,0.,.03) if moving else (.25,.015,.15)
        self.balance.standing = self.motion is None and self.transition is None and self.pose=='stand'
        self.balance.profile=self.profiles.get(self.profile,{}).get("balance_base",self.profile)
        self.balance.phase=self.phase
        self.balance.moving=self.motion is not None and self.transition is None
        self.balance.linear,self.balance.yaw=self.linear,self.yaw
        if self.stow_path:
            # Experimental unwrapped posture: the gait balance stage clamps J2
            # even when disabled. Preserve this posture's separately checked path.
            self.balance.integral[:]=0;self.balance.correction[:]=0
            self.balance.applied=False;self.balance.saturated=False
            self.command_target=self.target.copy()
        elif self.profiles.get(self.profile,{}).get('s_native'):
            # Native CAD contact IK must not be remapped by the legacy two-link balance.
            self.balance.integral[:]=0;self.balance.correction[:]=0
            self.balance.applied=False;self.balance.saturated=False
            self.command_target=self.target.copy()
        elif self.profile=='attitudepd':
            self.balance.integral[:]=0;self.balance.correction[:]=0
            self.balance.applied=False;self.balance.saturated=False
            gait_moving=self.motion is not None and self.transition is None
            params=self.active_profile_params()
            period=params[0]*(1.35-.35*min(1,abs(self.linear)+abs(self.yaw)))
            frame=dict(phase=getattr(self,'nominal_phase',self.phase),period_s=period,duty=params[1],moving=gait_moving)
            try:
                self.command_target=self.body_stabilizer.apply(self.target,frame,self.imu_reading,
                    float(self.plant.data.time),permitted=permitted and gait_moving)
            except ValueError as error:
                # Retain the entire last valid command; never publish partial IK
                # or cut torque merely because a correction is infeasible.
                self.motion=None;self.transition=None;self.request=(0.,0.)
                self.target=self.command_target.copy();self.safety='planner';self.pose='custom'
                self.reply('ERROR: '+str(error)+'; command held, torque preserved')
        elif self.plant.p.get('arc_attitude_trial') and self.profile=='arcturn' and self.motion and not self.transition and hasattr(self,'arc_frame'):
            # The CAD gait must not pass through the legacy two-link IK balance.
            self.balance.integral[:]=0;self.balance.correction[:]=0
            self.balance.applied=False;self.balance.saturated=False
            if not hasattr(self,'arc_attitude'):
                from simulation.mujoco.runtime.arc_attitude import ArcAttitude
                self.arc_attitude=ArcAttitude(self.plant.policy)
            available=bool(permitted and self.balance.enabled and self.imu_reading is not None
                and self.imu_reading['age_ms']<=100 and self.attitude_filter.failures==0)
            available=available and max(abs(self.linear),abs(self.yaw))>.001
            self.command_target=self.arc_attitude.apply(self.target,self.arc_frame,getattr(self,'arc_sensor',self.attitude_filter),
                available and getattr(self,'arc_sensor_available',True),self.plant.p['arc_attitude_trial'])
        else:
            self.command_target = self.balance.apply(self.target,self.attitude_filter,
                self.imu_reading is not None and self.attitude_filter.failures==0,permitted)
        if self.plant.p.get('arc_balance_trial') and self.profile=='arcturn' and self.motion and not self.transition and permitted:
            import ctypes
            fp=ctypes.POINTER(ctypes.c_float);fn=self.plant.policy._library.spot_arc_balance
            fn.argtypes=(fp,*([ctypes.c_float]*6),fp);fn.restype=ctypes.c_int
            out=(ctypes.c_float*12)();config=self.plant.p.get('arc_trial',[.02,.5,.04,0])
            phase=self.phase-.02/(1.44-.24*abs(self.yaw)/max(abs(self.linear)+abs(self.yaw),1e-9))
            gain,swing_gain=self.plant.p['arc_balance_trial']
            roll,pitch=np.radians(np.array(self.attitude_filter.filtered)/10)
            if not fn((ctypes.c_float*12)(*self.target),roll,pitch,phase,config[1],gain,swing_gain,out):
                raise ValueError('Cartesian balance target infeasible')
            self.command_target=np.array(out)
        if self.plant.p.get('arc_transfer_trial') and self.profile=='arcturn' and self.motion and not self.transition and permitted and hasattr(self,'arc_frame'):
            if not hasattr(self,'arc_transfer'):
                from simulation.mujoco.runtime.arc_preload import ArcTransfer
                self.arc_transfer=ArcTransfer(self.plant.policy)
            transfer_config=list(self.plant.p['arc_transfer_trial'])
            transfer_config[2]*=min(1.,abs(self.linear)+abs(self.yaw))
            self.command_target+=self.arc_transfer.correction(self.target,self.arc_frame,transfer_config)
        elif hasattr(self,'arc_transfer'):
            self.arc_transfer.reset()
        if self.plant.p.get('arc_preload_trial') and self.profile=='arcturn' and self.motion and not self.transition and permitted:
            if not hasattr(self,'arc_preload'):
                from simulation.mujoco.runtime.arc_preload import ArcPreload
                self.arc_preload=ArcPreload(self.plant.policy)
            gain,lead=self.plant.p['arc_preload_trial']
            duty=self.plant.p.get('arc_trial',[.02,.5,.04,0])[1]
            self.command_target+=self.arc_preload.correction(self.target,self.phase+lead/1.2,duty,gain)
        elif hasattr(self,'arc_preload'):
            self.arc_preload.reset()
        wbc_config=self.profiles.get(self.profile,{}).get('position_wbc')
        if wbc_config:
            encoder_sample=self.encoder_channel.read(np.degrees(self.plant.data.qpos[self.plant.q]))
            wbc_enabled=self.balance.enabled and self.balance.applied and self.balance.moving and permitted
            wbc_weight=1.
            if wbc_config.get('turn_only',False):
                smooth=self.plant.policy.smootherstep
                wbc_weight=smooth(min(1.,abs(self.yaw)/.25))*smooth(float(np.clip(1-abs(self.linear)/.6,0,1)))
            wbc_target=self.position_wbc.apply(self.target,encoder_sample,self.attitude_filter,
                self.phase,self.active_profile_params()[1],wbc_config,wbc_enabled and wbc_weight>0)
            if wbc_enabled and encoder_sample is not None:
                self.command_target+=wbc_weight*(wbc_target-self.command_target)
        shift_config=self.profiles.get(self.profile,{}).get('support_shift')
        if shift_config and self.support_shift:
            encoder_sample=self.encoder_channel.read(np.degrees(self.plant.data.qpos[self.plant.q]))
            enabled=self.balance.enabled and self.balance.applied and self.balance.moving and permitted
            if enabled:
                self.command_target=self.support_shift.feedback(self.target,encoder_sample,self.attitude_filter,
                    self.active_profile_params(),shift_config,enabled)
            else:self.support_shift.reset()
        support_config=self.profiles.get(self.profile,{}).get('support_j1')
        if support_config:
            self.command_target=self.support_j1.apply(self.target,self.command_target,
                self.attitude_filter,self.phase,self.active_profile_params()[1],support_config,
                self.balance.enabled and self.balance.applied and self.balance.moving and permitted)
        else:self.support_j1.reset()
        tracker_config=self.profiles.get(self.profile,{}).get('footstep_tracking')
        if tracker_config and self.footstep_tracker:
            encoder_sample=self.encoder_channel.read(np.degrees(self.plant.data.qpos[self.plant.q]))
            self.footstep_tracker.observe(encoder_sample,self.attitude_filter,self.imu_reading,
                                         reset_contacts=not self.balance.moving)
            if self.balance.moving and permitted and self.footstep_tracker.healthy:
                self.command_target=self.target.copy()
        hold_config=self.profiles.get(self.profile,{}).get('pivot_hold')
        if hold_config:
            from simulation.mujoco.runtime.pivot_hold import PivotHold
            if not hasattr(self,'pivot_hold'):self.pivot_hold=PivotHold(self.plant.model)
            encoders=self.encoder_channel.read(np.degrees(self.plant.data.qpos[self.plant.q]))
            enabled=permitted and self.motion is not None and abs(self.request[0])<.05 and abs(self.request[1])>.01
            self.command_target=self.pivot_hold.apply(self.command_target,encoders,self.attitude_filter,self.imu_reading,
                self.phase,self.active_profile_params()[1],hold_config,enabled)
        if not self.stow_path:
            self.tracking.sample(float(self.plant.data.time),self.command_target,
                np.degrees(self.plant.data.qpos[self.plant.q]),drop=self.plant.p.get('tracking_feedback_drop',False))
        self.plant.step(targets_deg=self.command_target,balance=False,torque_enabled=self.torque,
                        native_servo=self.profile=='s_native_v6_1')
        self.imu_reading = self.imu.read(float(self.plant.data.time))
        # Ignore configured sensor-entry warmup; subsequent missing reads fail
        # after three control polls, as in firmware. Ground truth is display only.
        ready_at = self.imu.config.startup_s+self.imu.config.fusion_delay_s+81/self.imu.config.i2c_hz
        previous_safety = self.safety
        if self.plant.data.time >= ready_at:
            fault = self.attitude_filter.update(self.imu_reading)
            if self.stow_path and self.transition and not stow_attitude_ok(self.imu_reading):
                self.stop('Stow IMU/tilt limit');self.torque=False;self.safety='imu' if self.imu_reading is None else 'tilt'
            # Explicit simulator fall experiment: keep native walking through tilt.
            # Sensor faults, Stow limits, user Stop and the watchdog still apply.
            if (fault == 'tilt' and self.motion and not self.stow_path
                    and self.plant.p.get('sim_allow_fall', False)
                    and self.profiles.get(self.profile, {}).get('s_native')):
                fault = None
            if (self.motion or (self.balance.enabled and self.pose=='stand' and self.torque)) and fault:
                self.safety=fault
                if self.motion: self.finish_stop(fault)
        rotation=self.plant.data.xmat[self.plant.model.body('robot').id].reshape(3,3)
        self.flight_frames.append(dict(time_s=float(self.plant.data.time),profile=self.profile,
            request=self.request,linear=self.linear,yaw=self.yaw,phase=self.phase,
            roll_deg=math.degrees(math.atan2(rotation[2,1],rotation[2,2])),
            pitch_deg=math.degrees(math.asin(float(np.clip(-rotation[2,0],-1,1)))),
            imu=self.imu_reading,balance=self.balance.diagnostic(),position_wbc=self.position_wbc.diagnostic,
            stabilization=self.body_stabilizer.diagnostic.copy() if self.profile=='attitudepd' else None,
            support_shift=self.support_shift.diagnostic.copy() if self.support_shift else None,
            footstep_tracking=self.footstep_tracker.diagnostic.copy() if self.footstep_tracker else None,
            nominal=self.target.tolist(),command=self.command_target.tolist(),
            voltage=self.plant.voltage,safety=self.safety))
        if previous_safety=='ok' and self.safety!='ok' and self.incident_directory:
            try:
                self.incident_directory.mkdir(parents=True,exist_ok=True)
                path=self.incident_directory/f'incident-{time.time_ns()}.json'
                path.write_text(json.dumps(dict(reason=self.safety,frames=list(self.flight_frames),
                                               commands=list(self.flight_commands))))
                self.last_incident=str(path)
                print('Simulator incident:',path,flush=True)
            except OSError as exc:
                print('Simulator incident write failed:',exc,flush=True)



class ConsoleServer:
    def __init__(self, controller, host, port):
        self.controller=controller; self.selector=selectors.DefaultSelector()
        self.listener=socket.socket(); self.listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.listener.bind((host,port)); self.listener.listen(2); self.listener.setblocking(False)
        self.selector.register(self.listener,selectors.EVENT_READ)
        self.client=None; self.buffer=bytearray(); self.output=bytearray()

    def drop(self):
        if self.client:
            self.selector.unregister(self.client); self.client.close(); self.client=None
            self.controller.disconnected()
        self.buffer.clear(); self.output.clear()

    def poll(self, now):
        for key,_ in self.selector.select(0):
            if key.fileobj is self.listener:
                client,_=self.listener.accept(); client.setblocking(False)
                if self.client or self.controller.motion or self.controller.transition:
                    client.close()
                else:
                    self.controller.drain()
                    self.client=client; self.selector.register(client,selectors.EVENT_READ)
            else:
                try: data=self.client.recv(4096)
                except (ConnectionError,OSError): data=b''
                if not data: self.drop(); continue
                for byte in data:
                    if byte==3:
                        self.buffer.clear(); self.controller.command('\x03',now)
                    elif byte==10:
                        self.controller.command(self.buffer.decode('ascii',errors='replace').strip(),now); self.buffer.clear()
                    else: self.buffer.append(byte)
                    if len(self.buffer)>1024: self.drop(); break
        data=self.controller.drain()
        if self.client:
            self.output.extend(data)
            if len(self.output)>65536: self.drop(); return
            try:
                if self.output:
                    sent=self.client.send(self.output); del self.output[:sent]
            except BlockingIOError: pass
            except OSError: self.drop()

    def close(self):
        self.drop(); self.selector.close(); self.listener.close()


def parse_args(argv=None):
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--host',default='0.0.0.0',help='LAN control for iPhone; use 127.0.0.1 for Mac-only control')
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--stow',action='store_true',help='Compatibility option; shared physical Stow is enabled by default')
    parser.add_argument('--no-video',action='store_true',help='Disable LAN phone video')
    parser.add_argument('--video-host',default='0.0.0.0')
    parser.add_argument('--video-port',type=int,default=8766)
    parser.add_argument('--allow-fall',action='store_true',help='Simulator experiment: native gait continues through body tilt; Stop and sensor faults remain active')
    parser.add_argument('--profile', help='Initial gait profile (including optional experiments)')
    parser.add_argument('--experimental-profiles',type=Path,help='Opt-in simulator-only profile manifest')
    parser.add_argument('--parameters',type=Path,default=CAD/'physics_parameters_measured_total_2754g.json')
    parser.add_argument('--desktop-heartbeat',type=Path,help='Desktop-owned process: exit when the heartbeat file is removed or older than 5s')
    parser.add_argument('--foot-cushion',type=Path,default=(SIM_ROOT / 'config/foot_cushion_d37p3_l27mm.json'),help='Attached cushion parameters; defaults to the measured robot cushion dimensions')
    parser.add_argument('--heading',choices=('on','off'),default='on',help='Initial IMU heading hold state')
    parser.add_argument('--balance',choices=('on','off'),default='on',help='Initial IMU body leveling state')
    display=parser.add_mutually_exclusive_group()
    display.add_argument('--viewer',dest='viewer',action='store_true')
    display.add_argument('--headless',dest='viewer',action='store_false')
    parser.set_defaults(viewer=True)
    ble=parser.add_mutually_exclusive_group()
    ble.add_argument('--ble',dest='ble',action='store_true')
    ble.add_argument('--no-ble',dest='ble',action='store_false')
    parser.set_defaults(ble=sys.platform == 'darwin')
    return parser.parse_args(argv)


def load_parameters(args):
    parameters=json.loads(args.parameters.read_text())
    parameters['timestep_s']=.0005
    parameters['experimental_stow']=True
    parameters['aligned_standing_pose']=True
    parameters['sim_allow_fall']=args.allow_fall
    parameters['foot_cushion']=json.loads(args.foot_cushion.read_text())
    return parameters


def model_description(plant, parameters):
    cushion=parameters['foot_cushion']
    return (f"{plant.model.body_mass.sum():.3f} kg / cushion "
            f"D{cushion.get('sole_diameter_m', 2*cushion['radius_m'])*1000:.1f} x "
            f"{cushion.get('total_length_m',cushion['thickness_m'])*1000:.0f} mm / estimated contact")


def main():
    args=parse_args()
    bridge_binary=None
    if args.ble:
        from simulation.mujoco.ble_bridge.build import build as build_bridge
        bridge_binary=build_bridge()
    parameters=load_parameters(args)
    plant=Simulation(parameters); controller=RobotController(plant)
    model_info=model_description(plant,parameters)
    print('Model: '+model_info,flush=True)
    if args.experimental_profiles: controller.load_experimental_profiles(args.experimental_profiles)
    if args.profile: controller.select_profile(args.profile)
    controller.heading.enabled=args.heading=='on'
    controller.balance.enabled=args.balance=='on'
    controller.incident_directory=Path(tempfile.gettempdir())/'spot-omg-sim'
    server=ConsoleServer(controller,args.host,args.port)
    bridge_process=None
    viewer=None
    video=None
    keys=collections.deque()
    profile_keys={49:'legacy',50:'crawl',51:'cruise',52:'trot',53:'highstep',54:'lift',55:'imu',56:'level',57:'level15',48:'joint',70:'jointfast',71:'jointsport'}
    try:
        if not args.no_video:
            from simulation.mujoco.runtime.video_stream import VideoStream
            video=VideoStream(parameters,args.video_host,args.video_port)
        if bridge_binary:
            bridge_process=subprocess.Popen([str(bridge_binary),"--port",str(args.port)])
        if args.viewer:
            import mujoco.viewer
            viewer=mujoco.viewer.launch_passive(plant.model,plant.data,key_callback=keys.append)
            # Default side view: robot front points right, with the feet in frame.
            viewer.cam.distance=1.05
            viewer.cam.azimuth=90
            viewer.cam.elevation=0
            viewer.cam.lookat[:]=plant.data.subtree_com[plant.model.body('robot').id]-[0,0,.1]
        print(f'Virtual robot: {server.listener.getsockname()} / physics=estimated / gait profiles + shared C balance',flush=True)
        pacer=PhysicsPacer(time.monotonic())
        rate_wall=time.monotonic();rate_sim=float(plant.data.time);realtime_factor=1.0;display_speed=0.0
        desktop_check_at = 0.
        while viewer is None or viewer.is_running():
            if args.desktop_heartbeat and time.monotonic() >= desktop_check_at:
                desktop_check_at = time.monotonic() + .25
                try:
                    if time.time() - args.desktop_heartbeat.stat().st_mtime > 5:
                        break
                except FileNotFoundError:
                    break
            while keys:
                key=keys.popleft()
                if key in profile_keys:
                    if controller.motion or controller.transition:
                        controller.pending_profile=profile_keys[key]
                        controller.stop('profile-change')
                    else: controller.select_profile(profile_keys[key])
                elif key==32: controller.stop('keyboard-stop')
                elif key==87 and not server.client and not controller.motion and not controller.transition:
                    controller.command('simwalk 8',time.monotonic())
            now=time.monotonic(); server.poll(now)
            steps=pacer.due(now)
            for _ in range(steps):
                controller.tick(time.monotonic())
                if video:video.publish(plant,viewer.cam if viewer else None)
            if steps:
                elapsed_wall=time.monotonic()-rate_wall
                if elapsed_wall>=1:
                    realtime_factor=(float(plant.data.time)-rate_sim)/elapsed_wall
                    rate_wall=time.monotonic();rate_sim=float(plant.data.time)
                    controller.realtime_factor=realtime_factor
                if viewer:
                    row=plant.row()
                    camera_alpha=1-math.exp(-steps*.02/.15)
                    camera_target=np.asarray(row['com_m'])-[0,0,.1]
                    viewer.cam.lookat[:]+=camera_alpha*(camera_target-viewer.cam.lookat)
                    rotation=plant.data.xmat[plant.model.body('robot').id].reshape(3,3)
                    speed=float(rotation[:,0]@plant.data.qvel[:3])
                    display_speed+=camera_alpha*(speed-display_speed)
                    viewer.opt.geomgroup[3]=0
                    viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150,mujoco.mjtGridPos.mjGRID_TOPLEFT,
                        'Profile / simulated physics\nModel / cushion\nMovement\nSpeed / tilt\nBNO055 / balance\nProfiles\nDemo / stop\nRemote',
                        f'{controller.profile.upper()}{" / FALL TEST" if parameters.get("sim_allow_fall") else ""} / 11.1V 3S / {parameters.get("scenario","nominal")}\n{model_info}\n{controller.motion[0] if controller.motion else "stand"}\n{display_speed:.2f} m/s / {max(abs(row["roll_deg"]),abs(row["pitch_deg"])):.1f} deg\n{controller.imu_reading["age_ms"] if controller.imu_reading else -1:.0f} ms / balance {"ON" if controller.balance.enabled else "OFF"} / heading {"ON" if controller.heading.enabled else "OFF"} / {max(abs(controller.balance.correction)):.1f} deg correction\n1 Legacy  2 Crawl  3 Cruise  4 Trot  5 High step  6 Lift  7 IMU  8 Level  9 Level15 0 Joint F Fast G Sport\nW: 8s walk (no remote owner) / Space: stop\nBLE app / TCP :{args.port} / real time {realtime_factor:.2f}x'))
                    viewer.sync()
            time.sleep(.001)
    except KeyboardInterrupt: pass
    finally:
        server.close()
        if video:video.close()
        if bridge_process:
            bridge_process.terminate()
            try: bridge_process.wait(timeout=3)
            except subprocess.TimeoutExpired: bridge_process.kill()
        if viewer: viewer.close()


if __name__=='__main__': main()
