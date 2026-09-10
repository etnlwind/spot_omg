"""Single-owner virtual robot console. Shared firmware C trajectories, estimated plant.

Run with mjpython for --viewer. No hardware transport is imported or opened.
The protocol adapter is intentionally separate from the physics plant.
"""
from stow_policy import LANDING, FOLDED, FOLD_SECONDS, RELEASE_TORQUE_AT_STOW
import argparse
import collections
import json
import math
import selectors
import socket
import subprocess
import time
from pathlib import Path

import numpy as np
from cad_gait import CAD, KEYS
from cad_physics import Simulation
from realtime_pacer import PhysicsPacer
from balance_controller import BalanceController
from heading_controller import HeadingController
from drive_controller import step as shared_drive_step
import copy
from bno055_emulator import BNO055Config, BNO055Emulator, FirmwareAttitudeFilter
from gait_profiles import foot_targets, load_profiles, PROFILE_FILE


class RobotController:
    """Console/session adapter; C gait equations are shared, STM32 HAL is not emulated."""
    def __init__(self, plant):
        self.plant = plant
        self.target = np.degrees(plant.desired).copy()
        self.pose = 'stand'
        self.torque = True
        self.safety = 'ok'
        self.motion = None
        self.transition = None
        self.stow_path = False
        self.stow_queue = []
        self.phase = 0.
        self.linear = self.yaw = 0.
        self.request = (0., 0.)
        self.sequence = 0
        self.last_packet = 0.
        self.elapsed = 0.
        self.out = []
        self.profiles = load_profiles() if PROFILE_FILE.exists() else {}
        self.deployed_profiles = copy.deepcopy(self.profiles)
        self.profile = json.loads(PROFILE_FILE.read_text()).get('default','legacy') if self.profiles else 'legacy'
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

    def observe_imu(self, model, data):
        r = data.xmat[model.body('robot').id].reshape(3,3)
        self.imu.advance(float(data.time),
            math.degrees(math.atan2(r[2,1],r[2,2])),
            math.degrees(math.asin(float(np.clip(-r[2,0],-1,1)))),
            math.degrees(math.atan2(r[1,0],r[0,0])))

    def imu_diagnostic(self):
        return dict(sensor='BNO055', mode='IMUPLUS', emulated=True,
                    feedback='active' if self.balance.applied else 'suspended', balance=self.balance.diagnostic(), heading=self.heading.diagnostic(), reading=self.imu_reading,
                    filtered_tenths=self.attitude_filter.filtered,
                    filtered_rate_tenths_s=self.attitude_filter.rate,
                    failures=self.attitude_filter.failures,
                    config=vars(self.imu.config),last_incident=self.last_incident,realtime_factor=self.realtime_factor)

    def reply(self, text='OK'):
        self.out.append(text + '\r\n# ')

    def drain(self):
        result = ''.join(self.out).encode()
        self.out.clear()
        return result

    def blend(self, target, completion, duration=1.):
        self.transition = (self.target.copy(), np.array(target), 0., duration, completion)
        self.pose = 'custom'

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
        if self.motion[0] in ('drive','profile') and not self.transition and self.elapsed > .1:
            self.stopping_reason = reason
            self.request = (0.,0.)
            return
        self.finish_stop(reason)

    def finish_stop(self, reason):
        drive = self.motion[0] == 'drive'
        self.motion = None
        self.stopping_reason = None
        self.request = (0., 0.)
        completion = f'$SPOTDRIVE stopped reason={reason}\r\nOK' if drive else ('OK' if reason=='complete' else 'STOPPED: '+reason)
        self.blend([0, 45, 90]*4, completion)

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
            elif cmd == 'syncstate':
                error = round(float(np.max(np.abs(self.command_target-np.degrees(self.plant.data.qpos[self.plant.q]))))*4096/360)
                self.reply(f'$SPOTSTATE pose={self.pose} error={error} torque={"on" if self.torque else "off"} safety={self.safety} balance={"active" if self.balance.applied else "suspended" if self.balance.enabled else "off"} heading={"on" if self.heading.enabled else "off"} rev=shared-locomotion-v24-sim caps=trot5,simprofiles,gaitprofiles,bno055emu,simbalance,balancecontrol,headinghold{",simstow" if self.plant.p.get("experimental_stow") else ""} imu=bno055-emulated backend=sim physics=estimated profile={self.profile} reverse_limit={600 if self.profile in ("trot","highstep","lift","imu","level","level15","joint","jointfast","jointsport") else 1000}')
            elif cmd == 'read' and words == ['read', '1']:
                self.reply(f'ID 1 voltage={round(self.plant.voltage*1000)}mV source=simulated')
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
                    self.request=(self.limited_linear(linear/1000), self.plant.policy.drive_yaw_limit(yaw)/1000)
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
            elif cmd=='stow' and len(words)==1:
                if not self.plant.p.get('experimental_stow'):
                    raise ValueError('Stow requires --stow experimental simulator')
                if self.safety!='ok':raise ValueError('recover required')
                already_stow=self.stow_path
                if already_stow and not self.torque:self.capture_current_target()
                self.stow_path=True;self.plant.stow_active=True;self.torque=True
                self.balance.correction[:]=0;self.balance.integral[:]=0
                if self.pose=='landing' or already_stow:
                    self.blend(FOLDED,'OK stow',FOLD_SECONDS)
                else:
                    self.stow_queue=[(FOLDED,'OK stow',FOLD_SECONDS)]
                    self.blend(LANDING,'STOW landing',2.)
            elif cmd=='landing' and self.stow_path and len(words)==1:
                self.capture_current_target()
                self.torque=True
                self.blend(LANDING,'OK landing',FOLD_SECONDS)
            elif cmd == 'heading' and len(words)==2 and words[1] in ('on','off'):
                self.heading.enabled=words[1]=='on'; self.heading.update(None,0,0,0); self.reply()
            elif cmd in ('balance','simbalance') and len(words)==2 and words[1] in ('on','off'):
                self.balance.enabled=words[1]=='on'; self.reply()
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
                target = {'stand':[0,45,90]*4,'stand11':[0,0,0]*4,'landing':[0,40,130]*4,
                          'hold':np.degrees(self.plant.data.qpos[self.plant.q])}[cmd]
                self.blend(target, 'OK '+cmd)
            elif cmd == 'drive':
                linear,yaw,seq=map(int,words[1:])
                if max(abs(linear),abs(yaw))>1000 or not 0<=seq<=0xffffffff:
                    raise ValueError('invalid drive input')
                self.begin(('drive',), now)
                self.sequence=seq; self.last_packet=now
                self.request=(self.limited_linear(linear/1000),self.plant.policy.drive_yaw_limit(yaw)/1000)
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
        if 'turn_reverse_params' not in profile:return forward
        # Blend continuously through low forward input; phase is never reset.
        weight=self.plant.policy.smootherstep(float(np.clip(self.linear/.5,0,1)))
        base=np.asarray(profile['turn_reverse_params'],dtype=float)
        return base+(forward-base)*weight

    def gait(self, phase, amplitude):
        policy=self.plant.policy; kind=self.motion[0]
        if kind in ('drive','profile') and self.profile != 'legacy':
            profile=self.profiles[self.profile]
            params=self.active_profile_params()
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
        self.motion=motion; self.elapsed=self.phase=self.linear=self.yaw=0.
        self.stopping_reason=None
        neutral=self.gait(0,0)
        self.blend([neutral[k] for k in KEYS], None)

    def tick(self, now):
        if self.motion and self.motion[0]=='drive' and now-self.last_packet>.8:
            self.stop('watchdog')
        if self.motion is None or self.transition:
            self.heading.update(None,0,0,0)
        if self.transition:
            start,end,elapsed,duration,completion=self.transition
            elapsed=min(duration,elapsed+.02)
            t=self.plant.policy.smootherstep(elapsed/duration)
            self.target=start+(end-start)*t
            self.transition=(start,end,elapsed,duration,completion)
            if elapsed>=duration:
                self.transition=None
                if self.stow_queue:
                    self.blend(*self.stow_queue.pop(0))
                elif completion:
                    if self.stow_path and completion in ('OK stow','OK landing'):
                        error=float(np.max(abs(np.degrees(self.plant.data.qpos[self.plant.q])-end)))
                        if error>12:
                            self.capture_current_target()
                            if completion=='OK stow':self.torque=False
                            completion=f'ERROR: Stow posture target not reached ({error:.1f} deg)'
                    if self.stow_path and completion=='OK landing':
                        self.stow_path=False;self.plant.stow_active=False
                    if completion=='OK stow' and RELEASE_TORQUE_AT_STOW:
                        # Do not keep pushing toward a position after the legs
                        # settle into contact. Gravity/contact now support them.
                        self.torque=False
                    self.pose=completion[3:] if completion in ('OK stand','OK stand11','OK landing','OK stow') else ('stand' if np.allclose(end,[0,45,90]*4) else 'custom')
                    if self.stow_path and completion.startswith('ERROR:'):
                        self.pose='stow-paused'
                    self.reply(completion)
        elif self.motion:
            if self.motion[0]=='drive' and self.profiles==self.deployed_profiles:
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
            if self.stopping_reason and max(abs(self.linear),abs(self.yaw))<=.008:
                self.finish_stop(self.stopping_reason)
            elif self.motion[0]!='drive' and self.elapsed>=self.motion[1]:
                if self.motion[0]=='profile': self.stop('complete')
                else: self.motion=None; self.blend([0,45,90]*4,'OK')
        if self.pending_profile and not self.motion and not self.transition:
            self.select_profile(self.pending_profile); self.pending_profile=None
        returning_stand = self.transition is not None and np.allclose(self.transition[1],[0,45,90]*4)
        permitted = self.torque and self.safety=='ok' and (self.pose=='stand' or self.motion is not None or returning_stand)
        if self.stow_path or (self.transition and self.transition[4] in ('OK landing','OK stand11','OK hold')):
            permitted=False
        # Delayed angle derivatives amplified full-speed gait oscillations.
        # Use bounded slow PI during all motion/pose transitions, D only at rest.
        moving = self.motion is not None or self.transition is not None
        self.balance.kp,self.balance.kd,self.balance.ki = (.1,0.,.03) if moving else (.25,.015,.15)
        self.balance.standing = self.motion is None and self.transition is None and self.pose=='stand'
        self.balance.profile=self.profile
        self.balance.phase=self.phase
        self.balance.moving=self.motion is not None and self.transition is None
        self.balance.linear,self.balance.yaw=self.linear,self.yaw
        if self.stow_path:
            # Experimental unwrapped posture: the gait balance stage clamps J2
            # even when disabled. Preserve this posture's separately checked path.
            self.balance.integral[:]=0;self.balance.correction[:]=0
            self.balance.applied=False;self.balance.saturated=False
            self.command_target=self.target.copy()
        else:
            self.command_target = self.balance.apply(self.target,self.attitude_filter,
                self.imu_reading is not None and self.attitude_filter.failures==0,permitted)
        self.plant.step(targets_deg=self.command_target,balance=False,torque_enabled=self.torque)
        self.imu_reading = self.imu.read(float(self.plant.data.time))
        # Ignore configured sensor-entry warmup; subsequent missing reads fail
        # after three control polls, as in firmware. Ground truth is display only.
        ready_at = self.imu.config.startup_s+self.imu.config.fusion_delay_s+81/self.imu.config.i2c_hz
        previous_safety = self.safety
        if self.plant.data.time >= ready_at:
            fault = self.attitude_filter.update(self.imu_reading)
            if (self.motion or (self.balance.enabled and self.pose=='stand' and self.torque)) and fault:
                self.safety=fault
                if self.motion: self.finish_stop(fault)
        rotation=self.plant.data.xmat[self.plant.model.body('robot').id].reshape(3,3)
        self.flight_frames.append(dict(time_s=float(self.plant.data.time),profile=self.profile,
            request=self.request,linear=self.linear,yaw=self.yaw,phase=self.phase,
            roll_deg=math.degrees(math.atan2(rotation[2,1],rotation[2,2])),
            pitch_deg=math.degrees(math.asin(float(np.clip(-rotation[2,0],-1,1)))),
            imu=self.imu_reading,balance=self.balance.diagnostic(),
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


def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--host',default='0.0.0.0',help='LAN control for iPhone; use 127.0.0.1 for Mac-only control')
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--stow',action='store_true',help='Enable experimental full-turn Stow (simulation only)')
    parser.add_argument('--no-video',action='store_true',help='Disable LAN phone video')
    parser.add_argument('--video-host',default='0.0.0.0')
    parser.add_argument('--video-port',type=int,default=8766)
    parser.add_argument('--profile', choices=('legacy',*load_profiles()), help='Initial gait profile')
    parser.add_argument('--parameters',type=Path,default=CAD/'physics_parameters.json')
    parser.add_argument('--heading',choices=('on','off'),default='on',help='Initial IMU heading hold state')
    parser.add_argument('--balance',choices=('on','off'),default='on',help='Initial IMU body leveling state')
    display=parser.add_mutually_exclusive_group()
    display.add_argument('--viewer',dest='viewer',action='store_true')
    display.add_argument('--headless',dest='viewer',action='store_false')
    parser.set_defaults(viewer=True)
    ble=parser.add_mutually_exclusive_group()
    ble.add_argument('--ble',dest='ble',action='store_true')
    ble.add_argument('--no-ble',dest='ble',action='store_false')
    parser.set_defaults(ble=True)
    args=parser.parse_args()
    bridge_binary=None
    if args.ble:
        from ble_bridge.build import build as build_bridge
        bridge_binary=build_bridge()
    parameters=json.loads(args.parameters.read_text()); parameters['timestep_s']=.0005
    if args.stow:parameters['experimental_stow']=True
    plant=Simulation(parameters); controller=RobotController(plant)
    if args.profile: controller.select_profile(args.profile)
    controller.heading.enabled=args.heading=='on'
    controller.balance.enabled=args.balance=='on'
    controller.incident_directory=Path('/private/tmp/spot-omg-sim')
    server=ConsoleServer(controller,args.host,args.port)
    bridge_process=None
    viewer=None
    video=None
    keys=collections.deque()
    profile_keys={49:'legacy',50:'crawl',51:'cruise',52:'trot',53:'highstep',54:'lift',55:'imu',56:'level',57:'level15',48:'joint',70:'jointfast',71:'jointsport'}
    try:
        if not args.no_video:
            from video_stream import VideoStream
            video=VideoStream(parameters,args.video_host,args.video_port)
        if bridge_binary:
            bridge_process=subprocess.Popen([str(bridge_binary),"--port",str(args.port)])
        if args.viewer:
            import mujoco.viewer
            viewer=mujoco.viewer.launch_passive(plant.model,plant.data,key_callback=keys.append)
            viewer.cam.distance=1.5
        print(f'Virtual robot: {server.listener.getsockname()} / physics=estimated / gait profiles + shared C balance',flush=True)
        pacer=PhysicsPacer(time.monotonic())
        rate_wall=time.monotonic();rate_sim=float(plant.data.time);realtime_factor=1.0;display_speed=0.0
        while viewer is None or viewer.is_running():
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
                    viewer.cam.lookat[:]+=camera_alpha*(np.asarray(row['com_m'])-viewer.cam.lookat)
                    rotation=plant.data.xmat[plant.model.body('robot').id].reshape(3,3)
                    speed=float(rotation[:,0]@plant.data.qvel[:3])
                    display_speed+=camera_alpha*(speed-display_speed)
                    viewer.opt.geomgroup[3]=0
                    viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150,mujoco.mjtGridPos.mjGRID_TOPLEFT,
                        'Profile / simulated physics\nMovement\nSpeed / tilt\nBNO055 / balance\nProfiles\nDemo / stop\nRemote',
                        f'{controller.profile.upper()} / 11.1V 3S / {parameters.get("scenario","nominal")}\n{controller.motion[0] if controller.motion else "stand"}\n{display_speed:.2f} m/s / {max(abs(row["roll_deg"]),abs(row["pitch_deg"])):.1f} deg\n{controller.imu_reading["age_ms"] if controller.imu_reading else -1:.0f} ms / balance {"ON" if controller.balance.enabled else "OFF"} / heading {"ON" if controller.heading.enabled else "OFF"} / {max(abs(controller.balance.correction)):.1f} deg correction\n1 Legacy  2 Crawl  3 Cruise  4 Trot  5 High step  6 Lift  7 IMU  8 Level  9 Level15 0 Joint F Fast G Sport\nW: 8s walk (no remote owner) / Space: stop\nBLE app / TCP :{args.port} / real time {realtime_factor:.2f}x'))
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
