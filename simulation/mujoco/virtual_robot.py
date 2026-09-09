"""Single-owner virtual robot console. Shared firmware C trajectories, estimated plant.

Run with mjpython for --viewer. No hardware transport is imported or opened.
The protocol adapter is intentionally separate from the physics plant.
"""
import argparse
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
        self.phase = 0.
        self.linear = self.yaw = 0.
        self.request = (0., 0.)
        self.sequence = 0
        self.last_packet = 0.
        self.elapsed = 0.
        self.out = []

    def reply(self, text='OK'):
        self.out.append(text + '\r\n# ')

    def drain(self):
        result = ''.join(self.out).encode()
        self.out.clear()
        return result

    def blend(self, target, completion, duration=1.):
        self.transition = (self.target.copy(), np.array(target), 0., duration, completion)
        self.pose = 'custom'

    def stop(self, reason='requested'):
        if self.motion is None:
            return
        drive = self.motion[0] == 'drive'
        self.motion = None
        self.request = (0., 0.)
        self.blend([0, 45, 90]*4,
                   f'$SPOTDRIVE stopped reason={reason}\r\nOK' if drive else 'STOPPED: '+reason)

    def disconnected(self):
        self.stop('disconnect')
        # Completion output belongs to the old session, never the next owner.
        self.out.clear()

    def command(self, line, now):
        words = line.split()
        if not words:
            return
        cmd = words[0]
        try:
            if cmd == 'identity':
                self.reply('$SPOTBACKEND backend=sim protocol=1 physics=estimated controller=shared-gait adapter=python')
            elif cmd in ('echo', 'log'):
                if line not in ('echo off', 'echo on') and not (len(words)==3 and words[:2]==['log','time'] and words[2].isdigit()):
                    raise ValueError('unsupported simulator command')
                self.reply()
            elif cmd == 'syncstate':
                error = round(float(np.max(np.abs(self.target-np.degrees(self.plant.data.qpos[self.plant.q]))))*4096/360)
                self.reply(f'$SPOTSTATE pose={self.pose} error={error} torque={"on" if self.torque else "off"} safety={self.safety} balance=monitor rev=walk-stance-v16-sim caps=trot5 backend=sim physics=estimated')
            elif cmd == 'read' and words == ['read', '1']:
                self.reply(f'ID 1 voltage={round(self.plant.voltage*1000)}mV source=simulated')
            elif cmd in ('gaitdiag', 'baldiag', 'targets'):
                self.reply('$SIMSTATE '+json.dumps(self.plant.row(), separators=(',', ':')))
            elif cmd == '@D':
                seq, linear, yaw = map(int, words[1:])
                if not 0 <= seq <= 0xffffffff or max(abs(linear),abs(yaw)) > 1000:
                    raise ValueError('invalid drive packet')
                delta = (seq-self.sequence) & 0xffffffff
                if self.motion and self.motion[0]=='drive' and 0 < delta < 0x80000000:
                    self.sequence=seq; self.last_packet=now
                    self.request=(linear/1000, self.plant.policy.drive_yaw_limit(yaw)/1000)
            elif cmd == '@S':
                seq = int(words[1])
                if len(words)!=2 or not 0 <= seq <= 0xffffffff:
                    raise ValueError('invalid stop packet')
                if 0 < ((seq-self.sequence)&0xffffffff) < 0x80000000:
                    self.sequence=seq; self.stop()
            elif cmd == '\x03':
                self.stop('interrupt')
            elif self.motion or self.transition:
                raise ValueError('busy; stop and wait for prompt')
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
                self.request=(linear/1000,self.plant.policy.drive_yaw_limit(yaw)/1000)
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

    def gait(self, phase, amplitude):
        policy=self.plant.policy; kind=self.motion[0]
        if kind=='drive':
            return policy.drive_walk_targets(phase,amplitude,self.linear,self.yaw)[0]
        if kind=='trot5': return policy.trot5_targets(phase,amplitude)[0]
        if kind in ('trot4','trot4back'):
            return policy.trot4_direction_targets(phase,amplitude,1 if kind=='trot4' else -1)[0]
        return getattr(policy,kind+'_targets')(phase,amplitude,self.motion[3])[0]

    def begin(self, motion, now):
        if self.safety!='ok' or not self.torque:
            raise ValueError('stand/recover required before gait')
        self.motion=motion; self.elapsed=self.phase=self.linear=self.yaw=0.
        neutral=self.gait(0,0)
        self.blend([neutral[k] for k in KEYS], None)

    def tick(self, now):
        if self.motion and self.motion[0]=='drive' and now-self.last_packet>.8:
            self.stop('watchdog')
        if self.transition:
            start,end,elapsed,duration,completion=self.transition
            elapsed=min(duration,elapsed+.02)
            t=self.plant.policy.smootherstep(elapsed/duration)
            self.target=start+(end-start)*t
            self.transition=(start,end,elapsed,duration,completion)
            if elapsed>=duration:
                self.transition=None
                if completion:
                    self.pose=completion[3:] if completion in ('OK stand','OK stand11','OK landing') else 'custom'
                    self.reply(completion)
        elif self.motion:
            self.elapsed+=.02
            self.linear=self.plant.policy.drive_slew(round(self.linear*1000),round(self.request[0]*1000))/1000
            self.yaw=self.plant.policy.drive_slew(round(self.yaw*1000),round(self.request[1]*1000))/1000
            result=self.gait(self.phase,self.plant.policy.smootherstep(min(1,self.elapsed)))
            self.target=np.array([result[k] for k in KEYS])
            period=self.plant.policy.drive_period_ms(round(self.linear*1000),round(self.yaw*1000))/1000 if self.motion[0]=='drive' else self.motion[2]
            self.phase=(self.phase+.02/period)%1
            if self.motion[0]!='drive' and self.elapsed>=self.motion[1]:
                self.motion=None; self.blend([0,45,90]*4,'OK')
        self.plant.step(targets_deg=self.target,balance=False,torque_enabled=self.torque)
        row=self.plant.row()
        if self.motion and max(abs(row['roll_deg']),abs(row['pitch_deg']))>12:
            self.safety='tilt'; self.stop('tilt')


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
    parser.add_argument('--host',default='127.0.0.1',help='use LAN IP or 0.0.0.0 for iPhone')
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--parameters',type=Path,default=CAD/'physics_parameters.json')
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
    plant=Simulation(parameters); controller=RobotController(plant)
    server=ConsoleServer(controller,args.host,args.port)
    bridge_process=None
    viewer=None
    try:
        if bridge_binary:
            bridge_process=subprocess.Popen([str(bridge_binary),"--port",str(args.port)])
        if args.viewer:
            import mujoco.viewer
            viewer=mujoco.viewer.launch_passive(plant.model,plant.data)
            viewer.cam.distance=1.5
        print(f'Virtual robot: {server.listener.getsockname()} / physics=estimated / V16 shared C gait',flush=True)
        deadline=time.monotonic()
        while viewer is None or viewer.is_running():
            now=time.monotonic(); server.poll(now)
            if now>=deadline:
                controller.tick(now); deadline=max(deadline+.02,time.monotonic())
                if viewer: viewer.sync()
            time.sleep(.001)
    except KeyboardInterrupt: pass
    finally:
        server.close()
        if bridge_process:
            bridge_process.terminate()
            try: bridge_process.wait(timeout=3)
            except subprocess.TimeoutExpired: bridge_process.kill()
        if viewer: viewer.close()


if __name__=='__main__': main()
