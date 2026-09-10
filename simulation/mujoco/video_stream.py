"""Read-only LAN video; offscreen rendering never runs in the control loop."""
import io
import json
import multiprocessing as mp
import queue
import socket
import subprocess
import threading
import time
from urllib.parse import urlsplit, parse_qs
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class VideoFrames:
    def __init__(self):
        self.lock=threading.Condition()
        self.jpeg=None
        self.sequence=0
        self.sim_time=0.
        self.updated=0.
        self.requested=0.

    def publish(self,jpeg,sim_time):
        with self.lock:
            self.jpeg=jpeg;self.sim_time=sim_time
            self.sequence+=1;self.updated=time.monotonic()
            self.lock.notify_all()

    def snapshot(self,after=None):
        with self.lock:
            self.requested=time.monotonic()
            if after is not None:
                self.lock.wait_for(lambda: self.sequence != after, timeout=.5)
            return self.jpeg,self.sequence,self.sim_time,self.updated


def handler_for(frames):
    class Handler(BaseHTTPRequestHandler):
        protocol_version='HTTP/1.1'

        def setup(self):
            super().setup()
            self.connection.settimeout(5)
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        def do_GET(self):
            if self.path.split('?')[0] not in ('/frame.jpg','/status'):
                self.send_error(404);return
            try:after=int(parse_qs(urlsplit(self.path).query).get('after',['-1'])[0])
            except ValueError:
                self.send_error(400);return
            jpeg,sequence,sim_time,updated=frames.snapshot(after)
            age=time.monotonic()-updated
            if self.path.split('?')[0]=='/status':
                data=json.dumps(dict(service='spotomg-video',protocol=1,sequence=sequence,
                                     simulation_time_s=sim_time,ready=jpeg is not None and age<2)).encode()
                code,content=200,'application/json'
            elif sequence==after:
                data=b'';code,content=204,'image/jpeg'
            elif jpeg is None or age>2:
                data=b'Waiting for a fresh simulator frame';code,content=503,'text/plain'
            else:data=jpeg;code,content=200,'image/jpeg'
            self.send_response(code)
            self.send_header('Content-Type',content)
            self.send_header('Content-Length',str(len(data)))
            self.send_header('Cache-Control','no-store')
            self.send_header('X-SpotOMG-Video','1')
            self.send_header('X-Frame-Sequence',str(sequence))
            self.send_header('X-Simulation-Time',str(sim_time))
            self.end_headers()
            try:self.wfile.write(data)
            except (BrokenPipeError,ConnectionResetError):pass
        def log_message(self,*args):pass
    return Handler


def serve(parameters,states,stop,host,port):
    import mujoco
    from PIL import Image
    from cad_physics import build
    frames=VideoFrames()
    server=None;advertisement=None;renderer=None
    try:
        server=ThreadingHTTPServer((host,port),handler_for(frames));server.daemon_threads=True
        threading.Thread(target=server.serve_forever,daemon=True).start()
        advertisement=subprocess.Popen(['/usr/bin/dns-sd','-R','SpotOMG MuJoCo','_spotomg-video._tcp','local',str(port),'protocol=1'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        xml,_=build(parameters,write_scene=False)
        model=mujoco.MjModel.from_xml_string(xml);data=mujoco.MjData(model)
        model.vis.quality.offsamples=0
        model.vis.quality.shadowsize=512
        model.vis.global_.offwidth=480;model.vis.global_.offheight=270
        renderer=mujoco.Renderer(model,height=270,width=480)
        camera=mujoco.MjvCamera();mujoco.mjv_defaultCamera(camera)
        option=mujoco.MjvOption();option.geomgroup[3]=0
        print(f'MuJoCo video: http://{host}:{port}/frame.jpg / Bonjour / 480x270 at up to 50 fps',flush=True)
        while not stop.is_set():
            try:state=states.get(timeout=.2)
            except queue.Empty:continue
            if time.monotonic()-frames.requested>3:continue
            qpos,sim_time,view=state
            data.qpos[:]=qpos;mujoco.mj_forward(model,data)
            camera.lookat[:]=view[0];camera.distance=view[1] * .75
            camera.azimuth=view[2];camera.elevation=view[3]
            renderer.update_scene(data,camera=camera,scene_option=option)
            output=io.BytesIO();Image.fromarray(renderer.render()).save(output,format='JPEG',quality=65)
            frames.publish(output.getvalue(),sim_time)
    except Exception as error:
        print(f'MuJoCo video unavailable: {error}',flush=True)
    finally:
        if renderer:renderer.close()
        if server:server.shutdown();server.server_close()
        if advertisement:
            advertisement.terminate()
            try:advertisement.wait(timeout=1)
            except subprocess.TimeoutExpired:advertisement.kill()


class VideoStream:
    def __init__(self,parameters,host='0.0.0.0',port=8766):
        ctx=mp.get_context('spawn')
        self.states=ctx.Queue(maxsize=1);self.stop_event=ctx.Event();self.last_sim_time=None
        self.process=ctx.Process(target=serve,args=(parameters,self.states,self.stop_event,host,port),daemon=True)
        self.process.start()

    def publish(self,plant,camera=None):
        sim_time=float(plant.data.time)
        if sim_time==self.last_sim_time or not self.process.is_alive():return
        self.last_sim_time=sim_time
        view=(camera.lookat.copy(),camera.distance,camera.azimuth,camera.elevation) if camera else (plant.data.xipos[plant.model.body('cad_base').id].copy(),1.1,135.,-20.)
        try:self.states.put_nowait((plant.data.qpos.copy(),float(plant.data.time),view))
        except queue.Full:pass

    def close(self):
        self.stop_event.set();self.process.join(timeout=2)
        if self.process.is_alive():self.process.terminate();self.process.join(timeout=1)
        self.states.cancel_join_thread();self.states.close()
