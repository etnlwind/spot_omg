"""Bounded independent scheduled support-transfer experiments. No plant truth in control."""
import ctypes, json, subprocess, sys, hashlib
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from arc_preload import ArcPreload
from diagnose_turn_clearance import run
from validate_support_shift import Recorder, metrics
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/audits/arc-transfer-2026-09-12'
SETTINGS=None

def setup():
    OUT.mkdir(parents=True,exist_ok=True)
    src=OUT/'gravity.c';lib=OUT/'gravity.dylib'
    src.write_text('#include "arc_preload.h"\nvoid evaluate(const float *values,float *bias,float *feet,float *jz,float *com){GaitPolicyLegTarget p[4];for(int i=0;i<4;i++){p[i].j1_deg=values[3*i];p[i].j2_deg=values[3*i+1];p[i].j3_deg=values[3*i+2];}arc_gravity_model(p,(float(*)[3])bias,(float(*)[3])feet,(float(*)[3])jz,com);}\n')
    subprocess.run(['clang','-O2','-shared','-fPIC','-I'+str(ROOT/'firmware/stm32-learning/Inc'),str(src),'-o',str(lib)],check=True)
    (OUT/'source-hashes.json').write_text(json.dumps({n:hashlib.sha256((ROOT/'firmware/stm32-learning/Inc'/n).read_bytes()).hexdigest() for n in ('arc_turn.h','arc_geometry.h','arc_preload.h')},indent=2))

def smooth(x):
    x=np.clip(x,0,1);return x*x*x*(10+x*(-15+6*x))

class ContinuousPreload(ArcPreload):
    def __init__(self,policy):
        self.lib=ctypes.CDLL(str(OUT/'gravity.dylib'));self.fn=self.lib.evaluate
        fp=ctypes.POINTER(ctypes.c_float);self.fn.argtypes=(fp,fp,fp,fp,fp)
        if SETTINGS.get('contact_point'):
            self.fn=policy._library.spot_arc_contact_gravity;self.fn.argtypes=(fp,fp,fp,fp,fp)
        self.state=np.zeros(12)
    def reset(self):self.state[:]=0
    def correction(self,target,phase,duty,gain):
        width,lead,gain,shape=SETTINGS['transfer']
        b=(ctypes.c_float*12)();feet=(ctypes.c_float*12)();jz=(ctypes.c_float*12)();com=(ctypes.c_float*3)()
        if SETTINGS.get('contact_point'):self.fn((ctypes.c_float*12)(*target),b,com,feet,jz)
        else:self.fn((ctypes.c_float*12)(*target),b,feet,jz,com)
        feet=np.array(feet).reshape(4,3);com=np.array(com)
        # Pair A supports [0,.5), pair B [.5,1). C2 blend centered on boundary.
        t=((phase+lead/1.2)%1)*1.2
        if width==0:w=float(t<.6)
        else:
            u=(t+width/2)%1.2
            if u<width:w=smooth(u/width)
            elif u<.6:w=1.
            elif u<.6+width:w=1-smooth((u-.6)/width)
            else:w=0.
        shares=np.zeros(4)
        for pair,pair_weight in (((0,3),w),((1,2),1-w)):
            a,c=pair;delta=feet[c,:2]-feet[a,:2]
            f=float(np.clip(np.dot(com[:2]-feet[a,:2],delta)/max(1e-9,np.dot(delta,delta)),.1,.9))
            shares[a]=(1-f)*pair_weight;shares[c]=f*pair_weight
        torque=np.array(b).reshape(4,3)-np.array(jz).reshape(4,3)*4.418*9.81*shares[:,None]
        requested=np.clip(np.degrees(gain*torque/35).ravel(),-4,4)
        self.state+=np.clip(requested-self.state,-.25,.25)
        return self.state.copy()

def trial(case):
    global SETTINGS
    SETTINGS=case
    import arc_preload
    arc_preload.ArcPreload=ContinuousPreload
    rec=Recorder();headings=[];end=case.get('end',10)
    def obs(now,r):
        rec(now,r)
        if 5<=now<end:
            R=r.plant.data.xmat[r.plant.model.body('robot').id].reshape(3,3)
            headings.append((now,float(np.arctan2(R[1,0],R[0,0]))))
    pad=json.loads(Path(__file__).with_name('foot_cushion_10mm.json').read_text())
    result,rows=run(0,case.get('yaw',-1000),end+4.02,profile='arcturn',cushion=pad,stop_at=end,observer=obs,
      parameter_overrides=dict(arc_trial=case.get('arc',[.021,.5,.04,0]),arc_preload_trial=[.3,0],command_delay_s=case.get('delay',.02),tracking_feedback_enabled=False))
    steady=[f for f in rec.frames if 5<=f['time_s']<end]
    steady_rows=[r for r in rows if 5<=r['time_s']<end]
    result['metrics']=metrics(steady,steady_rows);result['case']=case
    h=np.unwrap([x[1] for x in headings]);result['rotation_deg_s']=float(np.degrees(h[-1]-h[0])/(headings[-1][0]-headings[0][0]))
    result['stop_completed']=not rec.frames[-1]['moving'] and not rec.frames[-1]['transition']
    result['legs_steady']={}
    for leg in ('FL','FR','RL','RR'):
        swing=[r for r in steady_rows if r['leg']==leg and r['moving'] and r['phase']>=case.get('arc',[.021,.5,.04,0])[1]]
        middle=[r for r in swing if .2<=(r['phase']-.5)/.5<=.8]
        complete=[];active=None;previous=None
        for row in [r for r in steady_rows if r['leg']==leg and r['moving']]:
            q=row['phase']
            if previous is not None and previous<.5<=q:active=[]
            if active is not None and q>=.5:active.append(row['clearance_mm'])
            if previous is not None and previous>.8 and q<.2 and active:
                complete.append(max(active));active=None
            previous=q
        result['legs_steady'][leg]={'cycle_peak_min_mm':min(complete,default=None),'cycle_peak_p10_mm':float(np.percentile(complete,10)) if complete else None,'complete_swings':len(complete),'peak_mm':max((r['clearance_mm'] for r in swing),default=None),'contact':float(np.mean([r['force_n']>.2 for r in middle])) if middle else None}
    name=case['name'];(OUT/(name+'.json')).write_text(json.dumps(result,indent=2,default=float))
    m=result['metrics'];ls=list(result['legs_steady'].values())
    print(name,result['safety'],round(result['rotation_deg_s'],2),round(m['max_roll_deg'],2),round(m['max_pitch_deg'],2),[round(v['peak_mm'] or 0,1) for v in ls],[round(v['contact'] or 0,2) for v in ls],flush=True)
    return result

if __name__=='__main__':
    setup()
    if len(sys.argv)>1 and Path(sys.argv[1]).exists():cases=json.loads(Path(sys.argv[1]).read_text())
    else:
        cases=[{'name':f'w{w}-l{l}-g{g}','transfer':[w,l,g,0]} for w in (.04,.08,.12) for l in (0,.04) for g in (.3,.4)]
    with ProcessPoolExecutor(max_workers=2) as pool:list(pool.map(trial,cases))
