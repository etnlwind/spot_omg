"""Controlled V1 ablation. Preserve baseline and record every candidate."""
import copy,json
from pathlib import Path
from diagnose_turn_clearance import run
from validate_support_shift import Recorder,metrics
ROOT=Path(__file__).resolve().parent
OUT=ROOT.parents[1]/'artifacts/upright/2026-09-11/cushion'

def evaluate(name,profile,seconds=20.02):
    pad=json.loads((ROOT/'foot_cushion_10mm.json').read_text());rec=Recorder()
    result,rows=run(1000,0,seconds,profile=name,override=profile,cushion=pad,observer=rec)
    result['metrics']=metrics(rec.frames,rows)
    result['first_fault_s']=next((f['time_s'] for f in rec.frames if f['safety']!='ok'),None)
    (OUT/(name+'.json')).write_text(json.dumps(result,indent=2,default=lambda x:x.item()))
    print(name,result['safety'],result['first_fault_s'],result['metrics'],flush=True)
    return result

def main():
    v1=json.loads((ROOT/'upright_profiles.json').read_text())['profiles']['cushion_support_shift_v1']
    evaluate('v2-control-v1',v1)
    fixed=copy.deepcopy(v1);fixed['support_shift']['lock_j1']=True
    evaluate('v2-control-j1-only',fixed)
if __name__=='__main__':main()
