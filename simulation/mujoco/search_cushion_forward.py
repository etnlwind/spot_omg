"""Compare farther touchdown with unchanged measured cushion and servo limits."""
import copy,json
from pathlib import Path
from diagnose_turn_clearance import run
ROOT=Path(__file__).resolve().parent
OUT=ROOT.parents[1]/'artifacts/upright/2026-09-11/cushion'

def main():
    base=json.loads((ROOT/'upright_profiles.json').read_text())['profiles']['cushion_j2lift']
    pad=json.loads((ROOT/'foot_cushion_10mm.json').read_text());results=[]
    for stride,offset in [(.06,-.01),(.05,0),(.06,-.005),(.065,-.01),(.07,-.01),(.055,-.005)]:
        profile=copy.deepcopy(base);profile['params'][2]=stride;profile['params'][5]=offset
        result,_=run(1000,0,24,profile='cushion_forward',override=profile,cushion=pad)
        results.append(result);(OUT/'forward-reach-search.json').write_text(json.dumps(results,indent=2))
        print(stride,offset,result['safety'],round(result['speed_m_s'],3),[(l,round(v['touchdown_forward_median_mm'] or 0,1),round(v['peak_tilt_deg'] or 0,1)) for l,v in result['legs'].items()],flush=True)

if __name__=='__main__':main()
