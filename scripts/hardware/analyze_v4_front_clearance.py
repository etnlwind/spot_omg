"""Host-only production-C geometry audit; sends no robot commands."""
import csv
import io
import json
from pathlib import Path
import subprocess
import tempfile
from servo.host_build import build_executable

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'artifacts/v4-front-clearance-2026-09-21'
SOURCE = r'''
#include <stdio.h>
#include "locomotion.h"
#include "locomotion_servo.h"
int main(void) {
 int profile=locomotion_profile_id("attitudepd_v4");
 const float offset[4]={0,.5f,.5f,0},delta[4]={.01f,.01f,0,0};
 puts("linear,local_phase,leg,j1,j2,j3,x_mm,y_mm,z_mm,extra_z_mm,extra_xy_error_mm,option_ok,servo_ok");
 for(int direction=-1;direction<=1;direction+=2)for(int t=0;t<=1000;t++)for(int leg=0;leg<4;leg++){
  float local=t/1000.f,phase=local-offset[leg],p[7],point[3],changed[3];
  GaitPolicyLegTarget q[4],extra[4];uint16_t ticks[12];
  locomotion_params(profile,direction,p);
  if(!locomotion_targets(profile,phase,1,direction,0,q))return 2;
  float angles[3]={q[leg].j1_deg,q[leg].j2_deg,q[leg].j3_deg};
  arc_foot(leg,angles,point,0);
  int ok=arc_swing_shape_apply(q,phase,p[1],1,offset,delta,extra),servo=0;
  float dz=0,xy=0;
  if(ok){
   servo=locomotion_servo_targets(extra,ticks);
   float a[3]={extra[leg].j1_deg,extra[leg].j2_deg,extra[leg].j3_deg};
   arc_foot(leg,a,changed,0);dz=(changed[2]-point[2])*1000;
   xy=hypotf(changed[0]-point[0],changed[1]-point[1])*1000;
  }
  printf("%d,%.6f,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d\n",direction,local,leg,angles[0],angles[1],angles[2],point[0]*1000,point[1]*1000,point[2]*1000,dz,xy,ok,servo);
 }
}
'''

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory() as temp:
        source=Path(temp)/'audit.c';source.write_text(SOURCE)
        binary=build_executable([source,ROOT/'firmware/stm32-learning/Src/robot_config.c'],
            ROOT/'firmware/stm32-learning/Inc',Path(temp)/'audit',extra=('-ffp-contract=off',))
        raw=subprocess.check_output([str(binary)],text=True)
    (OUT/'geometry.csv').write_text(raw)
    rows=list(csv.DictReader(io.StringIO(raw)))
    summary={'scope':'Host CAD geometry, full amplitude, straight full input; no PD, physics or robot motion',
             'option':'Analysis-only front +10mm C2 swing envelope; production policy unchanged', 'legs':[]}
    for direction in (-1,1):
        for leg,name in enumerate(('FL','FR','RL','RR')):
            subset=[r for r in rows if int(r['linear'])==direction and int(r['leg'])==leg]
            z=[float(r['z_mm']) for r in subset]
            # Height excursion is in the fixed CAD body frame, not world ground clearance.
            summary['legs'].append(dict(linear=direction,leg=name,z_min_mm=min(z),z_max_mm=max(z),
                body_frame_z_excursion_mm=max(z)-min(z),
                added_z_peak_mm=max(float(r['extra_z_mm']) for r in subset),
                option_xy_error_max_mm=max(float(r['extra_xy_error_mm']) for r in subset),
                option_failures=sum(r['option_ok']!='1' or r['servo_ok']!='1' for r in subset)))
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
