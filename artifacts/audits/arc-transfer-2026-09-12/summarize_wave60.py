import csv,json
from pathlib import Path
root=Path('artifacts/audits/arc-control-2026-09-12');out=Path('artifacts/audits/arc-transfer-2026-09-12')
summary=[]
for p in sorted(root.glob('wave60_rank*_y*.json')):
 r=json.loads(p.read_text());m=r['metrics'];legs=r['contact_metrics']['phase_bases']['target']['legs']
 raw=list(csv.DictReader(p.with_suffix('.csv').open()));post=[x for x in raw if float(x['time_s'])>=65]
 gates=dict(uninterrupted=r['safety']=='ok' and not m['safety_faults'] and m['moving_fraction']==1,
     speed=abs(r['rotation_deg_s'])>=17.4,
     max_tilt=m['max_roll_deg']<=3 and m['max_pitch_deg']<=3,
     rms_tilt=m['rms_roll_deg']<=1.5 and m['rms_pitch_deg']<=1.5,
     every_swing_clearance=all(v['swing_peak_clearance_mm']['min'] is not None and v['swing_peak_clearance_mm']['min']>=15 for v in legs.values()),
     middle_contact=all(v['middle_swing_contact_fraction'] is not None and v['middle_swing_contact_fraction']<.1 for v in legs.values()),
     stop=r['stop_completed'] and all(x['safety']=='ok' for x in post))
 r['validated']=all(gates.values());r['gates']=gates;r['tracking_and_slip_comparison']='Not compared here; raw absolute metrics retained.'
 p.write_text(json.dumps(r,indent=2))
 row=dict(name=p.stem,rotation_deg_s=r['rotation_deg_s'],max_roll_deg=m['max_roll_deg'],max_pitch_deg=m['max_pitch_deg'],rms_roll_deg=m['rms_roll_deg'],rms_pitch_deg=m['rms_pitch_deg'],tracking_rms_deg=m['tracking_rms_deg'],stance_center_speed_proxy_m_s=m['stance_center_speed_proxy_m_s'],stop_completed=r['stop_completed'],failed_gates=';'.join(k for k,v in gates.items() if not v))
 for l,v in legs.items():
  for k in ('min','p10','median','max'):row[f'{l}_peak_{k}_mm']=v['swing_peak_clearance_mm'][k]
  row[f'{l}_midcontact']=v['middle_swing_contact_fraction'];row[f'{l}_swings']=v['complete_swings']
 summary.append(row)
 if p.stem.endswith('y-1000'):
  frames=[x for x in raw if x['leg']=='FL'];print(p.stem,'rollbins',[(t,round(max(abs(float(x['roll_deg'])) for x in frames if t<=float(x['time_s'])<t+10),3)) for t in (5,15,25,35,45,55)])
 print(row['name'],round(row['rotation_deg_s'],2),round(row['max_roll_deg'],2),round(row['max_pitch_deg'],2),[round(row[f'{l}_peak_min_mm'],2) if row[f'{l}_peak_min_mm'] is not None else None for l in legs],[round(row[f'{l}_peak_p10_mm'],2) if row[f'{l}_peak_p10_mm'] is not None else None for l in legs],[round(row[f'{l}_midcontact']*100,1) if row[f'{l}_midcontact'] is not None else None for l in legs],row['failed_gates'])
if summary:
 with (out/'wave60-summary.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=list(summary[0]));w.writeheader();w.writerows(summary)
 (out/'wave60-summary.json').write_text(json.dumps(summary,indent=2))
