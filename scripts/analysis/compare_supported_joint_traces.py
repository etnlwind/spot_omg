"""Compare sampled joint excursion; sequential feedback cannot establish exact peak speed."""
import argparse,csv,json,math,bisect
from pathlib import Path

def summarize(path,start,end,grid_ms=0):
 rows=list(csv.DictReader(path.open()));result={}
 for name in dict.fromkeys(r['joint_name'] for r in rows):
  selected=[r for r in rows if r['joint_name']==name and start<=float(r['time_ms'])/1000<=end and int(r['status'])==0]
  t=[float(r['time_ms'])/1000 for r in selected];q=[float(r['actual_deg']) for r in selected];goal=[float(r['target_deg']) for r in selected]
  if len(q)<2:continue
  if grid_ms:
   all_rows=sorted((r for r in rows if r['joint_name']==name and int(r['status'])==0),key=lambda r:float(r['time_ms']))
   ts=[float(r['time_ms'])/1000 for r in all_rows]
   grid=[start+i*grid_ms/1000 for i in range(math.floor((end-start)*1000/grid_ms)+1)]
   if not ts[0]<=grid[0]<grid[-1]<=ts[-1]:raise ValueError('Grid exceeds recorded feedback; shorten the comparison window')
   def interp(key,x):
    i=max(0,min(len(ts)-2,bisect.bisect_right(ts,x)-1));u=(x-ts[i])/(ts[i+1]-ts[i])
    return (1-u)*float(all_rows[i][key])+u*float(all_rows[i+1][key])
   t=grid;q=[interp('actual_deg',x) for x in grid];goal=[interp('target_deg',x) for x in grid]
  intervals=[b-a for a,b in zip(t,t[1:])];distance=[abs(b-a) for a,b in zip(q,q[1:])]
  result[name]={'samples':len(q),'sample_span_s':t[-1]-t[0],
   'observed_min_deg':min(q),'observed_max_deg':max(q),'observed_excursion_deg':max(q)-min(q),
   'sampled_target_excursion_deg':max(goal)-min(goal),'rms_error_deg':math.sqrt(sum((a-b)**2 for a,b in zip(q,goal))/len(q)),
   'observed_total_travel_per_s':sum(distance)/(t[-1]-t[0]),
   'max_interval_average_speed_deg_s':max(d/dt for d,dt in zip(distance,intervals)),
   'max_feedback_interval_ms':max(intervals)*1000}
 return result

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--before',type=Path,required=True);ap.add_argument('--after',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--start',type=float,default=1.);ap.add_argument('--end',type=float,default=4.9);ap.add_argument('--grid-ms',type=float,default=0);a=ap.parse_args()
 if a.grid_ms<0:ap.error('grid-ms must be nonnegative')
 report={'before_path':str(a.before),'after_path':str(a.after),'window_s':[a.start,a.end],
 'limitations':['Sequential feedback; extrema between samples may be missed.','Different gait frequencies have different aliasing; sampled speed is not exact physical speed.','No simultaneous foot position or floor-contact measurement.'],
 'common_interpolated_grid_ms':a.grid_ms or None,
 'before':summarize(a.before,a.start,a.end,a.grid_ms),'after':summarize(a.after,a.start,a.end,a.grid_ms)}
 a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n')
 for name,v in report['after'].items():
  if name.endswith('J1'):continue
  b=report['before'][name]
  print(f"{name}: observed range {b['observed_excursion_deg']:.2f} -> {v['observed_excursion_deg']:.2f} deg; sampled travel/s {b['observed_total_travel_per_s']:.2f} -> {v['observed_total_travel_per_s']:.2f}; min {b['observed_min_deg']:.2f} -> {v['observed_min_deg']:.2f}; max {b['observed_max_deg']:.2f} -> {v['observed_max_deg']:.2f}")
if __name__=='__main__':main()
