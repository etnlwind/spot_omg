"""Offline turn endurance and direction/Stop transition verification; no hardware I/O."""
import json
from concurrent.futures import ProcessPoolExecutor
from check_wide80_turns import trial, ROOT, OUT
from diagnose_turn_clearance import run
from validate_support_shift import Recorder, metrics


def endurance(direction):
    return trial(direction,prefix='wide80-arc-endurance',stop_at=65.)


def transitions():
    config=json.loads((ROOT/'upright_profiles.json').read_text())['profiles']['cushion_diagonal_sync_wide80']
    pad=json.loads((ROOT/'foot_cushion_10mm.json').read_text());rec=Recorder();seen=[]
    def observe(now,robot):
        rec(now,robot)
        # Commands are sent through the same app protocol. The evaluation
        # observer supplies a fixed schedule, never plant-state feedback.
        if 3<=now<23:
            linear,yaw=(1000,0) if now<7 else ((500,1000) if now<11 else ((0,-1000) if now<17 else (0,1000)))
            robot.command(f'@D {round(now*100)+10000} {linear} {yaw}',now)
            seen.append([now,linear,yaw,robot.request])
        if abs(now-23.)<.001:robot.command('@S 99999',now)
    result,rows=run(0,0,28.,profile='cushion_diagonal_sync_wide80',override=config,cushion=pad,observer=observe,stop_at=28.)
    result['metrics']=metrics(rec.frames,rows)
    result['stop_completed']=not rec.frames[-1]['moving'] and not rec.frames[-1]['transition']
    result['commands']=seen[::50]
    (OUT/'wide80-arc-transitions.json').write_text(json.dumps(result,indent=2))
    return result

if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=2) as pool:
        results=list(pool.map(endurance,[-1,1]))
    (OUT/'wide80-arc-endurance-summary.json').write_text(json.dumps(results,indent=2))
    transitions()
