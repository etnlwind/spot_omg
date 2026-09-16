import importlib.util
import json
from pathlib import Path
import pytest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('rr_analysis', ROOT/'scripts/analysis/analyze_rr_j3_response.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def trial(tmp_path, failed=False):
    records = [f'$JT,M,1,3,8,0,0,22,3400,10',
        *[f'$JT,J,{i},{i+1},2046,-1' for i in range(12)]]
    for n, begin, end, goal in [(0,0,0,1022),(1,200,202,1045),(2,700,702,1022)]:
        records.append(f'$JT,C,{n},{begin},{end},'+','.join([str(goal)]*12))
    for begin,end,command,position in [(180,182,0,1022),(190,192,0,1022),
        (204,206,1,1022),(214,216,1,1026),(684,686,1,1042),
        (704,706,2,1042),(714,716,2,1038),(1184,1186,2,1022)]:
        status=3 if failed and begin==214 else 0
        records.append(f'$JT,S,{begin},{end},{command},11,{status},{position},0,100,10,12000,35,0')
    (tmp_path/'jointtrace.txt').write_text('\n'.join(records+['$JT,END']))
    (tmp_path/'summary.json').write_text(json.dumps({'success':not failed}))


def test_small_step_reports_observation_bounds_not_fitted_acceleration(tmp_path):
    trial(tmp_path)
    report,rows=module.analyze_trial(tmp_path)
    assert report['sampling']['max_transaction_width_ms']==2
    assert report['phases'][0]['first_3tick_displacement_bounds_ms']==[2,16]
    assert report['phases'][0]['commanded_delta_deg']==pytest.approx(-23*360/4096)
    assert rows[3]['actual_deg']<rows[2]['actual_deg']  # positive RR ticks flex knee
    assert all('acceleration_deg_s2' not in phase for phase in report['phases'])


def test_failed_bus_read_cannot_establish_displacement(tmp_path):
    trial(tmp_path,failed=True)
    report,rows=module.analyze_trial(tmp_path)
    assert report['failed_reads']==1
    assert all(row['time_ms']!=215 for row in rows)
    assert report['phases'][0]['first_3tick_displacement_bounds_ms'][1]==486
