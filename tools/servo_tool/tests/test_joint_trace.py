import math
from pathlib import Path
import pytest
from servo.joint_trace import analyze,parse,write_report,wrapped_error,signed_target


def fixture(gap=20,lag=60,origin=0):
    n=251;commands=[];samples=[]
    def target(t):return round(2048+180*math.sin(t/370)+80*math.sin(t/133))
    for i in range(n):
        t=i*20;encoded=target(t)
        commands.append(f"$JT,C,{i},{(origin+t)&0xffffffff},{(origin+t+1)&0xffffffff},"+','.join([str(encoded)]*12))
        if t>=400 and t%gap==0:
            samples.append(f'$JT,S,{(origin+t+2)&0xffffffff},{(origin+t+4)&0xffffffff},{i},0,0,{target(t-lag)},0,15,20,11200,30,0')
    return '\n'.join([f'$JT,M,1,{n},{len(samples)},0,0,30,3400,254',*[f'$JT,J,{j},{j+1},2048,{1 if j%2==0 else -1}' for j in range(12)],*commands,*samples,'$JT,END'])


def test_dense_synthetic_delay_identified():
    data,_=analyze(fixture());r=data['joints']['FL-J1']
    assert r['delay_estimate_ms']==pytest.approx(60,abs=10)
    assert r['samples']>100
    assert r['min_voltage_mv']==11200


def test_sparse_feedback_does_not_claim_precise_delay():
    data,_=analyze(fixture(gap=240));r=data['joints']['FL-J1']
    assert r['delay_estimate_ms'] is None
    assert r['timing_resolution_bound_ms']>=120


def test_timer_rollover():
    a,_=analyze(fixture());b,_=analyze(fixture(origin=0xfffffff0))
    assert a['joints']==b['joints']


def test_signed_targets_and_wrap_are_distinct():
    assert signed_target(0x8000|937)==-937
    assert wrapped_error(1,4097)==0
    assert wrapped_error(4095,4097)==-2
    assert wrapped_error(3159,-937)==0


@pytest.mark.parametrize('edit',[
    lambda s:s.replace('$JT,END',''),
    lambda s:s.replace('$JT,C,2,', '$JT,C,3,'),
    lambda s:s.replace('$JT,J,0,1,2048,1','$JT,J,0,1,2048,0'),
    lambda s:s.replace('$JT,S,402,404,20,0','$JT,S,300,404,20,0'),
])
def test_reject_incomplete_or_misaligned_capture(edit):
    with pytest.raises(ValueError):parse(edit(fixture()))


def test_failed_reads_are_not_zero_positions():
    text=fixture().replace('$JT,S,402,404,20,0,0,','$JT,S,402,404,20,0,3,')
    data,rows=analyze(text)
    assert data['joints']['FL-J1']['failed_reads']==1
    assert all(r['time_ms']!=403 for r in rows)


def test_report_and_comparison(tmp_path):
    p=tmp_path/'synthetic.log';p.write_text(fixture())
    output=tmp_path/'out';report=write_report(p,output,p)
    assert report.exists() and (output/'samples.csv').exists()
    assert '비교 입력' in report.read_text()


def test_analysis_window_excludes_startup():
    data,rows=analyze(fixture(),1000,4000)
    assert rows and all(1000<=r['time_ms']<4000 for r in rows)
    with pytest.raises(ValueError):analyze(fixture(),6000,None)


def test_missing_joint_is_unknown_not_stationary():
    data,_=analyze(fixture())
    assert data['joints']['FR-J3']['observed_excursion_deg'] is None


def test_cli_analysis_never_resolves_a_robot(tmp_path,monkeypatch):
    from servo import cli
    def forbidden(*a,**k):raise AssertionError('offline analysis accessed robot transport')
    monkeypatch.setattr(cli,'resolve_transport',forbidden)
    p=tmp_path/'synthetic.log';p.write_text(fixture())
    assert cli._main(['analyze-joints',str(p),'--output',str(tmp_path/'report')])==0


def test_dump_gets_transfer_timeout():
    from servo.console import estimate_timeout
    assert estimate_timeout('jointtrace dump')==60


def test_paged_download_is_complete_and_read_only(tmp_path):
    from types import SimpleNamespace
    from servo.joint_trace import download
    original=fixture().splitlines();headers=[v for v in original if v.startswith(('$JT,M,','$JT,J,'))]
    data=[v for v in original if v.startswith(('$JT,C,','$JT,S,'))]
    calls=[]
    class Console:
        def send(self,command,timeout):
            calls.append(command);assert command.startswith('jointtrace dump ')
            offset=int(command.split()[2]);part=data[offset:offset+12]
            lines=(headers if offset==0 else [])+[f'$JT,P,{offset},{len(part)},{len(data)}']+part
            if offset+len(part)==len(data):lines+=['$JT,END']
            return SimpleNamespace(ok=True,lines=lines,text='\n'.join(lines))
    p=download(Console(),tmp_path/'trace.log')
    assert len(calls)>1
    assert analyze(p.read_text())[0]['joints']==analyze(fixture())[0]['joints']


def test_paged_download_rejects_missing_record(tmp_path):
    from types import SimpleNamespace
    from servo.joint_trace import download
    class Broken:
        def send(self,command,timeout):
            return SimpleNamespace(ok=True,lines=['$JT,P,0,1,2'],text='')
    p=tmp_path/'bad.log'
    with pytest.raises(ValueError,match='Incomplete'):download(Broken(),p)
    assert not p.exists() and p.with_suffix('.log.partial').exists()
