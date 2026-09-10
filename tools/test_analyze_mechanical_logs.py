from analyze_mechanical_logs import parse,report


def fixture(label,position,torque=0,complete=True):
    s=f'''MECH_BEGIN mode=pose label={label}
MECH_POS id=1 leg=0 j=1 pos={position} angle10=0 moving=0
MECH_PWR id=1 speed=0 load=0 current_raw=0 mv=12500 temp=25 hw=0
MECH_CAL id=1 ctr=1929 dir=-1 min=0 max=4095
MECH_GOAL id=1 goal=1929 err10=0 torque={torque}
'''
    return s+('MECH_END mode=pose valid=1 missing=0 stored=1\n' if complete else '')


def test_stand11_is_not_an_external_zero_measurement():
    groups=parse(fixture('stand11',1935)*2)
    text=report(groups)
    assert '| 1 | 2 | 1929 | 1935 |' not in text


def test_fixture_repeat_generates_review_only_center():
    text=report(parse(fixture('jig_plus',1935)+fixture('jig_minus',1937)),['jig_plus','jig_minus'])
    assert '| 1 | 2 | 1929 | 1936 | +7 | -0.615 | 0.176 |' in text
    assert '자동 적용하지 않습니다' in text


def test_incomplete_or_torque_on_reference_is_excluded():
    groups=parse(fixture('jig',1935)+fixture('jig',1937,torque=1)+fixture('jig',1939,complete=False))
    assert '| 1 | 2 | 1929 |' not in report(groups,['jig'])


def test_exported_duplicate_sequence_is_deduplicated():
    lines=fixture('jig',1935).splitlines()
    exported='\n'.join(f'$SPOTLOG seq={i} boot=5 uptime_ms=10 epoch_ms=0 text={line}' for i,line in enumerate(lines))
    assert len(parse(exported+'\n'+exported))==1
