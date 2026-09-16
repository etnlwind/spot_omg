import pytest
from servo.imu_trace import parse,download
from types import SimpleNamespace

TEXT='$IT,M,1,2,0,0\n$IT,P,0,2,2\n$IT,S,0,4294967280,130,0,500,1000,1,0\n$IT,S,1,4,140,0,510,0,1,2\n$IT,END\n'
def test_wrap_and_fault():
    meta,rows=parse(TEXT)
    assert meta[1]==2 and rows[-1][-1]==2
@pytest.mark.parametrize('text',[TEXT.replace('$IT,END',''),TEXT.replace('$IT,S,1,','$IT,S,0,'),TEXT.replace('$IT,P,0,2,2','$IT,P,1,2,2'),TEXT.replace(',1,2\n',',9,2\n')])
def test_reject_corrupt_trace(text):
    with pytest.raises(ValueError):parse(text)
def test_download_read_only(tmp_path):
    class Console:
        def send(self,command,timeout):
            assert command=='imutrace dump 0 12'
            return SimpleNamespace(ok=True,text=TEXT)
    target=tmp_path/'imu.txt'
    _,rows=download(Console(),target)
    assert len(rows)==2 and target.with_suffix('.csv').exists()
