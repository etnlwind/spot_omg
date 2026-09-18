"""Install the reviewed control bridge only after a fresh verified Landing/OFF."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace
from servo.cli import update_esp32_firmware

ROOT=Path(__file__).resolve().parents[2]
IMAGE=ROOT/'artifacts/control-channel-v81/esp32/control-bridge-v1.bin'
HASH='d52af40301d5970b4e272ac432d0bb9a5287f4866b558eecb9dd6137b7b53439'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--proof',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise RuntimeError('Preserve existing installation evidence')
    prior=json.loads(args.proof.read_text(encoding='utf-8'))
    assert prior['success'] and prior['completed_landing'] and prior['all_torque_off']
    assert 0<=time.time()-prior['finished_epoch']<600,'Landing/OFF preparation expired'
    assert 'rev=attitudepd-v4-v81' in prior['final_state'].split()
    assert hashlib.sha256(IMAGE.read_bytes()).hexdigest()==HASH
    report=dict(success=False,image_sha256=HASH,landing_proof=str(args.proof),
                started=datetime.now(timezone.utc).isoformat())
    def save():args.output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    save()
    try:
        result=update_esp32_firmware(SimpleNamespace(image=IMAGE,ble_name='SpotOMG-Bridge',
                                                   chunk_size=180,skip_landing=True))
        assert result==0
        report['success']=True
    except BaseException as exc:
        report['error']=repr(exc);raise
    finally:
        report['finished']=datetime.now(timezone.utc).isoformat();save()


if __name__=='__main__':main()
