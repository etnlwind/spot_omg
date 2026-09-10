"""Paired-iPhone app control over CoreDevice, independent of the robot BLE link."""
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import uuid

BUNDLE = 'com.etnlwind.spotomg.controller'
CONFIG = Path.home()/'.config/spot_omg/app-control.json'


def configured_device():
    if os.environ.get('SPOT_IOS_DEVICE'):
        return os.environ['SPOT_IOS_DEVICE']
    try:
        return json.loads(CONFIG.read_text())['device']
    except (OSError, ValueError, KeyError):
        return None


def _device_call(arguments, directory, *, allow_failure=False):
    result_path=directory/'devicectl.json'
    result_path.unlink(missing_ok=True)
    try:
        result=subprocess.run(['xcrun','devicectl',*arguments,'--timeout','10',
                               '--json-output',str(result_path)],capture_output=True,text=True,timeout=15)
    except subprocess.TimeoutExpired as exc:
        if allow_failure:
            return False
        raise RuntimeError('Paired iPhone timed out; unlock the phone and check its developer connection') from exc
    if result.returncode and not allow_failure:
        raise RuntimeError('Paired iPhone operation failed: '+result.stderr.strip())
    return result.returncode==0


def request(device, action, target=None):
    if not device:
        raise ValueError('Choose a paired iPhone with --device, or run spotctl app pair --device UJIN17')
    if action not in ('status','connect','disconnect'):
        raise ValueError('Unsupported app control action')
    CONFIG.parent.mkdir(parents=True,exist_ok=True)
    with (CONFIG.parent/'app-control.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        with tempfile.TemporaryDirectory(prefix='spot-app-') as temp:
            directory=Path(temp)
            # First launch creates the mailbox on a fresh installation.
            _device_call(['device','process','launch','--device',device,BUNDLE],directory,allow_failure=True)
            identifier=str(uuid.uuid4());now=time.time()
            payload=dict(id=identifier,action=action,issuedAt=now,expiresAt=now+45,target=target)
            source=directory/'request.json';source.write_text(json.dumps(payload))
            common=['--device',device,'--domain-type','appDataContainer','--domain-identifier',BUNDLE]
            _device_call(['device','copy','to',*common,'--source',str(source),
                          '--destination','Documents/RemoteControl/request.json'],directory)
            # Activate an existing app without terminating it or losing a stop acknowledgement.
            _device_call(['device','process','launch','--device',device,'--payload-url',
                          'spotomg://remote-control',BUNDLE],directory,allow_failure=True)
            response=directory/'response.json'
            while time.time()<payload['expiresAt']+5:
                if _device_call(['device','copy','from',*common,'--source',
                    f'Documents/RemoteControl/response-{identifier}.json',
                    '--destination',str(response)],directory,allow_failure=True):
                    value=json.loads(response.read_text())
                    if value.get('request_id')!=identifier or value.get('action')!=action:
                        raise RuntimeError('App returned an unrelated response')
                    if not value.get('ok'):
                        raise RuntimeError('App control failed: '+str(value.get('error')))
                    return value
                time.sleep(.5)
            raise RuntimeError('App did not acknowledge the request; check that the iPhone is unlocked and this app build supports remote control')


def run(args):
    device=args.device or configured_device()
    value=request(device,'status' if args.action=='pair' else args.action,args.target)
    if args.action=='pair':
        CONFIG.write_text(json.dumps(dict(device=device),indent=2)+'\n')
        CONFIG.chmod(0o600)
    print(json.dumps(value,ensure_ascii=False,indent=2))
    return 0


@contextmanager
def robot_ble_lease(device):
    """Release the app before BLE work, then restore only a prior live connection."""
    if not device:
        yield
        return
    status=request(device,'status')
    restore=status.get('ready',False) and status.get('target')=='robot'
    if status.get('target')=='robot':
        released=request(device,'disconnect')
        if released.get('ready'):
            raise RuntimeError('App still owns the robot connection')
        print('App robot connection released',flush=True)
    try:
        yield
    except BaseException:
        # An incomplete firmware operation must not trigger an automatic connection.
        raise
    else:
        if restore:
            connected=request(device,'connect','robot')
            if not connected.get('ready'):
                raise RuntimeError('BLE operation completed, but app reconnection was not confirmed')
            print('App robot connection restored',flush=True)
