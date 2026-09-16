"""One explicitly requested floor trial; never enables torque, stands or retries.

Requires v75 and a prepared, loaded Stand. Firmware performs one bounded RR J3
positive-raw-tick step/return and captures MCU bus timing. This step decreases
the configured RR joint angle; physical flexion direction/clearance was not
independently verified. Do not interpret it as a flexion test. Failure stops
the experiment; only read-only downloads follow. Review before re-running.
"""
import argparse
import json
import re
from pathlib import Path

from servo.console import Stm32Console
from servo.transport import BleTransport
from servo.joint_trace import download


def registers(text, servo_id):
    match = re.search(rf'SERVO_CONFIG id={servo_id} addr=0: ([0-9A-F ]+)', text)
    if not match:
        raise ValueError(f'Missing config for servo {servo_id}')
    raw = bytes.fromhex(match[1])
    if len(raw) < 48:
        raise ValueError('Truncated servo configuration')
    return list(raw)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--acceleration', type=int, choices=(10, 30, 50), required=True)
    p.add_argument('--delta-ticks', type=int, default=23)
    p.add_argument('--loaded-floor', action='store_true', required=True,
                   help='Operator confirmed weight on four feet, ready to catch a fall')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if not 12 <= a.delta_ticks <= 34:
        p.error('delta must be 12..34 ticks')
    a.output.mkdir(parents=True, exist_ok=False)
    events = []
    report = dict(acceleration=a.acceleration, delta_ticks=a.delta_ticks,
                  loaded_floor_confirmed=a.loaded_floor, success=False)
    def persist():
        (a.output/'events.json').write_text(json.dumps(events, indent=2), encoding='utf-8')
        (a.output/'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    persist()
    try:
        with Stm32Console('SpotOMG-Bridge', transport=BleTransport('SpotOMG-Bridge')) as c:
            def send(command):
                response = c.send(command, timeout=15)
                events.append(dict(command=command, status=response.status, text=response.text))
                persist()
                return response
            state = send('syncstate')
            required = {'rev=s-native-v6-2-6-v75', 'pose=stand', 'torque=on', 'safety=ok', 'fault_code=0'}
            if not state.ok or not required.issubset(state.text.split()):
                raise RuntimeError('Prepared v75 Stand not confirmed: '+state.text)
            imu = send('imucal status')
            if not imu.ok or 'IMUCAL device:' not in imu.text:
                raise RuntimeError('This v75 probe was validated with the fresh-reading BNO055 path only')
            before = {}
            for i in range(1, 13):
                response = send(f'servoconfig {i}')
                if not response.ok:
                    raise RuntimeError(response.text)
                before[i] = registers(response.text, i)
                if before[i][40] != 1:
                    raise RuntimeError(f'Servo {i} torque not on')
            report['registers_before'] = before
            response = send(f'responseprobe {a.acceleration} {a.delta_ticks}')
            report['response'] = response.text
            print(response.text, flush=True)
            # After any failure only observations follow; no automatic retry,
            # recover, pose command, or Relax from a loaded standing posture.
            report['state_after'] = send('syncstate').text
            report['status_after'] = send('status').text
            after = {}
            for i in range(1, 13):
                config = send(f'servoconfig {i}')
                if not config.ok:
                    raise RuntimeError(config.text)
                after[i] = registers(config.text, i)
            report['registers_after'] = after
            report['all_goal_profile_torque_bytes_restored'] = all(
                before[i][40:48] == after[i][40:48] for i in range(1, 13))
            trace_status = send('jointtrace status')
            report['trace_status'] = trace_status.text
            marker = re.search(r'\$JT,M,1,(\d+),(\d+),', trace_status.text)
            stage = re.search(r'\$RESPONSE stage=(\d+)', response.text)
            if stage and int(stage[1]) >= 1 and marker and int(marker[1]) > 0:
                download(c, a.output/'jointtrace.txt')
            report['success'] = (response.ok and 'restored=1' in response.text and
                report['all_goal_profile_torque_bytes_restored'] and
                {'pose=stand', 'torque=on', 'safety=ok'}.issubset(report['state_after'].split()))
            persist()
            if not report['success']:
                raise RuntimeError('Trial incomplete; inspect saved data before any further movement')
            print('Trial completed, all 12 original torque/goal/profile blocks verified', flush=True)
    except Exception as exc:
        report['error'] = str(exc)
        persist()
        raise


if __name__ == '__main__':
    main()
