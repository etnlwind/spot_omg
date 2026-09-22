"""Verify V92 OTA and one guarded Landing; preserve settings and keep torque ON.

Uses the dedicated BLE control channel while draining console diagnostics.
Never retries motion, clears guards, changes profiles/settings, or walks.
"""
import argparse
import asyncio
import hashlib
import json
from pathlib import Path
import re
import secrets
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'apps/windows'), str(ROOT / 'tools/servo_tool')]
from spot_controller.connection import BleLink
from spot_controller.control_channel import ControlStream, request

REVISION = 'attitudepd-v6-v92'
PROFILE = 'attitudepd_v6'
LEGS = ('fl', 'fr', 'rl', 'rr')


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def parse_state(text):
    lines = [line for line in text.splitlines() if line.startswith('$SPOTSTATE ')]
    require(len(lines) == 1, 'Expected one correlated state reply: ' + text)
    return dict(word.split('=', 1) for word in lines[0].split()[1:] if '=' in word)


def registers(text, servo_id):
    match = re.search(rf'SERVO_CONFIG id={servo_id} addr=0: ((?:[0-9A-F]{{2}} ?)+)\r?\n', text)
    require(match is not None, f'Missing complete servo configuration for ID {servo_id}')
    raw = list(bytes.fromhex(match[1]))
    require(len(raw) >= 48, f'Truncated servo configuration for ID {servo_id}')
    return raw


def healthy_stationary(text):
    rows = {}
    for line in text.splitlines():
        match = re.fullmatch(r'ID (\d+) pos=(\d+) speed=(-?\d+) load=(-?\d+) voltage=(\d+)mV temp=(\d+)C current=(-?\d+) moving=(\d+) hw=0x([0-9A-Fa-f]+)', line)
        if not match:
            continue
        servo_id, position, speed, load, voltage, temperature, current, moving = map(int, match.groups()[:8])
        require(servo_id not in rows, f'Duplicate status for ID {servo_id}')
        row = dict(position=position, speed=speed, load=load, voltage_mv=voltage,
                   temperature_c=temperature, current=current, moving=moving,
                   hardware_error=int(match[9], 16))
        rows[servo_id] = row
        require(speed == 0 and moving == 0, f'ID {servo_id} is not stationary: {row}')
        require(voltage >= 10500 and temperature < 70 and abs(current) < 700 and row['hardware_error'] == 0,
                f'ID {servo_id} health check failed: {row}')
    require(set(rows) == set(range(1, 13)), 'Expected healthy status from all 12 servo IDs: ' + text)
    return rows


async def run(args):
    revision = getattr(args, 'revision', REVISION)
    profile = 'attitudepd_v7' if revision == 'attitudepd-v7-v93' else PROFILE
    args.output.mkdir(parents=True, exist_ok=True)
    evidence = args.output / 'post-verify.json'
    # Exclusive creation protects earlier evidence, including failed attempts.
    with evidence.open('x', encoding='utf-8') as file:
        file.write('{}\n')
    report = dict(stage='post-verify', success=False, expected_revision=revision,
                  expected_profile=profile, started_epoch=time.time(), events=[], console=[])
    radio = None
    drainer = None

    def save():
        evidence.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')

    try:
        image = args.image.read_bytes()
        digest = hashlib.sha256(image).hexdigest()
        report['sha256'] = digest
        require(revision.encode() in image and len(image) <= 320 * 1024, 'Expected the selected application image within the OTA slot')
        prior = json.loads((args.output / 'prepare.json').read_text())
        ota = json.loads((args.output / 'ota.json').read_text())
        require(prior.get('success') is True and prior.get('landing_confirmed') is True and prior.get('torque_off') is True,
                'Successful confirmed Landing and torque-OFF preparation required')
        require(ota.get('success') is True and prior.get('sha256') == ota.get('sha256') == digest,
                'Prepare, successful OTA, and image SHA-256 must match')
        saved_lift = prior['foot_lift_mm']
        require(len(saved_lift) == 4 and all(type(value) is int and 0 <= value <= 2147483647 for value in saved_lift),
                'Invalid preserved foot-height evidence')
        previous_registers = {}
        for servo_id in range(1, 13):
            events = [event for event in prior['events'] if event['command'] == f'servoconfig {servo_id}']
            require(len(events) == 1, f'Expected one prepared configuration for ID {servo_id}')
            event = events[0]
            raw = registers(''.join(event.get('data', [])) + event.get('diagnostic_reply', ''), servo_id)
            require(raw[40] == 0, f'Prepared ID {servo_id} was not torque OFF')
            previous_registers[servo_id] = raw
        report['foot_lift_mm'] = saved_lift
        save()

        radio = BleLink()
        stream = ControlStream()
        sequence = secrets.randbelow(0xffffff00) + 1
        await radio.open()
        require(radio.control_characteristic is not None, 'Dedicated BLE control channel required')
        radio.control_mode = True

        async def drain():
            while True:
                report['console'].append((await radio.read()).decode(errors='replace'))
                save()

        drainer = asyncio.create_task(drain())

        def check_drain():
            if drainer.done():
                drainer.result()
                raise ConnectionError('Console drain stopped unexpectedly')

        async def rpc(command):
            nonlocal sequence
            check_drain()
            sequence += 1
            seq = sequence
            event = dict(command=command, sequence=seq, data=[], control_chunks=[])
            report['events'].append(event)
            console_start = len(report['console'])
            save()
            await radio.write_control(request(seq, command.encode('ascii')))

            async def receive():
                while True:
                    chunk = await radio.read_control()
                    event['control_chunks'].append(chunk.decode(errors='replace'))
                    save()
                    # ControlStream rejects ERROR/unknown terminal kinds;
                    # only the correlated DONE below can complete a command.
                    for number, kind, payload in stream.feed(chunk):
                        if number != seq:
                            continue
                        if kind == 'DATA':
                            text = payload.decode(errors='replace')
                            event['data'].append(text)
                            require(not re.search(r'(?:ERROR:|STOPPED:|unknown command)', text), text)
                        elif kind == 'DONE':
                            return
                        else:
                            raise ConnectionError(f'Control terminal {kind}: {payload!r}')

            receiver = asyncio.create_task(receive())
            try:
                done, _ = await asyncio.wait({receiver, drainer}, timeout=75 if command == 'landing' else 15,
                                             return_when=asyncio.FIRST_COMPLETED)
                check_drain()
                require(receiver in done, f'Timed out waiting for {command}; no retry performed')
                await receiver
            finally:
                if not receiver.done():
                    receiver.cancel()
                    try:
                        await receiver
                    except asyncio.CancelledError:
                        pass
            text = ''.join(event['data'])
            if command == 'landing' or command.startswith('servoconfig '):
                marker = (r'POSE landing reason=[^\r\n]+\r?\n' if command == 'landing' else
                          rf'SERVO_CONFIG id={command.split()[1]} addr=0: (?:[0-9A-F]{{2}} ?){{48,}}\r?\n')
                deadline = time.monotonic() + 10
                while True:
                    check_drain()
                    logs = ''.join(report['console'][console_start:])
                    if re.search(marker, logs):
                        break
                    require(time.monotonic() < deadline, f'Missing complete diagnostics for {command}; no retry performed')
                    await asyncio.sleep(.25)
                event['diagnostic_reply'] = logs
                text += '\n' + logs
            save()
            require('ERROR:' not in text and 'STOPPED:' not in text, text)
            print(command + '\n' + text, flush=True)
            return text

        def check_state(state, torque, landed=False):
            require(state.get('rev') == revision and state.get('profile') == profile, f'Unexpected firmware/profile: {state}')
            require({'controlv1', 'footliftpersist', profile} <= set(state.get('caps', '').split(',')), f'Missing capabilities: {state}')
            require(state.get('safety') == 'ok' and state.get('fault_code') == '0' and state.get('torque') == torque,
                    f'Unexpected safety/torque state: {state}')
            require([int(state['lift_' + leg]) for leg in LEGS] == saved_lift, f'Saved foot heights changed: {state}')
            if revision in ('attitudepd-v6-v92-r1', 'attitudepd-v6-v92-r2', 'attitudepd-v7-v93'):
                require('footwidth' in state.get('caps', '').split(','), 'Missing footwidth capability')
                saved_width = [int(prior['initial'].get('width_' + leg, '0')) for leg in LEGS]
                require([int(state['width_' + leg]) for leg in LEGS] == saved_width,
                        f'Saved foot widths changed: {state}')
            if landed:
                require(state.get('pose') == 'landing' and int(state['error']) <= 40, f'Landing arrival not confirmed: {state}')

        initial = parse_state(await rpc('syncstate'))
        report['initial'] = initial
        check_state(initial, 'off')
        # Torque-OFF settling may report custom. The existing firmware guards
        # decide whether Landing is allowed; this script never bypasses them.
        report['initial_status'] = healthy_stationary(await rpc('status'))
        # This command records calibrated Stand targets, NOT Landing goals.
        await rpc('targets')
        check_state(parse_state(await rpc('footlift show')), 'off')
        landing = await rpc('landing')  # The only motion command; never retried.
        require(re.search(r'POSE landing reason=complete(?:-residual)?(?:\s|$)', landing), 'Landing did not complete: ' + landing)
        landed = parse_state(await rpc('syncstate'))
        check_state(landed, 'on', landed=True)
        report['landing_confirmed'] = True
        report['landed'] = landed
        report['servo_registers'] = {}
        for servo_id in range(1, 13):
            raw = registers(await rpc(f'servoconfig {servo_id}'), servo_id)
            report['servo_registers'][servo_id] = raw
            require(raw[:40] == previous_registers[servo_id][:40], f'ID {servo_id} registers 0..39 changed')
            require(raw[40] == 1, f'ID {servo_id} torque is not ON at Landing')
            save()
        report['final_status'] = healthy_stationary(await rpc('status'))
        final = parse_state(await rpc('footlift show'))
        check_state(final, 'on', landed=True)
        report['final'] = final
        report['permanent_registers_preserved'] = True
        report['foot_lift_preserved'] = True
        report['success'] = True
    except BaseException as exc:
        report['error'] = repr(exc)
        raise
    finally:
        if drainer:
            drainer.cancel()
            try:
                await drainer
            except (asyncio.CancelledError, Exception):
                pass
        if radio:
            try:
                await radio.close()
            except Exception as exc:
                report['disconnect_error'] = repr(exc)
                report['success'] = False
        report['finished_epoch'] = time.time()
        save()
    require(report['success'], 'Verification failed; inspect ' + str(evidence))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument('--revision', choices=[REVISION, 'attitudepd-v6-v92-r1', 'attitudepd-v6-v92-r2', 'attitudepd-v7-v93'], default=REVISION)
    asyncio.run(run(parser.parse_args()))
