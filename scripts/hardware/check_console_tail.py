"""Read-only BLE diagnostic burst + status recovery; never sends motion or Ctrl+C."""
import asyncio
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'apps/windows'))
from spot_controller.connection import BleLink
from spot_controller.protocol import ConsoleStream


async def main():
    out = ROOT/'artifacts/attitudepd-v4/windows-stop-tail-loss/readonly-query.json'
    if out.exists():
        raise RuntimeError('Preserve existing evidence before another attempt')
    report = {'started_at': datetime.now(timezone.utc).isoformat(), 'motion_sent': False,
              'commands': [], 'success': False}
    def save():
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    link = BleLink()
    async def bridge():
        uuid = '6e400004-b5a3-f393-e0a9-e50e24dcca9e'
        if link.client.services.get_characteristic(uuid) is None:
            return 'diagnostic characteristic unavailable'
        return bytes(await link.client.read_gatt_char(uuid)).decode(errors='replace')
    try:
        await link.open()
        report['bridge_before'] = await bridge()
        for command in ('syncstate\n', 'gaitdiag\n', '\nsyncstate\n'):
            event = {'command': command, 'chunks': [], 'prompt': False, 'text': ''}
            report['commands'].append(event)
            parser = ConsoleStream()
            started = time.monotonic()
            await link.write(command.encode())
            while time.monotonic()-started < 12:
                try:
                    data = await asyncio.wait_for(link.read(), 5)
                except asyncio.TimeoutError:
                    event['silence_timeout'] = True
                    break
                event['chunks'].append({'seconds': time.monotonic()-started,
                                        'bytes': len(data), 'text': data.decode(errors='replace')})
                event['text'] += data.decode(errors='replace')
                if any(kind == 'prompt' for kind, _ in parser.feed(data)):
                    if command != '\nsyncstate\n' or '$SPOTSTATE ' in event['text']:
                        event['prompt'] = True
                        break
            event['connected_after'] = link.client.is_connected
            event['bridge_after'] = await bridge()
            save()
            print(json.dumps({k:v for k,v in event.items() if k not in ('chunks','text')},ensure_ascii=False),flush=True)
            print(f"received={len(event['text'])} tail={event['text'][-180:]!r}",flush=True)
        report['success'] = True
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        await link.close()
        save()


if __name__ == '__main__':
    asyncio.run(main())
