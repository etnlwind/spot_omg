"""Build/sign/install V4 app on a paired Mac; does not launch or drive the robot."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
PROJECT = ROOT/'apps/ios/SpotOMGController/SpotOMGController.xcodeproj'
BUNDLE = 'com.etnlwind.spotomg.controller'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='00008150-00141904267A401C', help='Paired UJIN17 identifier, or another explicitly selected phone')
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/attitudepd-v4/ios-install')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    build = args.output/'build'
    app = build/'Build/Products/Debug-iphoneos/SpotOMGController.app'
    versions = re.findall(r'CURRENT_PROJECT_VERSION = (\d+);', (PROJECT/'project.pbxproj').read_text())
    if not versions or set(versions) != {'51'}:
        raise RuntimeError('Expected reviewed V4 app build 51')
    commands = [
        ['xcodebuild', '-project', str(PROJECT), '-scheme', 'SpotOMGController',
         '-configuration', 'Debug', '-destination', f'platform=iOS,id={args.device}',
         '-derivedDataPath', str(build), '-allowProvisioningUpdates', 'build'],
        ['xcrun', 'devicectl', 'device', 'install', 'app', '--device', args.device,
         str(app), '--json-output', str(args.output/'install.json')],
        ['xcrun', 'devicectl', 'device', 'info', 'apps', '--device', args.device,
         '--json-output', str(args.output/'apps.json')],
    ]
    if args.dry_run:
        print(json.dumps(dict(device=args.device, bundle=BUNDLE, build='51', commands=commands), indent=2))
        return
    if platform.system() != 'Darwin' or not shutil.which('xcodebuild') or not shutil.which('xcrun'):
        raise RuntimeError('A Mac with Xcode and a paired, unlocked iPhone is required')
    if args.output.exists():
        raise RuntimeError('Use a new --output directory to preserve earlier installation evidence')
    args.output.mkdir(parents=True)
    for name, command in zip(('build', 'install', 'readback'), commands):
        print(name+': '+subprocess.list2cmdline(command), flush=True)
        with (args.output/(name+'.log')).open('w') as log:
            subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
    payload = json.loads((args.output/'apps.json').read_text())

    def dictionaries(value):
        if isinstance(value, dict):
            yield value
            for child in value.values():
                yield from dictionaries(child)
        elif isinstance(value, list):
            for child in value:
                yield from dictionaries(child)

    installed = [entry for entry in dictionaries(payload) if entry.get('bundleIdentifier') == BUNDLE]
    if len(installed) != 1 or installed[0].get('bundleVersion') != '51':
        raise RuntimeError('Installation command completed but app build 51 readback was not confirmed')
    report = dict(success=True, verified_at=datetime.now(timezone.utc).isoformat(),
                  app=installed[0], launched=False, physical_robot_test=False)
    (args.output/'verified.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
