"""Build the local macOS BLE peripheral app (no third-party runtime)."""
import argparse
import hashlib
import plistlib
import subprocess
from pathlib import Path


def build(output=Path('/private/tmp/SpotOMGSimBridge.app')):
    source=Path(__file__).with_name('Bridge.swift')
    digest=hashlib.sha256(source.read_bytes()+Path(__file__).read_bytes()).hexdigest()
    contents=output/'Contents'
    binary=contents/'MacOS/SpotOMGSimBridge'
    stamp=output.with_suffix('.sha256')
    # Arbitrary files directly under Contents are treated as nested code by codesign.
    (contents/'source.sha256').unlink(missing_ok=True)
    if binary.exists() and stamp.exists() and stamp.read_text()==digest:
        return binary
    binary.parent.mkdir(parents=True,exist_ok=True)
    info={
        'CFBundleIdentifier':'com.etnlwind.spotomg.simbridge',
        'CFBundleName':'SpotOMGSimBridge', 'CFBundleDisplayName':'Spot OMG MuJoCo BLE',
        'CFBundleExecutable':'SpotOMGSimBridge', 'CFBundlePackageType':'APPL',
        'CFBundleVersion':'1', 'CFBundleShortVersionString':'0.1.0',
        'LSMinimumSystemVersion':'12.0',
        'NSBluetoothAlwaysUsageDescription':'iPhone 앱의 가상 로봇 명령을 MuJoCo에 전달합니다.',
        'NSBluetoothPeripheralUsageDescription':'MuJoCo 가상 로봇을 Bluetooth 장치로 제공합니다.',
    }
    (contents/'Info.plist').write_bytes(plistlib.dumps(info))
    subprocess.run(['xcrun','swiftc','-module-cache-path','/private/tmp/SpotOMGSwiftModuleCache','-swift-version','5','-O','-framework','AppKit',
                    '-framework','CoreBluetooth','-framework','Network',
                    str(Path(__file__).with_name('Bridge.swift')),'-o',str(binary)],check=True)
    subprocess.run(['codesign','--force','--sign','-',str(output)],check=True)
    stamp.write_text(digest)
    return binary


if __name__=='__main__':
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--output',type=Path,default=Path('/private/tmp/SpotOMGSimBridge.app'))
    print(build(parser.parse_args().output))
