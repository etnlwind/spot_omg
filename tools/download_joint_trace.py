"""Read the frozen V47 joint trace over BLE in verified, bounded pages."""
import argparse
from pathlib import Path
from servo.console import Stm32Console
from servo.transport import BleTransport
from servo.joint_trace import download

def main():
    p=argparse.ArgumentParser(__doc__);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    with BleTransport('SpotOMG-Bridge') as transport:
        console=Stm32Console('SpotOMG-Bridge',transport=transport);console.sync()
        print(download(console,args.output))
if __name__=='__main__':main()
