from pathlib import Path
import subprocess, hashlib, struct, json, argparse, re, sys

root = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description='Build STM32 application in a clean output directory')
parser.add_argument('--output', type=Path, default=Path('/private/tmp/spot-v13-build'))
args = parser.parse_args()
subprocess.run([sys.executable,str(root.parents[1]/'tools/generate_locomotion_profiles.py'),'--check'],check=True)
out = args.output.resolve()
revision = re.search(r'#define ROBOT_CONTROL_REV "([^"]+)"', (root/'Inc/robot.h').read_text()).group(1)
out.mkdir(parents=True, exist_ok=True)
tool = next(Path('/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins').glob('com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin/arm-none-eabi-gcc'))
prefix = str(tool)[:-3]
flags = ['-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard', '--specs=nano.specs']
includes = ['Inc','Drivers/STM32F4xx_HAL_Driver/Inc','Drivers/STM32F4xx_HAL_Driver/Inc/Legacy','Drivers/CMSIS/Device/ST/STM32F4xx/Include','Drivers/CMSIS/Include','Drivers/sh2']
objects = []
sources = (root/'firmware_objects.txt').read_text().splitlines()
with (out/'build.log').open('w') as log:
    def run(args):
        result = subprocess.run(args, stdout=log, stderr=subprocess.STDOUT)
        if result.returncode:
            raise RuntimeError('Build failed: ' + str(out/'build.log'))
    for obj in sources:
        src = root/str(Path(obj).with_suffix('.s' if obj.startswith('Startup/') else '.c'))
        dest = out/obj
        dest.parent.mkdir(parents=True, exist_ok=True)
        cmd = [str(tool), *flags, '-g3', '-DDEBUG', '-c']
        if src.suffix == '.c':
            cmd += ['-std=gnu11','-DUSE_HAL_DRIVER','-DSTM32F446xx','-O0','-ffunction-sections','-fdata-sections','-Wall','-Werror'] + ['-I'+str(root/p) for p in includes]
        else:
            cmd += ['-x','assembler-with-cpp']
        run(cmd + [str(src), '-o', str(dest)])
        objects.append(str(dest))
    elf = out/(revision+'.elf')
    run([str(tool), *flags, '-o',str(elf),*objects,'-T'+str(root/'STM32F446RETX_FLASH.ld'),'--specs=nosys.specs','-Wl,-Map='+str(out/(revision+'.map')),'-Wl,--gc-sections','-static','-Wl,--start-group','-lc','-lm','-Wl,--end-group'])
    binary = out/(revision+'.bin')
    run([prefix+'objcopy','-O','binary',str(elf),str(binary)])
data = binary.read_bytes()
sp, reset = struct.unpack('<II',data[:8])
assert 0x20000000 <= sp <= 0x20020000
assert reset & 1 and 0x08010000 <= reset < 0x08010000 + len(data)
assert len(data) <= 320*1024 and revision.encode() in data
report = dict(binary=str(binary),size=len(data),sha256=hashlib.sha256(data).hexdigest(),sp=hex(sp),reset=hex(reset))
(out/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
