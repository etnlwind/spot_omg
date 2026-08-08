#!/bin/bash
# build_any.sh <project-dir> <output-dir>
set -u
PROJ=$1
OUT=$2
TOOLS=/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.macosaarch64_1.0.0.202602081740/tools/bin
GCC=$TOOLS/arm-none-eabi-gcc
rm -rf "$OUT"; mkdir -p "$OUT"
cd "$PROJ" || exit 1

FLAGS=(
  -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F446xx -c -O0 -g3
  -ffunction-sections -fdata-sections -Wall
  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb
  -IInc
  -IDrivers/STM32F4xx_HAL_Driver/Inc
  -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy
  -IDrivers/CMSIS/Device/ST/STM32F4xx/Include
  -IDrivers/CMSIS/Include
)

fail=0
for f in Src/*.c Drivers/STM32F4xx_HAL_Driver/Src/*.c; do
  case "$f" in *_template.c) continue;; esac
  "$GCC" "${FLAGS[@]}" "$f" -o "$OUT/$(basename "${f%.c}").o" 2>>"$OUT/err.log" || { echo "FAILED $f"; fail=1; }
done
"$GCC" -mcpu=cortex-m4 -c -x assembler-with-cpp -mfpu=fpv4-sp-d16 \
  -mfloat-abi=hard -mthumb Startup/*.s -o "$OUT/startup.o" 2>>"$OUT/err.log" || fail=1

"$GCC" -o "$OUT/firmware.elf" "$OUT"/*.o \
  -mcpu=cortex-m4 -T"$PROJ/STM32F446RETX_FLASH.ld" --specs=nosys.specs \
  -Wl,--gc-sections -static -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb \
  -Wl,--start-group -lc -lm -Wl,--end-group 2>>"$OUT/err.log" || fail=1

if [ -f "$OUT/firmware.elf" ]; then
  "$TOOLS/arm-none-eabi-objcopy" -O binary "$OUT/firmware.elf" "$OUT/firmware.bin"
  "$TOOLS/arm-none-eabi-objcopy" -O ihex "$OUT/firmware.elf" "$OUT/firmware.hex"
  "$TOOLS/arm-none-eabi-size" "$OUT/firmware.elf"
fi
echo "errors: $(grep -cE ' error:' "$OUT/err.log" 2>/dev/null || echo 0)"
echo "fail=$fail"
