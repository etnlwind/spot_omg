"""Static contracts for the power-loss-safe STM32 BLE update layout."""

from pathlib import Path
import struct

import pytest


ROOT = Path(__file__).resolve().parents[3]
APP = ROOT / "firmware" / "stm32-learning"
BOOT = ROOT / "firmware" / "stm32-ota-bootloader"


def test_application_is_relocated_and_reserves_calibration_sector() -> None:
    linker = (APP / "STM32F446RETX_FLASH.ld").read_text()
    system = (APP / "Src" / "system_stm32f4xx.c").read_text()
    assert "ORIGIN = 0x08010000" in linker
    assert "LENGTH = 320K" in linker
    assert "LENGTH = 0x1FFF0" in linker
    assert "#define USER_VECT_TAB_ADDRESS" in system
    assert "#define VECT_TAB_OFFSET         0x00010000U" in system


def test_bootloader_never_erases_its_own_or_calibration_sectors() -> None:
    source = (BOOT / "bootloader.c").read_text()
    assert "METADATA_ADDRESS     0x0800C000UL" in source
    assert "APP_ADDRESS          0x08010000UL" in source
    assert "APP_LIMIT_ADDRESS    0x08060000UL" in source
    assert "flash_erase_sector(3U)" in source
    assert "flash_erase_sector(4U)" in source
    assert "flash_erase_sector(5U)" in source
    assert "flash_erase_sector(6U)" in source
    assert "flash_erase_sector(7U)" not in source


def test_host_rejects_an_image_linked_at_the_legacy_address(tmp_path) -> None:
    import sys

    sys.path.insert(0, str(ROOT / "tools" / "servo_tool"))
    from servo.cli import _validate_stm32_application

    valid = tmp_path / "valid.bin"
    valid.write_bytes(struct.pack("<II", 0x2001FFF0, 0x08010101))
    _validate_stm32_application(valid, valid.stat().st_size)

    legacy = tmp_path / "legacy.bin"
    legacy.write_bytes(struct.pack("<II", 0x2001FFF0, 0x08000101))
    with pytest.raises(ValueError, match="not linked"):
        _validate_stm32_application(legacy, legacy.stat().st_size)


def test_spotctl_exposes_the_stm32_firmware_target() -> None:
    import sys

    sys.path.insert(0, str(ROOT / "tools" / "servo_tool"))
    from servo.cli import parse_args

    args = parse_args(["firmware", "stm32", "firmware.bin"])
    assert args.firmware_target == "stm32"
    assert args.chunk_size == 180


def test_factory_image_commits_the_relocated_application() -> None:
    import sys

    sys.path.insert(0, str(BOOT))
    from make_factory_image import APP_OFFSET, METADATA_OFFSET, build_factory_image

    bootloader = b"bootloader"
    app = struct.pack("<II", 0x2001FFF0, 0x08010101) + b"application"
    image = build_factory_image(bootloader, app)
    magic, size, _crc, version = struct.unpack_from("<IIII", image, METADATA_OFFSET)
    assert image[: len(bootloader)] == bootloader
    assert image[APP_OFFSET:] == app
    assert magic == 0x53504657
    assert size == len(app)
    assert version == 1
