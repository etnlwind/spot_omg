#!/usr/bin/env python3
"""Combine bootloader, committed metadata and relocated app for first SWD flash."""

from __future__ import annotations

import argparse
from pathlib import Path
import struct
import zlib


APP_OFFSET = 0x10000
METADATA_OFFSET = 0xC000
APP_CAPACITY = 320 * 1024
METADATA_MAGIC = 0x53504657
PROTOCOL_VERSION = 1


def build_factory_image(bootloader: bytes, app: bytes) -> bytes:
    if not bootloader or len(bootloader) > METADATA_OFFSET:
        raise ValueError("bootloader must fit below the sector-3 metadata")
    if len(app) < 8 or len(app) > APP_CAPACITY:
        raise ValueError("application must be between 8 bytes and 320 KiB")
    stack, reset = struct.unpack_from("<II", app)
    if not 0x20000000 <= stack <= 0x20020000:
        raise ValueError(f"invalid application stack pointer 0x{stack:08x}")
    if not reset & 1 or not 0x08010000 <= reset < 0x08060000:
        raise ValueError(f"application is not linked at 0x08010000: 0x{reset:08x}")

    image = bytearray(b"\xff" * (APP_OFFSET + len(app)))
    image[: len(bootloader)] = bootloader
    crc = zlib.crc32(app) & 0xFFFFFFFF
    metadata = struct.pack(
        "<IIII", METADATA_MAGIC, len(app), crc, PROTOCOL_VERSION
    )
    image[METADATA_OFFSET : METADATA_OFFSET + len(metadata)] = metadata
    image[APP_OFFSET:] = app
    return bytes(image)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bootloader", type=Path)
    parser.add_argument("application", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    combined = build_factory_image(
        args.bootloader.read_bytes(), args.application.read_bytes()
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(combined)
    print(f"factory image: {args.output} ({len(combined)} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
