"""High-level STS3215 servo interface."""

from __future__ import annotations

import time
from dataclasses import dataclass

from .bus import ServoBus
from .protocol import decode_sign_magnitude, decode_u16, encode_u16


@dataclass(frozen=True)
class ServoState:
    position: int
    speed: int
    load: int
    voltage: float
    temperature: int
    hardware_error: int
    moving: bool
    current: int


@dataclass(frozen=True)
class ServoDiagnostics:
    operating_mode: int
    torque_enabled: bool
    acceleration: int
    goal_position: int
    goal_speed: int
    torque_limit: int
    state: ServoState


class STS3215:
    DEFAULT_BAUDRATE = 1_000_000
    STEPS_PER_REVOLUTION = 4096
    MIN_POSITION = 0
    MAX_POSITION = 4095

    ADDR_ID = 5
    ADDR_OPERATING_MODE = 33
    ADDR_TORQUE_ENABLE = 40
    ADDR_ACCELERATION = 41
    ADDR_GOAL_POSITION = 42
    ADDR_GOAL_TIME = 44
    ADDR_GOAL_SPEED = 46
    ADDR_TORQUE_LIMIT = 48
    ADDR_LOCK = 55
    ADDR_PRESENT_POSITION = 56
    ADDR_PRESENT_SPEED = 58
    ADDR_PRESENT_LOAD = 60
    ADDR_PRESENT_VOLTAGE = 62
    ADDR_PRESENT_TEMPERATURE = 63
    ADDR_HARDWARE_ERROR = 65
    ADDR_MOVING = 66
    ADDR_PRESENT_CURRENT = 69

    def __init__(self, bus: ServoBus, servo_id: int) -> None:
        if not 0 <= servo_id <= 253:
            raise ValueError("servo id must be between 0 and 253")
        self.bus = bus
        self.id = servo_id

    def ping(self) -> bool:
        return self.bus.ping(self.id)

    def _read_u8(self, address: int) -> int:
        return self.bus.read(self.id, address, 1)[0]

    def _read_u16(self, address: int) -> int:
        return decode_u16(self.bus.read(self.id, address, 2))

    def _write_u8(
        self, address: int, value: int, *, expect_response: bool = True
    ) -> None:
        if not 0 <= value <= 0xFF:
            raise ValueError("8-bit value must be between 0 and 255")
        self.bus.write(
            self.id, address, bytes((value,)), expect_response=expect_response
        )

    def _write_u16(self, address: int, value: int) -> None:
        self.bus.write(self.id, address, encode_u16(value))

    def read_id(self) -> int:
        return self._read_u8(self.ADDR_ID)

    def change_id(self, new_id: int, *, eeprom_wait: float = 1.0) -> None:
        """Change, lock, and verify the servo's EEPROM-backed ID."""
        if not 1 <= new_id <= 253:
            raise ValueError("new servo id must be between 1 and 253")
        if eeprom_wait < 0:
            raise ValueError("eeprom_wait cannot be negative")
        old_id = self.id
        if new_id == old_id:
            return
        if self.bus.ping(new_id):
            raise RuntimeError(f"servo ID {new_id} is already in use")

        self.unlock()
        try:
            # The response, if enabled, is sent using the old ID.
            self._write_u8(self.ADDR_ID, new_id)
            self.id = new_id
            self.lock()
            time.sleep(eeprom_wait)
            if not self.ping():
                raise RuntimeError(
                    f"ID write completed, but servo {new_id} did not respond"
                )
            if self.bus.ping(old_id):
                raise RuntimeError(
                    f"servo still responds to its previous ID {old_id}"
                )
        except Exception:
            # The write may have succeeded even when its acknowledgement was
            # lost. Lock whichever address responds so EEPROM is not left open.
            for candidate in (new_id, old_id):
                self.id = candidate
                if self.bus.ping(candidate):
                    try:
                        self.lock()
                    except Exception:
                        pass
                    break
            raise

    def lock(self) -> None:
        self._write_u8(self.ADDR_LOCK, 1)

    def unlock(self) -> None:
        self._write_u8(self.ADDR_LOCK, 0)

    def enable_torque(self, enabled: bool = True) -> None:
        self._write_u8(self.ADDR_TORQUE_ENABLE, int(enabled))

    def disable_torque(self) -> None:
        self.enable_torque(False)

    def move(
        self,
        position: int,
        *,
        speed: int = 0,
        acceleration: int = 0,
        time_ms: int = 0,
    ) -> None:
        if not self.MIN_POSITION <= position <= self.MAX_POSITION:
            raise ValueError("position must be between 0 and 4095")
        if not 0 <= speed <= 0xFFFF:
            raise ValueError("speed must be between 0 and 65535")
        if not 0 <= acceleration <= 0xFF:
            raise ValueError("acceleration must be between 0 and 255")
        if not 0 <= time_ms <= 0xFFFF:
            raise ValueError("time_ms must be between 0 and 65535")
        data = (
            bytes((acceleration,))
            + encode_u16(position)
            + encode_u16(time_ms)
            + encode_u16(speed)
        )
        self.bus.write(self.id, self.ADDR_ACCELERATION, data)

    def set_torque_limit(self, limit: int) -> None:
        self._write_u16(self.ADDR_TORQUE_LIMIT, limit)

    @property
    def position(self) -> int:
        return self._read_u16(self.ADDR_PRESENT_POSITION)

    @property
    def moving(self) -> bool:
        return bool(self._read_u8(self.ADDR_MOVING))

    def read_state(self) -> ServoState:
        # One contiguous read minimizes bus traffic and gives a coherent snapshot.
        raw = self.bus.read(
            self.id,
            self.ADDR_PRESENT_POSITION,
            self.ADDR_PRESENT_CURRENT + 2 - self.ADDR_PRESENT_POSITION,
        )
        offset = self.ADDR_PRESENT_POSITION

        def at(address: int, size: int = 1) -> bytes:
            start = address - offset
            return raw[start : start + size]

        load_raw = decode_u16(at(self.ADDR_PRESENT_LOAD, 2))
        load_magnitude = load_raw & 0x03FF
        return ServoState(
            position=decode_u16(at(self.ADDR_PRESENT_POSITION, 2)),
            speed=decode_sign_magnitude(at(self.ADDR_PRESENT_SPEED, 2)),
            load=-load_magnitude if load_raw & 0x0400 else load_magnitude,
            voltage=at(self.ADDR_PRESENT_VOLTAGE)[0] / 10.0,
            temperature=at(self.ADDR_PRESENT_TEMPERATURE)[0],
            hardware_error=at(self.ADDR_HARDWARE_ERROR)[0],
            moving=bool(at(self.ADDR_MOVING)[0]),
            current=decode_sign_magnitude(at(self.ADDR_PRESENT_CURRENT, 2)),
        )

    def read_diagnostics(self) -> ServoDiagnostics:
        return ServoDiagnostics(
            operating_mode=self._read_u8(self.ADDR_OPERATING_MODE),
            torque_enabled=bool(self._read_u8(self.ADDR_TORQUE_ENABLE)),
            acceleration=self._read_u8(self.ADDR_ACCELERATION),
            goal_position=self._read_u16(self.ADDR_GOAL_POSITION),
            goal_speed=self._read_u16(self.ADDR_GOAL_SPEED),
            torque_limit=self._read_u16(self.ADDR_TORQUE_LIMIT),
            state=self.read_state(),
        )
