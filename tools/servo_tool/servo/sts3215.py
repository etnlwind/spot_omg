"""High-level STS3215 servo interface."""

from __future__ import annotations

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
    moving: bool
    current: int


class STS3215:
    DEFAULT_BAUDRATE = 1_000_000
    STEPS_PER_REVOLUTION = 4096
    MIN_POSITION = 0
    MAX_POSITION = 4095

    ADDR_ID = 5
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

    def change_id(self, new_id: int) -> None:
        if not 0 <= new_id <= 253:
            raise ValueError("new servo id must be between 0 and 253")
        old_id = self.id
        self.unlock()
        try:
            # The response, if enabled, is sent using the old ID.
            self._write_u8(self.ADDR_ID, new_id, expect_response=False)
            self.id = new_id
            if not self.ping():
                self.id = old_id
                raise RuntimeError(
                    f"ID write completed, but servo {new_id} did not respond"
                )
        finally:
            if self.id == new_id:
                self.lock()

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

        return ServoState(
            position=decode_u16(at(self.ADDR_PRESENT_POSITION, 2)),
            speed=decode_sign_magnitude(at(self.ADDR_PRESENT_SPEED, 2)),
            load=decode_sign_magnitude(at(self.ADDR_PRESENT_LOAD, 2)),
            voltage=at(self.ADDR_PRESENT_VOLTAGE)[0] / 10.0,
            temperature=at(self.ADDR_PRESENT_TEMPERATURE)[0],
            moving=bool(at(self.ADDR_MOVING)[0]),
            current=decode_sign_magnitude(at(self.ADDR_PRESENT_CURRENT, 2)),
        )
