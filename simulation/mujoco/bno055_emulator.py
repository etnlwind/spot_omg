"""BNO055 IMUPLUS Euler-output model, not Bosch's proprietary fusion algorithm.

Clock is simulation time. Sensor state is sampled at physics substeps, delivered
causally after fusion + I2C latency, then polled by the 50 Hz robot controller.
See docs/BNO055-EMULATION.md for measured/code-derived versus assumed parameters.
"""
from collections import deque
from dataclasses import asdict, dataclass
import math
import random
import struct


@dataclass(frozen=True)
class BNO055Config:
    sample_hz: float = 100.0
    fusion_delay_s: float = .020
    fusion_tau_s: float = .010
    i2c_hz: float = 100000.0
    noise_std_deg: float = .08
    roll_bias_deg: float = 0.0
    pitch_bias_deg: float = 0.0
    yaw_drift_deg_s: float = .02
    startup_s: float = .030  # configured IMUPLUS entry, not power-on boot
    stale_s: float = .100
    seed: int = 55

    def __post_init__(self):
        for name, value in asdict(self).items():
            if not math.isfinite(value):
                raise ValueError(name+' must be finite')
        if self.sample_hz <= 0 or self.i2c_hz <= 0 or self.stale_s <= 0:
            raise ValueError('rates and stale timeout must be positive')
        if min(self.fusion_delay_s, self.fusion_tau_s, self.noise_std_deg, self.startup_s) < 0:
            raise ValueError('delays and noise must be nonnegative')


class BNO055Emulator:
    def __init__(self, config=None):
        self.config = config or BNO055Config()
        self.rng = random.Random(self.config.seed)
        self.next_sample = 0.0
        self.queue = deque(maxlen=max(32, math.ceil((self.config.fusion_delay_s+81/self.config.i2c_hz+self.config.stale_s)*self.config.sample_hz)+2))
        self.fused = None
        self.latest = None
        self.online = True
        self.frozen = False
        self.level = (0, 0)
        self.samples = 0

    def advance(self, now, roll_deg, pitch_deg, yaw_deg=0.0):
        """Called before every physics step: never sample a future orientation."""
        c = self.config
        if now + 1e-9 < self.next_sample:
            return
        # No invented historical samples if a caller skips physics updates.
        self.next_sample = now + 1 / c.sample_hz
        if not self.online or self.frozen or now < c.startup_s:
            return
        truth = (roll_deg + c.roll_bias_deg, pitch_deg + c.pitch_bias_deg,
                 (yaw_deg + now*c.yaw_drift_deg_s) % 360)
        if self.fused is None:
            self.fused = list(truth)
        alpha = 1 if not c.fusion_tau_s else -math.expm1(-1/c.sample_hz/c.fusion_tau_s)
        for axis, value in enumerate(truth):
            delta = value-self.fused[axis]
            if axis == 2:
                delta = (delta+180) % 360-180
            self.fused[axis] += alpha*delta
        roll, pitch, yaw = [x+self.rng.gauss(0,c.noise_std_deg) for x in self.fused]
        # Inverse of the physical board's bno055.c mapping: sensor roll=robot
        # pitch; sensor pitch=robot roll. Only the upright walking envelope is
        # modeled; no claim to Bosch Euler branch behavior when inverted.
        raw = (round((yaw % 360)*16) % 5760,
               round(max(-180,min(180,pitch))*16),
               round(max(-180,min(180,roll))*16))
        packet = struct.pack('<hhh', *raw)
        # Address-write, register, address-read, six data bytes, each + ACK.
        available = now+c.fusion_delay_s+81/c.i2c_hz
        self.queue.append((available, now, packet))
        self.samples += 1

    def read(self, now):
        while self.queue and self.queue[0][0] <= now+1e-9:
            self.latest = self.queue.popleft()
        if not self.online or self.latest is None or now-self.latest[1] > self.config.stale_s:
            return None
        _, sampled, packet = self.latest
        yaw, sensor_roll, sensor_pitch = [math.trunc(x*10/16) for x in struct.unpack('<hhh',packet)]
        return dict(roll_tenths=sensor_pitch-self.level[0],
                    pitch_tenths=sensor_roll-self.level[1], yaw_tenths=yaw,
                    sample_time_s=sampled, age_ms=(now-sampled)*1000,
                    euler_register_hex=packet.hex())


class FirmwareAttitudeFilter:
    """Integer update equations from robot.c (20 ms, truncation toward zero)."""
    def __init__(self):
        self.previous = [0, 0]
        self.filtered = [0, 0]
        self.rate = [0, 0]
        self.failures = self.tilt_frames = 0
        self.initialized = False

    def update(self, reading):
        if reading is None:
            self.failures += 1
            return 'imu' if self.failures >= 3 else None
        self.failures = 0
        errors = [max(-300,min(300,reading[key])) for key in ('roll_tenths','pitch_tenths')]
        if not self.initialized:
            self.previous = errors.copy()
            self.initialized = True
        self.tilt_frames = self.tilt_frames+1 if max(map(abs,errors)) > 120 else 0
        for i, error in enumerate(errors):
            rate = max(-1200,min(1200,(error-self.previous[i])*50))
            self.previous[i] = error
            self.filtered[i] = math.trunc((3*self.filtered[i]+error)/4)
            self.rate[i] = math.trunc((3*self.rate[i]+rate)/4)
        return 'tilt' if self.tilt_frames >= 2 else None
