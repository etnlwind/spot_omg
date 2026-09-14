"""BNO055 IMUPLUS Euler-output model, not Bosch's proprietary fusion algorithm.

Clock is simulation time. Sensor state is sampled at physics substeps, delivered
causally after fusion + I2C latency, then polled by the 50 Hz robot controller.
See docs/BNO055-EMULATION.md for measured/code-derived versus assumed parameters.
"""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
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
    gyro_delay_s: float = .003  # assumed register/filter + transfer delay
    gyro_noise_std_deg_s: float = .15

    def __post_init__(self):
        for name, value in asdict(self).items():
            if not math.isfinite(value):
                raise ValueError(name+' must be finite')
        if self.sample_hz <= 0 or self.i2c_hz <= 0 or self.stale_s <= 0:
            raise ValueError('rates and stale timeout must be positive')
        if min(self.fusion_delay_s, self.fusion_tau_s, self.noise_std_deg, self.startup_s,
               self.gyro_delay_s, self.gyro_noise_std_deg_s) < 0:
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
        self.gyro_rng = random.Random(self.config.seed + 1)
        self.gyro_queue = deque(maxlen=self.queue.maxlen)
        self.latest_gyro = None

    def advance(self, now, roll_deg, pitch_deg, yaw_deg=0.0, gyro_body_rad_s=None):
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
        # This channel samples body angular velocity, NEVER Euler differences.
        # Independent RNG preserves existing Euler-only reproducibility.
        if gyro_body_rad_s is not None:
            if len(gyro_body_rad_s) != 3 or not all(math.isfinite(v) for v in gyro_body_rad_s):
                raise ValueError('gyro requires three finite body-frame rad/s values')
            gyro = [round(max(-2048, min(2047.9375, math.degrees(v) +
                    self.gyro_rng.gauss(0, c.gyro_noise_std_deg_s))) * 16)
                    for v in gyro_body_rad_s]
            self.gyro_queue.append((now+c.gyro_delay_s, now, self.samples, gyro))

    def read(self, now):
        while self.gyro_queue and self.gyro_queue[0][0] <= now+1e-9:
            self.latest_gyro = self.gyro_queue.popleft()
        while self.queue and self.queue[0][0] <= now+1e-9:
            self.latest = self.queue.popleft()
        if not self.online or self.latest is None or now-self.latest[1] > self.config.stale_s:
            return None
        _, sampled, packet = self.latest
        yaw, sensor_roll, sensor_pitch = [math.trunc(x*10/16) for x in struct.unpack('<hhh',packet)]
        result = dict(roll_tenths=sensor_pitch-self.level[0],
                    pitch_tenths=sensor_roll-self.level[1], yaw_tenths=yaw,
                    sample_time_s=sampled, age_ms=(now-sampled)*1000,
                    euler_register_hex=packet.hex())
        gyro = self.latest_gyro
        if gyro is not None and now-gyro[1] <= self.config.stale_s:
            result.update(gyro_body_rad_s=[math.radians(v/16) for v in gyro[3]],
                          gyro_sample_time_s=gyro[1], gyro_age_ms=(now-gyro[1])*1000,
                          gyro_sequence=gyro[2], gyro_axis_verified=True)
        return result


class FirmwareAttitudeFilter:
    """Integer update equations from robot.c (20 ms, truncation toward zero)."""
    def __init__(self):
        self.previous = [0, 0]
        self.filtered = [0, 0]
        self.rate = [0, 0]
        self.failures = self.tilt_frames = 0
        self.initialized = False

    def update(self, reading):
        import ctypes
        from simulation.mujoco.runtime.gait_profiles import _shared
        fn=_shared()[0]._library.spot_attitude_update
        fn.argtypes=(ctypes.POINTER(ctypes.c_int),ctypes.c_int,ctypes.c_int,ctypes.c_int)
        fn.restype=ctypes.c_int
        state=(ctypes.c_int*9)(*self.previous,*self.filtered,*self.rate,self.failures,self.tilt_frames,self.initialized)
        result=fn(state,reading is not None,reading['roll_tenths'] if reading else 0,reading['pitch_tenths'] if reading else 0)
        self.previous=list(state[:2]);self.filtered=list(state[2:4]);self.rate=list(state[4:6])
        self.failures,self.tilt_frames=state[6:8];self.initialized=bool(state[8])
        return {0:None,1:'imu',2:'tilt'}[result]
