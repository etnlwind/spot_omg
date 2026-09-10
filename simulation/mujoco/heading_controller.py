"""Heading hold via the HAL-independent C controller; delayed IMU input only."""
import ctypes

class HeadingController:
    def __init__(self, policy):
        self.enabled = True
        self.state = (ctypes.c_float * 6)()
        self.update_c = policy._library.spot_heading_update
        self.update_c.argtypes = (ctypes.POINTER(ctypes.c_float), ctypes.c_float,
            ctypes.c_int, ctypes.c_int, ctypes.c_float, ctypes.c_float,
            ctypes.c_float, ctypes.c_float)
        self.update_c.restype = ctypes.c_float

    def update(self, reading, linear, manual_yaw, applied_yaw, permitted=True):
        valid = reading is not None and reading.get('age_ms', 1e9) <= 100
        return self.update_c(self.state, reading['yaw_tenths']/10 if valid else 0,
            valid, self.enabled and permitted, linear, manual_yaw, applied_yaw, .02)

    def diagnostic(self):
        return dict(enabled=self.enabled, active=bool(self.state[5]),
                    reference_deg=self.state[0], error_deg=self.state[1],
                    correction=self.state[3])
