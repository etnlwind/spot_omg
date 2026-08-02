"""Small, dependency-light STS3215 control package."""

from .bus import BROADCAST_ID, ServoBus
from .protocol import ProtocolError, ServoError
from .spot import GaitParameters, JointConfig, SpotConfig, SpotRobot
from .sts3215 import STS3215, ServoDiagnostics, ServoState

__all__ = [
    "BROADCAST_ID",
    "ProtocolError",
    "GaitParameters",
    "JointConfig",
    "SpotConfig",
    "SpotRobot",
    "STS3215",
    "ServoBus",
    "ServoError",
    "ServoDiagnostics",
    "ServoState",
]
