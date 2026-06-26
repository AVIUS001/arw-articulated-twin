"""Co-sim bridge stub — license boundary to proprietary WQP allocator.

Open the bits, protect the atoms: this repo ships the interface only.
The real ACCA constrained allocator runs off-board (Jetson / nox-ecs) and
connects over ardupilot-bridge. The open twin uses pseudo-inverse reference
allocation in-process for sim and RL; production co-sim swaps in the
proprietary stack behind this boundary.
"""
from __future__ import annotations

import json
import socket
import struct
from dataclasses import dataclass, asdict
from typing import Any

import numpy as np

from arw_articulated_twin.allocator import pseudo_inverse_allocate
from arw_articulated_twin.effectiveness import build_b_matrix


@dataclass
class BridgeCommand:
    omega_cmd_rad_s: list[float]
    beta_art_deg: list[float]
    timestamp_ns: int = 0


@dataclass
class BridgeTelemetry:
    pose: list[float]
    velocity: list[float]
    imu_accel: list[float]
    imu_gyro: list[float]
    joint_positions: list[float]
    joint_velocities: list[float]
    b_matrix_shape: list[int]
    sigma_max: float
    sigma_min: float


class ArdupilotBridgeStub:
    """UDP JSON stub — reference allocator in-process; replace with external WQP."""

    def __init__(self, vehicle_code: str, host: str = "127.0.0.1", port: int = 14560):
        self.vehicle_code = vehicle_code
        self.host = host
        self.port = port
        self._sock: socket.socket | None = None

    def bind(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self.host, self.port))
        self._sock.settimeout(0.001)

    def allocate_reference(self, wrench: np.ndarray, beta: dict[str, float]) -> np.ndarray:
        eff = build_b_matrix(self.vehicle_code, beta)
        result = pseudo_inverse_allocate(wrench, eff)
        return result.thrust_cmds

    def pack_command(self, cmd: BridgeCommand) -> bytes:
        return json.dumps(asdict(cmd)).encode("utf-8")

    def unpack_telemetry(self, data: bytes) -> BridgeTelemetry:
        d = json.loads(data.decode("utf-8"))
        return BridgeTelemetry(**d)

    def serve_once(self) -> dict[str, Any] | None:
        if self._sock is None:
            return None
        try:
            data, addr = self._sock.recvfrom(65535)
        except BlockingIOError:
            return None
        msg = json.loads(data.decode("utf-8"))
        wrench = np.array(msg.get("wrench_desired", [0, 0, 0, 0, 0, 0]), dtype=float)
        beta = msg.get("beta_art_deg", {})
        thrust = self.allocate_reference(wrench, beta)
        reply = {"thrust_alloc_n": thrust.tolist(), "allocator": "pseudo_inverse_reference"}
        self._sock.sendto(json.dumps(reply).encode("utf-8"), addr)
        return reply
