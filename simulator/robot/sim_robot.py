"""A simulated robot: physics + sensors + the REAL firmware core.

The right motor is mounted mirrored on the real chassis (both motor shafts
point outwards), so a positive duty turns it backwards and its encoder
counts down. That is reproduced here so the firmware's default polarity
flags (right_motor_invert = right_enc_invert = 1) are exercised exactly as
on the robot.
"""
from __future__ import annotations

import math

import numpy as np

from ..arena import Arena
from ..config import SimParams
from ..sensors.reflectance import ReflectanceArray
from .core_bridge import RobotCore
from .physics import DiffDrive


class SimRobot:
    def __init__(self, arena: Arena, p: SimParams, core: RobotCore | None = None):
        self.arena = arena
        self.p = p
        self.rng = np.random.default_rng(p.seed)
        sx, sy, sh = arena.start
        self.body = DiffDrive(p, self.rng, sx, sy, math.radians(sh))
        self.sensors = ReflectanceArray(arena, p, self.rng)
        self.core = core or self.new_core()
        self.t_us = 0
        self.button = False
        self.out = None
        self.log: list[str] = []
        self.raw = np.zeros(p.num_sensors)

    def new_core(self) -> RobotCore:
        c = RobotCore()
        for k, v in self.p.firmware_overrides().items():
            c.set_param(k, v)
        return c

    def power_cycle(self) -> None:
        """Simulate switching the robot off and on: RAM is lost, only the
        storage blob (flash) survives."""
        blob = self.core.storage()
        self.log.extend(self.core.pop_logs())
        self.core.close()
        self.core = self.new_core()
        self.core.set_storage(blob)
        self.core.boot()
        self.log.append(f"{self.t_us / 1e6:7.3f} SIM power cycle ({len(blob)} bytes in flash)")

    def place(self, x: float, y: float, heading_deg: float) -> None:
        self.body.place(x, y, math.radians(heading_deg))
        self.sensors.history.clear()

    def step(self) -> None:
        p = self.p
        b = self.body
        self.raw = self.sensors.read(b.x, b.y, b.theta)
        el, er = b.encoder_counts()
        if self.p.encoder_mode == 0:
            er = -er  # mirrored right motor: its quadrature encoder counts down going forward
        g, gok = b.gyro_dps()
        vbat = b.vbat if p.has_vbat_sense else 0.0  # 0 = not measured
        out = self.core.step(self.t_us, self.raw, el, er, vbat, self.button, g, gok)
        self.out = out
        b.step(out.duty_left, -out.duty_right, p.dt)
        self.t_us += int(p.dt * 1e6)
        logs = self.core.pop_logs()
        if logs:
            self.log.extend(logs)

    def run_until(self, cond, max_s: float) -> bool:
        end = self.t_us + int(max_s * 1e6)
        while self.t_us < end:
            self.step()
            if cond():
                return True
        return False

    def press(self, hold_s: float) -> None:
        """Press and release the start button (operator action)."""
        self.button = True
        n = int(hold_s / self.p.dt)
        for _ in range(n):
            self.step()
        self.button = False
        self.step()

    @property
    def led(self) -> bool:
        return bool(self.out and self.out.led_red)

    @property
    def pose(self):
        return self.body.x, self.body.y, self.body.theta
