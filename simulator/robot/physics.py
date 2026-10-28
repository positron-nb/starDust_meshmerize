"""Differential-drive physics: two DC gearmotors with first-order dynamics,
battery sag, wheel-diameter error, random and acceleration-induced slip,
and quadrature encoders that measure WHEEL rotation (not ground motion).

Known simplifications (see simulator/README.md): no lateral slip/skid in
turns, no caster drag, no motor current limit inside the driver, no
backlash, perfectly flat floor.
"""
from __future__ import annotations

import math

import numpy as np

from ..config import SimParams


class DiffDrive:
    def __init__(self, p: SimParams, rng: np.random.Generator, x: float, y: float, theta: float):
        self.p = p
        self.rng = rng
        self.place(x, y, theta)
        self.enc_l = 0.0
        self.enc_r = 0.0
        self.pulses_l = 0.0  # slot-disc pulses (always counting up)
        self.pulses_r = 0.0
        self.w_l = 0.0  # wheel rim speed at NOMINAL diameter, mm/s
        self.w_r = 0.0
        self.vbat = p.battery_full_v
        self.energy_mAh = 0.0
        self.odometer = 0.0  # true distance travelled by the axle centre (mm)
        self.current = 0.0
        self.jammed = False  # fault injection: wheels blocked

    def place(self, x: float, y: float, theta: float) -> None:
        self.x, self.y, self.theta = x, y, theta
        self.w_l = self.w_r = 0.0
        self.v = 0.0
        self.omega = 0.0

    def _wheel(self, duty: float, w: float, gain: float, dt: float) -> tuple[float, float]:
        p = self.p
        volts = max(-1.0, min(1.0, duty)) * self.vbat
        if abs(volts) > p.static_volts:
            w_ss = p.mm_s_per_volt * gain * (volts - math.copysign(p.static_volts, volts))
        else:
            w_ss = 0.0
        w_new = w + (w_ss - w) * min(1.0, dt / p.motor_tau_s)
        # current: proportional to the "back-EMF deficit" (stall at w=0)
        stall_a = p.motor_stall_amps_at_6v * abs(volts) / 6.0
        free = abs(w_ss) if abs(w_ss) > 1 else 1.0
        i = stall_a * max(0.0, 1.0 - abs(w_new) / free) + 0.12 * abs(duty)
        return w_new, i

    def step(self, duty_l: float, duty_r: float, dt: float) -> None:
        p = self.p
        if p.motor_polarity_error:
            duty_l = -duty_l
        gl, gr = p.motor_gain_mismatch
        w_l_old, w_r_old = self.w_l, self.w_r
        self.w_l, il = self._wheel(duty_l, self.w_l, gl, dt)
        self.w_r, ir = self._wheel(duty_r, self.w_r, gr, dt)
        if self.jammed:
            self.w_l = self.w_r = 0.0
        self.current = il + ir + 0.15  # + logic, sensors, LED
        self.vbat = p.battery_full_v - p.battery_internal_ohm * self.current - 0.0004 * self.energy_mAh
        self.energy_mAh += self.current * dt / 3.6

        # encoders count wheel rotation
        cpm = p.counts_per_wheel_rev / (math.pi * p.wheel_diameter_mm)
        self.enc_l += self.w_l * dt * cpm
        self.enc_r += self.w_r * dt * cpm
        self.pulses_l += abs(self.w_l) * dt * cpm
        self.pulses_r += abs(self.w_r) * dt * cpm

        # ground motion: true diameter and slip
        el, er = p.wheel_diameter_error
        slip_l = slip_r = 0.0
        if p.slip_noise or p.accel_slip_gain:
            al = abs(self.w_l - w_l_old) / dt / 1000.0  # m/s^2
            ar = abs(self.w_r - w_r_old) / dt / 1000.0
            slip_l = abs(self.rng.normal(0, p.slip_noise)) + p.accel_slip_gain * al
            slip_r = abs(self.rng.normal(0, p.slip_noise)) + p.accel_slip_gain * ar
            slip_l = min(slip_l, 0.5)
            slip_r = min(slip_r, 0.5)
        vl = self.w_l * (1.0 + el) * (1.0 - slip_l)
        vr = self.w_r * (1.0 + er) * (1.0 - slip_r)
        self.v = 0.5 * (vl + vr)
        self.omega = (vr - vl) / p.track_width_mm
        th_mid = self.theta + 0.5 * self.omega * dt
        self.x += self.v * math.cos(th_mid) * dt
        self.y += self.v * math.sin(th_mid) * dt
        self.theta += self.omega * dt
        self.odometer += abs(self.v) * dt

    def encoder_counts(self) -> tuple[int, int]:
        """What the HAL reports: signed quadrature counts (mode 0), unsigned
        slot-disc pulses (mode 1) or nothing (mode 2)."""
        mode = self.p.encoder_mode
        if mode == 2:
            return 0, 0
        if mode == 1:
            return int(self.pulses_l), int(self.pulses_r)
        l = int(math.floor(self.enc_l))
        r = int(math.floor(self.enc_r))
        if self.p.encoder_polarity_error:
            l = -l
        return l, r

    def gyro_dps(self) -> tuple[float, bool]:
        """MPU6050 yaw rate (deg/s, CCW positive) with bias, noise and scale error."""
        p = self.p
        if not p.has_gyro:
            return 0.0, False
        g = math.degrees(self.omega) * (1.0 + p.gyro_scale_error) + p.gyro_bias_dps
        if p.gyro_noise_dps:
            g += self.rng.normal(0, p.gyro_noise_dps)
        return g, True
