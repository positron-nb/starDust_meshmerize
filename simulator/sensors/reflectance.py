"""Reflectance-array model (Pololu QTR-MD-08A style, analog, 12-bit ADC).

Each channel sees a small optical spot (radius ~4 mm at 3-5 mm height). The
white fraction under the spot is approximated from the signed distance to
each white rectangle with a linear ramp of width 2*spot radius - good enough
to reproduce partial readings at line edges, which is what the centroid
interpolation relies on. Per-channel gain/offset spread, ADC noise, dull
vinyl, shiny floor, a dead channel and sample delay are configurable.
QTR analog output is HIGH over black and LOW over white.
"""
from __future__ import annotations

import math

import numpy as np

from ..arena import Arena
from ..config import SimParams


class ReflectanceArray:
    def __init__(self, arena: Arena, p: SimParams, rng: np.random.Generator):
        self.p = p
        self.rng = rng
        rects = arena.rects()
        self.line_rects = rects[:-1]
        self.box_rect = rects[-1]
        n = p.num_sensors
        lat = list((np.arange(n) - (n - 1) / 2.0) * p.sensor_pitch_mm)  # + = right
        if p.num_wings == 2:
            lat += [-p.wing_lateral_mm, p.wing_lateral_mm]  # left wing, right wing (after the array)
        self.lateral = np.array(lat)
        n = len(lat)
        self.gain = 1.0 + rng.normal(0, p.sensor_gain_spread, n) if p.sensor_gain_spread else np.ones(n)
        self.offset = rng.normal(0, p.sensor_offset_spread, n) if p.sensor_offset_spread else np.zeros(n)
        # digital comparator modules: each has its own threshold (potentiometer setting)
        self.threshold = np.clip(0.5 + rng.normal(0, 0.08, n), 0.3, 0.7) if p.sensor_gain_spread else np.full(n, 0.5)
        self.state = np.zeros(n, dtype=bool)
        self._near = self.line_rects
        self._near_center = (1e9, 1e9)
        self.history: list[np.ndarray] = []

    def _cull(self, x: float, y: float) -> None:
        if abs(x - self._near_center[0]) + abs(y - self._near_center[1]) < 60:
            return
        R = 260.0
        r = self.line_rects
        m = (r[:, 0] < x + R) & (r[:, 2] > x - R) & (r[:, 1] < y + R) & (r[:, 3] > y - R)
        self._near = r[m]
        self._near_center = (x, y)

    @staticmethod
    def _coverage(px, py, rects, spot):
        if len(rects) == 0:
            return np.zeros_like(px)
        xmin = rects[:, 0][None, :]
        ymin = rects[:, 1][None, :]
        xmax = rects[:, 2][None, :]
        ymax = rects[:, 3][None, :]
        X = px[:, None]
        Y = py[:, None]
        dx = np.maximum(np.maximum(xmin - X, X - xmax), 0.0)
        dy = np.maximum(np.maximum(ymin - Y, Y - ymax), 0.0)
        outside = np.hypot(dx, dy)
        inside = np.minimum(np.minimum(X - xmin, xmax - X), np.minimum(Y - ymin, ymax - Y))
        d = np.where(outside > 0, outside, -inside)
        cov = np.clip(0.5 - d / (2.0 * spot), 0.0, 1.0)
        return cov.max(axis=1)

    def sensor_points(self, x: float, y: float, theta: float):
        c, s = math.cos(theta), math.sin(theta)
        fx = x + self.p.sensor_forward_mm * c
        fy = y + self.p.sensor_forward_mm * s
        # right-hand vector is (s, -c)
        px = fx + self.lateral * s
        py = fy - self.lateral * c
        return px, py

    def read(self, x: float, y: float, theta: float) -> np.ndarray:
        p = self.p
        px, py = self.sensor_points(x, y, theta)
        self._cull(x, y)
        cov_line = self._coverage(px, py, self._near, p.sensor_spot_radius_mm)
        cov_box = self._coverage(px, py, self.box_rect[None, :], p.sensor_spot_radius_mm)
        refl = np.maximum(cov_line * p.white_reflectance, cov_box * p.box_reflectance)
        cov = np.maximum(cov_line, cov_box)
        refl = refl + (1.0 - cov) * p.floor_reflectance
        if p.sensor_digital:
            # TCRT5000 + LM393 comparator: output LOW when it sees white, with a
            # little hysteresis; read by the ESP32 ADC as ~0 or ~4095.
            r = refl * self.gain + (self.rng.normal(0, p.sensor_noise_raw / 3300.0, refl.shape) if p.sensor_noise_raw else 0)
            self.state = np.where(self.state, r > self.threshold - 0.04, r > self.threshold + 0.04)
            raw = np.where(self.state, 40.0, 4060.0)
            if 0 <= p.dead_channel < len(raw):
                raw[p.dead_channel] = 4060.0
            self.history.append(raw)
            if len(self.history) > p.sensor_delay_steps + 1:
                self.history.pop(0)
            return self.history[0]
        raw = p.raw_black - (p.raw_black - p.raw_white) * refl
        raw = self.offset + self.gain * raw
        if p.sensor_noise_raw:
            raw = raw + self.rng.normal(0, p.sensor_noise_raw, raw.shape)
        if 0 <= p.dead_channel < len(raw):
            raw[p.dead_channel] = 4000.0
        raw = np.clip(raw, 0, 4095)
        # sample delay (ADC conversion + filtering latency)
        self.history.append(raw)
        if len(self.history) > p.sensor_delay_steps + 1:
            self.history.pop(0)
        return self.history[0]
