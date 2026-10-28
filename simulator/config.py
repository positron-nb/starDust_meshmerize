"""Simulation parameters (the PHYSICAL world and its imperfections).

Robot *firmware* parameters live in firmware/include/config.h and are pushed
into the core with core.set_param(); only the geometry entries below must
agree with them (wheel diameter, track width, counts per revolution, sensor
layout) - `SimParams.firmware_overrides()` keeps them in sync.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace


@dataclass
class SimParams:
    # --- timing
    dt: float = 0.001                  # physics + control step (1 kHz, same as firmware)
    sensor_delay_steps: int = 1        # ADC sample age seen by the controller

    # --- robot geometry (TRUE values). Defaults = the PURCHASED hardware:
    #     5-channel TCRT5000 array + 2 wing IR sensors, BO 500 RPM motors with
    #     65 mm wheels on a 2-wheel chassis, MPU6050, no wheel encoders.
    #     `n20_hardware()` gives the original N20 + QTR-MD-08A design.
    wheel_diameter_mm: float = 65.0
    track_width_mm: float = 135.0
    counts_per_wheel_rev: float = 40.0   # slot disc: 20 slots, both edges
    sensor_forward_mm: float = 100.0
    sensor_pitch_mm: float = 15.0
    num_sensors: int = 5
    num_wings: int = 2
    wing_lateral_mm: float = 45.0
    sensor_digital: bool = True          # comparator modules: output is 0/1 only
    sensor_spot_radius_mm: float = 7.0   # optical footprint at ~10 mm height (TCRT5000)
    encoder_mode: int = 2                # 0 quadrature, 1 slot disc, 2 none
    has_gyro: bool = True
    body_front_mm: float = 115.0         # axle -> front edge (for box-fit check)
    body_rear_mm: float = 95.0

    # --- gyro (MPU6050) model
    gyro_bias_dps: float = 0.0
    gyro_noise_dps: float = 0.0
    gyro_scale_error: float = 0.0

    # --- imperfections (0 = ideal)
    wheel_diameter_error: tuple[float, float] = (0.0, 0.0)  # fractional, left/right (true vs nominal)
    motor_gain_mismatch: tuple[float, float] = (1.0, 1.0)   # torque/speed constant multipliers
    slip_noise: float = 0.0             # random fractional slip std per step
    accel_slip_gain: float = 0.0        # extra slip per m/s^2 of wheel acceleration
    sensor_noise_raw: float = 0.0       # ADC noise std (counts)
    sensor_gain_spread: float = 0.0     # per-channel gain variation (fraction)
    sensor_offset_spread: float = 0.0   # per-channel offset variation (counts)
    white_reflectance: float = 1.0      # 1 = calibrated vinyl; <1 = dirtier/darker line
    floor_reflectance: float = 0.0      # 0 = calibrated black; >0 = shinier floor
    box_reflectance: float = 1.0        # end-box surface relative to vinyl
    dead_channel: int = -1              # index of a disconnected channel (reads black)
    encoder_polarity_error: bool = False  # left encoder wired backwards
    motor_polarity_error: bool = False    # left motor wired backwards

    # --- motor / battery model (BO 500 RPM @ 6 V, 65 mm wheel ~ 1.7 m/s no-load)
    motor_tau_s: float = 0.060          # mechanical time constant
    mm_s_per_volt: float = 300.0        # true no-load wheel speed per volt above static
    static_volts: float = 0.60
    battery_full_v: float = 16.6         # 4S 18650 pack (16.8 V off the charger)
    battery_internal_ohm: float = 0.30
    has_vbat_sense: bool = False         # no resistor divider fitted: firmware assumes vbat_nominal_v
    motor_stall_amps_at_6v: float = 1.0  # estimate for BO motors - MEASURE

    # --- QTR analog raw levels (12-bit ADC, QTR output high over black)
    raw_black: float = 3600.0
    raw_white: float = 300.0

    seed: int = 0
    firmware: dict = field(default_factory=dict)  # extra firmware param overrides

    def firmware_overrides(self) -> dict:
        """Firmware parameters that must match the simulated hardware. Note:
        the firmware is told the NOMINAL geometry; the imperfections above
        are not revealed to it (as on the real robot)."""
        d = {
            "num_sensors": self.num_sensors,
            "sensor_pitch_mm": self.sensor_pitch_mm,
            "num_wings": self.num_wings,
            "wing_lateral_mm": self.wing_lateral_mm,
            "sensor_forward_mm": self.sensor_forward_mm,
            "wheel_diameter_mm": self.wheel_diameter_mm,
            "track_width_mm": self.track_width_mm,
            "counts_per_wheel_rev": self.counts_per_wheel_rev,
            "encoder_mode": self.encoder_mode,
            "use_gyro": 1 if self.has_gyro else 0,
        }
        d.update(self.firmware)
        return d

    def with_(self, **kw) -> "SimParams":
        return replace(self, **kw)


def n20_hardware(**kw) -> dict:
    """Field overrides for the ORIGINAL design (N20 + magnetic quadrature
    encoders, 34 mm wheels, 8-channel analog QTR-MD-08A, no gyro). Use as
    SimParams(**n20_hardware()) or realistic(seed, **n20_hardware())."""
    d = dict(
        wheel_diameter_mm=34.0, track_width_mm=96.0, counts_per_wheel_rev=1400.0, sensor_forward_mm=70.0,
        sensor_pitch_mm=8.0, num_sensors=8, num_wings=0, sensor_digital=False, sensor_spot_radius_mm=4.0,
        encoder_mode=0, has_gyro=False, body_front_mm=85.0, body_rear_mm=55.0, motor_tau_s=0.035,
        mm_s_per_volt=175.0, static_volts=0.30, motor_stall_amps_at_6v=1.6,
        battery_full_v=8.2, battery_internal_ohm=0.25, has_vbat_sense=True,
        firmware={"vbat_nominal_v": 7.6, "low_batt_v": 6.4, "branch_min_sensors": 5, "branch_side_margin_mm": 0.0, "ff_mm_s_per_v": 170.0, "ff_static_v": 0.35, "motor_tau_ms": 35.0,
                  "goal_stop_extra_mm": 140.0},
    )
    fw = kw.pop("firmware", None)
    d.update(kw)
    if fw:
        d["firmware"] = {**d["firmware"], **fw}
    return d


def realistic(seed: int = 0, **kw) -> SimParams:
    """Imperfections representative of a carefully built student robot."""
    p = SimParams(
        seed=seed,
        wheel_diameter_error=(0.01, -0.008),
        motor_gain_mismatch=(1.04, 0.96),
        slip_noise=0.002,
        accel_slip_gain=0.004,
        sensor_noise_raw=30.0,
        sensor_gain_spread=0.08,
        sensor_offset_spread=60.0,
        white_reflectance=0.92,
        floor_reflectance=0.04,
        box_reflectance=0.95,
        sensor_delay_steps=1,
        gyro_bias_dps=1.5,       # residual after the firmware's standstill bias estimate
        gyro_noise_dps=0.1,
        gyro_scale_error=0.01,
        mm_s_per_volt=309.0,     # 3 % speed-model calibration error (no-encoder mode)
    )
    return replace(p, **kw)


def harsh(seed: int = 0, **kw) -> SimParams:
    """Stress case: worn wheels, noisy sensors, dull vinyl, more slip."""
    p = SimParams(
        seed=seed,
        wheel_diameter_error=(0.025, -0.02),
        motor_gain_mismatch=(1.10, 0.90),
        slip_noise=0.005,
        accel_slip_gain=0.01,
        sensor_noise_raw=80.0,
        sensor_gain_spread=0.15,
        sensor_offset_spread=120.0,
        white_reflectance=0.80,
        floor_reflectance=0.10,
        box_reflectance=0.85,
        sensor_delay_steps=2,
        gyro_bias_dps=3.0,
        gyro_noise_dps=0.3,
        gyro_scale_error=0.03,
        mm_s_per_volt=330.0,     # 10 % speed-model calibration error
    )
    return replace(p, **kw)
