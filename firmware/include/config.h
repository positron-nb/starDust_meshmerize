// config.h - every tunable parameter of the robot, in one place.
//
// The parameter list is an X-macro so that the same table generates:
//   * the RobotConfig struct (with defaults) used by the firmware,
//   * a name->field lookup used by the simulator (mesh_set_param) and by the
//     tuning documentation generator.
//
// Units are in the parameter names: _mm, _mm_s (mm/s), _mm_s2 (mm/s^2),
// _deg, _deg_s, _ms, _v (volts). "norm" values are calibrated reflectance,
// 0 = darkest seen during calibration, 1000 = whitest (the vinyl line).
//
// MUST BE MEASURED ON THE REAL ROBOT (see docs/calibration_guide.md):
//   wheel_diameter_mm, track_width_mm, counts_per_wheel_rev, sensor_forward_mm,
//   *_invert flags, ff_mm_s_per_v, ff_static_v.
// TUNE ON THE PRACTICE ARENA: line_kp*, line_kd, speeds, goal_confirm_mm.
#pragma once
#include <stdint.h>

// X(type, name, default, description)
#define MESH_CONFIG_FIELDS(X)                                                                              \
  /* ---- reflectance sensors: 5-channel TCRT5000 array (15 mm pitch) + 2 single IR "wing" sensors ---- */ \
  X(int32_t, num_sensors, 5, "number of channels in the line ARRAY (wings come after them)")            \
  X(float, sensor_pitch_mm, 15.0f, "centre-to-centre spacing of the array sensors (MEASURE)")            \
  X(int32_t, num_wings, 2, "0 = no wing sensors; 2 = left + right single IR sensors after the array")    \
  X(float, wing_lateral_mm, 45.0f, "wing sensor distance from the robot centreline (MEASURE)")            \
  X(float, sensor_lateral_offset_mm, 0.0f, "array centre offset to the right of robot centreline")        \
  X(float, sensor_forward_mm, 100.0f, "distance from wheel axle to sensor line (L_s) (MEASURE)")          \
  X(int32_t, sensor_flip, 0, "1 if channel 0 is on the robot's RIGHT side")                               \
  X(int32_t, sensor_dark_high, 1, "polarity used before calibration: 1 = raw HIGH over black")           \
  X(int32_t, sensor_auto_polarity, 1, "1 = learn each channel's polarity during the calibration sweep")   \
  X(float, sensor_on_norm, 500.0f, "calibrated value above which a channel sees the white line")         \
  X(float, sensor_noise_norm, 120.0f, "values below this are ignored in the centroid")                   \
  X(float, cal_min_span_raw, 300.0f, "min (max-min) raw span for a channel to count as healthy")         \
  X(int32_t, branch_min_sensors, 4, "white run touching an array edge must be this wide to be a branch") \
  X(float, branch_side_margin_mm, 20.0f, "white this far beyond the tracked line centre = a side branch")\
  /* ---- drive train: BO 500 RPM gearmotors, 65 mm wheels, MPU6050 gyro ---- */                         \
  X(int32_t, encoder_mode, 2, "0 = quadrature encoders, 1 = slot-disc encoders, 2 = no encoders")        \
  X(int32_t, use_gyro, 1, "1 = take turn angle / heading from the MPU6050 gyro when it is present")      \
  X(float, gyro_sign, 1.0f, "+1 or -1 so that a left (CCW) turn reads positive (calibration checks it)")  \
  X(float, wheel_diameter_mm, 65.0f, "nominal wheel diameter (MEASURE)")                                 \
  X(float, track_width_mm, 135.0f, "wheel centre to wheel centre (MEASURE)")                             \
  X(float, counts_per_wheel_rev, 40.0f, "counts per WHEEL revolution (slot disc: 2 x slots; N20: 1400)") \
  X(float, motor_tau_ms, 60.0f, "motor speed time constant (no-encoder distance model)")                 \
  X(float, motor_trim_tau_s, 1.5f, "no encoders: time constant for learning the left/right motor balance") \
  X(float, wheel_scale_left, 1.0f, "left wheel diameter correction factor")                               \
  X(float, wheel_scale_right, 1.0f, "right wheel diameter correction factor")                             \
  X(int32_t, left_motor_invert, 0, "1 to reverse left motor polarity in software")                       \
  X(int32_t, right_motor_invert, 1, "1 to reverse right motor polarity (mirrored mounting)")             \
  X(int32_t, left_enc_invert, 0, "1 to reverse left encoder count direction")                            \
  X(int32_t, right_enc_invert, 1, "1 to reverse right encoder count direction")                          \
  X(float, motor_max_avg_v, 6.0f, "duty is clamped so average motor voltage never exceeds this")         \
  X(float, vbat_nominal_v, 16.2f, "pack voltage assumed when not measured: CHARGE FULLY before each run")  \
  X(float, ff_mm_s_per_v, 300.0f, "steady wheel speed per volt above static (feed-forward) (CALIBRATE)") \
  X(float, ff_static_v, 0.6f, "volts needed to overcome static friction")                                \
  X(float, wheel_kp_v_per_mm_s, 0.004f, "wheel speed PI proportional gain")                              \
  X(float, wheel_ki_v_per_mm, 0.06f, "wheel speed PI integral gain")                                     \
  /* ---- line following ---- */                                                                         \
  X(float, line_kp, 0.09f, "steering rad/s per mm of line error at 0 m/s")                              \
  X(float, line_kp_per_m_s, 0.15f, "extra steering gain per m/s of forward speed")                       \
  X(float, line_kd, 0.004f, "steering rad/s per (mm/s) of line error rate")                             \
  X(float, max_steer_rad_s, 9.0f, "steering command limit")                                              \
  /* ---- dry-run (exploration) motion ---- */                                                           \
  X(float, explore_speed_mm_s, 450.0f, "cruise speed on unexplored lines")                               \
  X(float, explore_known_speed_mm_s, 700.0f, "cruise speed on already-mapped edges during dry run")      \
  X(float, explore_accel_mm_s2, 2500.0f, "acceleration limit in dry run")                                \
  X(float, explore_decel_mm_s2, 3000.0f, "braking limit in dry run")                                     \
  X(float, approach_speed_mm_s, 250.0f, "speed when an expected junction is near")                       \
  /* ---- actual-run motion ---- */                                                                      \
  X(float, actual_speed_mm_s, 1000.0f, "cruise speed on straight segments of the stored route")          \
  X(float, actual_accel_mm_s2, 3500.0f, "acceleration limit in actual run")                              \
  X(float, actual_decel_mm_s2, 3500.0f, "braking limit in actual run")                                   \
  X(float, actual_approach_mm_s, 320.0f, "speed when reaching an expected turn point")                    \
  X(float, actual_margin_mm, 90.0f, "reach approach speed this far before an expected turn point")       \
  X(float, actual_pass_speed_mm_s, 700.0f, "max speed through a junction the route goes straight over")  \
  /* ---- turns ---- */                                                                                  \
  X(float, turn_omega_deg_s, 540.0f, "peak pivot rate, dry run")                                         \
  X(float, actual_turn_omega_deg_s, 720.0f, "peak pivot rate, actual run")                               \
  X(float, turn_alpha_deg_s2, 7000.0f, "pivot angular acceleration")                                     \
  X(float, turn_capture_start_deg, 55.0f, "start looking for the new line after this much rotation")    \
  X(float, uturn_capture_start_deg, 150.0f, "same, for 180 degree turns")                               \
  X(float, turn_capture_tol_mm, 7.0f, "turn ends when the line is within this of the centre")            \
  X(float, turn_max_overshoot_deg, 40.0f, "turn fails if no line this far past the nominal angle")      \
  /* ---- junction / dead-end / goal classification ---- */                                              \
  X(float, crossbar_clear_mm, 4.0f, "branch pattern must be absent this long to end a crossing")        \
  X(float, lookahead_mm, 16.0f, "travel after a crossing used to test for a straight exit")              \
  X(float, deadend_confirm_mm, 14.0f, "line must be absent this long to call a dead end")                \
  X(float, line_lost_mm, 80.0f, "unexpected line absence longer than this is a LINE_LOST fault")         \
  X(float, goal_confirm_mm, 70.0f, "continuous all-white travel needed to accept the end box")           \
  X(float, goal_stop_extra_mm, 180.0f, "drive this far after goal confirmation, then stop")              \
  X(float, goal_abort_loss_mm, 20.0f, "white lost this long during the final drive = false goal")       \
  X(float, min_event_spacing_mm, 30.0f, "ignore a new junction pattern this soon after the last")        \
  /* ---- mapping ---- */                                                                                \
  X(float, node_match_base_mm, 70.0f, "radius for matching a junction to a known node")                  \
  X(float, node_match_drift_frac, 0.03f, "extra radius per mm travelled since last loop closure")        \
  X(float, node_match_max_mm, 150.0f, "upper bound of the matching radius (keep < half the junction spacing)")                              \
  /* ---- planning / exploration ---- */                                                                 \
  X(float, turn_tiebreak_mm, 30.0f, "route cost added per turn: picks fewer turns among equal routes")   \
  X(int32_t, explore_policy, 0, "0=DFS nearest frontier, 1=left hand, 2=right hand")                     \
  X(int32_t, explore_pref, 2, "branch order 0=LSR 1=LRS 2=SLR 3=SRL 4=RLS 5=RSL")                         \
  X(float, explore_start_weight, 2.0f, "frontier cost += this x distance from START (0 = pure DFS)")      \
  X(int32_t, defer_goal, 0, "1 = peek at end box, finish proving shortest route (NEEDS ORGANIZER OK)")    \
  /* ---- safety ---- */                                                                                 \
  X(float, stall_ms, 450.0f, "commanded motion with no encoder motion for this long = stall")            \
  X(float, stall_no_event_mm, 3200.0f, "no encoders: modelled travel with no track event = stuck (arena 2.3 m)") \
  X(float, low_batt_v, 13.2f, "below this (4S, 3.3 V/cell) the robot refuses to start (needs divider)")  \
  X(float, recover_sweep_deg, 70.0f, "line-loss recovery: max pivot each way while searching")           \
  X(int32_t, max_recoveries, 3, "automatic recoveries per run before declaring a fault")                 \
  /* ---- operator interface ---- */                                                                     \
  X(float, arm_delay_ms, 700.0f, "delay between button release and motion")                              \
  X(float, long_press_ms, 1500.0f, "hold this long (when idle) to start sensor calibration")             \
  X(float, clear_hold_ms, 4000.0f, "hold this long (when idle) to erase the stored map")                 \
  X(float, calib_sweep_deg, 50.0f, "calibration: pivot this far each side of the line")

namespace mesh {

struct RobotConfig {
#define MESH_CFG_DECLARE(type, name, def, doc) type name = def;
  MESH_CONFIG_FIELDS(MESH_CFG_DECLARE)
#undef MESH_CFG_DECLARE
};

// Firmware build identification (stored with the map so a reflash invalidates it).
constexpr uint32_t kFirmwareVersion = 0x00010100;  // 1.1.0

// Fixed control timing (justified in docs/system_architecture.md).
constexpr uint32_t kControlPeriodUs = 1000;  // 1 kHz sensor + control loop

// Capacity limits for the static map (no heap on the robot).
constexpr int kMaxNodes = 96;
constexpr int kMaxEdges = 160;
constexpr int kMaxEdgeSegs = 12;
constexpr int kMaxRouteSteps = 96;

}  // namespace mesh
