#include "robot.h"

#include <math.h>
#include <string.h>

#include "../util/util.h"

namespace mesh {

static const char* stateName(RobotState s) {
  switch (s) {
    case RobotState::BOOT: return "BOOT";
    case RobotState::IDLE: return "IDLE";
    case RobotState::CALIBRATING: return "CALIBRATING";
    case RobotState::ARMED: return "ARMED";
    case RobotState::DRY_RUN: return "DRY_RUN";
    case RobotState::GOAL_CONFIRMATION: return "GOAL_CONFIRMATION";
    case RobotState::MAP_VALIDATION: return "MAP_VALIDATION";
    case RobotState::ROUTE_PLANNING: return "ROUTE_PLANNING";
    case RobotState::WAIT_FOR_ACTUAL_RUN: return "WAIT_FOR_ACTUAL_RUN";
    case RobotState::ACTUAL_RUN: return "ACTUAL_RUN";
    case RobotState::FINISHED: return "FINISHED";
    case RobotState::RECOVERY: return "RECOVERY";
    case RobotState::FAULT: return "FAULT";
    case RobotState::CLEAR_MEMORY: return "CLEAR_MEMORY";
  }
  return "?";
}

static const char* faultName(FaultCode f) {
  static const char* n[] = {"NONE", "LINE_LOST", "STALL", "LOW_BATTERY", "ROUTE_MISMATCH", "MAP_FULL",
                            "NO_FRONTIER", "TURN_FAILED", "CALIBRATION_BAD", "STORAGE", "TIMEOUT"};
  int i = (int)f;
  return i < (int)(sizeof(n) / sizeof(n[0])) ? n[i] : "?";
}

// ------------------------------------------------------------------- setup --
void Robot::init() {
  sensors_.init(&cfg);
  odo_.init(&cfg);
  motion_.init(&cfg, &odo_);
  detector_.init(&cfg);
  goal_.init(&cfg);
  explorer_.init(&cfg);
  faults_.init(&cfg);
  graph_.clear();
  old_graph_.clear();
  route_ = Route();
  state_ = RobotState::BOOT;
  stage_ = Stage::FRESH;
  run_ = RunKind::NONE;
  first_step_ = true;
}

void Robot::loadPersisted(const uint8_t* buf, size_t len) {
  PersistState st;
  st.graph = &graph_;
  st.route = &route_;
  if (buf && len && persistDeserialize(st, buf, len)) {
    stage_ = st.stage;
    sensors_.setCalibration(st.cal);
    restarts_ = st.restarts_used;
    Log::printf("STORE loaded: stage=%d nodes=%d edges=%d route=%d cal=%d", (int)stage_, graph_.nodeCount(),
                graph_.edgeCount(), route_.valid ? route_.n : 0, st.cal.valid);
  } else {
    graph_.clear();
    route_ = Route();
    stage_ = Stage::FRESH;
    Log::printf("STORE no valid record - starting FRESH");
  }
}

size_t Robot::serialize(uint8_t* buf, size_t cap) const {
  PersistState st;
  st.stage = stage_;
  st.cal = sensors_.calibration();
  // While relocalising, the pre-restart map is the trustworthy one.
  st.graph = const_cast<MazeGraph*>(relocalizing_ ? &old_graph_ : &graph_);
  st.route = const_cast<Route*>(&route_);
  st.restarts_used = (uint8_t)restarts_;
  return persistSerialize(st, buf, cap);
}

void Robot::enter(RobotState s) {
  if (s != state_) Log::printf("STATE %s -> %s", stateName(state_), stateName(s));
  state_ = s;
  state_ms_ = 0;
}

void Robot::fail(FaultCode code) {
  fault_ = code;
  auto_start_ = false;
  Log::printf("FAULT %s (run=%d nav=%d)", faultName(code), (int)run_, (int)nav_);
  motion_.idle();
  nav_ = Nav::NONE;
  if (run_ == RunKind::DRY) stage_ = Stage::DRY_PARTIAL;
  requestSave();
  enter(RobotState::FAULT);
}

float Robot::poseX() const {
  if (nav_ == Nav::FOLLOW || nav_ == Nav::GOAL_DRIVE) return seg_x_ + dirDx(heading_) * (odo_.s() - seg_start_s_);
  return seg_x_;
}
float Robot::poseY() const {
  if (nav_ == Nav::FOLLOW || nav_ == Nav::GOAL_DRIVE) return seg_y_ + dirDy(heading_) * (odo_.s() - seg_start_s_);
  return seg_y_;
}

// -------------------------------------------------------------------- step --
void Robot::step(const HalInputs& in, HalOutputs& out) {
  float dt = 0.001f;
  if (!first_step_) dt = (float)(uint32_t)(in.t_us - last_us_) * 1e-6f;
  first_step_ = false;
  last_us_ = in.t_us;
  dt = clampf(dt, 0.0f, 0.02f);
  Log::setTimeMs(in.t_us / 1000u);
  vbat_ = in.vbat;
  raw_ = in.raw;

  odo_.update(in.enc_left, in.enc_right, dt, in.gyro_z_dps, in.gyro_ok != 0, motion_.refLeft(), motion_.refRight());
  sensors_.process(in.raw, feat_);
  state_ms_ += dt * 1000.0f;

  out.emitters_on = 1;
  out.motors_enable = 1;
  out.save_request = 0;

  handleButton(in, dt);

  switch (state_) {
    case RobotState::BOOT:
      enter(RobotState::IDLE);
      break;
    case RobotState::IDLE:
    case RobotState::WAIT_FOR_ACTUAL_RUN:
    case RobotState::FINISHED:
    case RobotState::FAULT:
      if (motion_.mode() != MotionMode::IDLE) motion_.idle();
      break;
    case RobotState::CALIBRATING:
      calibrateStep(dt);
      break;
    case RobotState::ARMED:
      if (state_ms_ >= cfg.arm_delay_ms) startRun(armed_kind_, armed_restart_);
      break;
    case RobotState::DRY_RUN:
    case RobotState::ACTUAL_RUN:
    case RobotState::GOAL_CONFIRMATION:
      runStep(dt);
      break;
    case RobotState::RECOVERY:
      run_ms_ += dt * 1000.0f;
      recoveryStep(dt);
      break;
    case RobotState::MAP_VALIDATION: {
      if (relocalizing_) {
        Log::printf("MAP never re-joined the pre-restart map: START->END route unknown");
      }
      int problems = graph_.validate();
      Log::printf("MAP validation: %d nodes, %d edges, %d problems, frontier=%d", graph_.nodeCount(),
                  graph_.edgeCount(), problems, graph_.hasFrontier() ? 1 : 0);
      enter(RobotState::ROUTE_PLANNING);
      break;
    }
    case RobotState::ROUTE_PLANNING: {
      route_ = Route();
      if (relocalizing_) {
        // Keep the START-based map: the local map cannot be used from START.
        graph_ = old_graph_;
      }
      if (!relocalizing_ && graph_.goalNode() >= 0 &&
          Planner::shortest(graph_, 0, DIR_N, graph_.goalNode(), cfg.turn_tiebreak_mm, route_)) {
        lower_bound_ = Planner::unexploredLowerBound(graph_, 0, DIR_N, graph_.goalNode());
        Log::printf("ROUTE %d steps, %.0f mm, %d turns; unexplored lower bound %.0f mm -> %s", route_.n,
                    route_.length, route_.turns, minf(lower_bound_, 999999.0f),
                    lower_bound_ >= route_.length - 20.0f ? "PROVABLY SHORTEST" : "may not be shortest");
        char buf[100];
        int k = 0;
        uint8_t h = DIR_N;
        for (int i = 0; i < route_.n && k < 96; ++i) {
          buf[k++] = relTurnChar(relTurn(h, route_.steps[i].exit));
          h = graph_.arrivalHeading(route_.steps[i].edge, route_.steps[i].node);
          const Edge& e = graph_.edge(route_.steps[i].edge);
          if (e.a == e.b) h = dirOpp(route_.steps[i].exit == e.da ? e.db : e.da);
        }
        buf[k] = 0;
        Log::printf("ROUTE actions at nodes: %s", buf);
      } else {
        Log::printf("ROUTE planning failed - actual run will search");
      }
      relocalizing_ = false;
      stage_ = Stage::DRY_DONE;
      requestSave();
      enter(RobotState::WAIT_FOR_ACTUAL_RUN);
      break;
    }
    case RobotState::CLEAR_MEMORY:
      graph_.clear();
      route_ = Route();
      stage_ = Stage::FRESH;
      restarts_ = 0;
      relocalizing_ = false;
      led_red_ = false;
      Log::printf("MEMORY cleared (calibration kept)");
      requestSave();
      enter(RobotState::IDLE);
      break;
  }

  motion_.update(feat_, dt, in.vbat > 3.0f ? in.vbat : cfg.vbat_nominal_v, out);

  // Stall supervision while driving.
  if (state_ == RobotState::DRY_RUN || state_ == RobotState::ACTUAL_RUN || state_ == RobotState::RECOVERY ||
      state_ == RobotState::GOAL_CONFIRMATION) {
    float meas = absf(odo_.vLeft()) + absf(odo_.vRight());
    float cmd = motion_.mode() == MotionMode::IDLE || motion_.done() ? 0.0f : motion_.commandedSpeed();
    FaultCode f = faults_.update(cmd, meas, in.vbat, dt);
    // Without wheel encoders a jammed robot looks like a moving one. Every
    // line in the 2.3 m arena ends in a junction, corner, dead end or the box
    // within a few metres, so a long stretch with no track event means stuck.
    if (f == FaultCode::NONE && cfg.encoder_mode == 2 && nav_ == Nav::FOLLOW &&
        odo_.s() - feature_s_ > cfg.stall_no_event_mm) {
      Log::printf("STALL: %.0f mm modelled travel without a junction or corner", odo_.s() - feature_s_);
      f = FaultCode::STALL;
    }
    if (f != FaultCode::NONE) {
      fail(f);
      out.duty_left = out.duty_right = 0;
    }
  }
  if (nav_ != Nav::FOLLOW) feature_s_ = odo_.s();

  // Persist only while standing still (flash writes stall the CPU).
  if (save_pending_ && (motion_.mode() == MotionMode::IDLE) && absf(odo_.v()) < 30.0f) {
    out.save_request = 1;
    save_pending_ = false;
  }

  // LEDs
  out.led_red = led_red_ ? 1 : 0;
  uint32_t ms = in.t_us / 1000u;
  bool st = false;
  switch (state_) {
    case RobotState::IDLE:
      if (btn_ms_ >= cfg.clear_hold_ms) st = (ms / 50) & 1;          // very fast: release to clear
      else if (btn_ms_ >= cfg.long_press_ms) st = true;              // solid: release to calibrate
      else if (stage_ == Stage::FRESH) st = (ms % 1000) < 500;       // slow blink
      else if (stage_ == Stage::DRY_PARTIAL) st = (ms % 1000) < 120 || ((ms % 1000) > 240 && (ms % 1000) < 360);
      else st = (ms % 2000) < 1800;                                  // nearly solid: route stored
      break;
    case RobotState::WAIT_FOR_ACTUAL_RUN:
    case RobotState::FINISHED:
      st = true;
      break;
    case RobotState::FAULT:
      st = (ms / 100) & 1;
      break;
    case RobotState::CALIBRATING:
      st = (ms / 250) & 1;
      break;
    default:
      st = false;
  }
  out.led_status = st ? 1 : 0;
}

// ------------------------------------------------------------------ button --
void Robot::handleButton(const HalInputs& in, float dt) {
  bool b = in.button != 0;
  if (b) btn_ms_ += dt * 1000.0f;
  if (!b && btn_prev_) {
    float held = btn_ms_;
    btn_ms_ = 0;
    bool idle_like = state_ == RobotState::IDLE || state_ == RobotState::WAIT_FOR_ACTUAL_RUN ||
                     state_ == RobotState::FINISHED || state_ == RobotState::FAULT;
    bool running = state_ == RobotState::DRY_RUN || state_ == RobotState::ACTUAL_RUN ||
                   state_ == RobotState::RECOVERY || state_ == RobotState::GOAL_CONFIRMATION ||
                   state_ == RobotState::ARMED || state_ == RobotState::CALIBRATING;
    if (running) {
      Log::printf("BUTTON during motion: operator stop");
      if (state_ == RobotState::ARMED) {
        enter(RobotState::IDLE);
      } else {
        fail(FaultCode::NONE);
      }
    } else if (idle_like && held >= 30.0f) {
      if (held >= cfg.clear_hold_ms) {
        enter(RobotState::CLEAR_MEMORY);
      } else if (held >= cfg.long_press_ms) {
        cal_phase_ = 0;
        cal_checked_ = false;
        sensors_.beginCalibration();
        enter(RobotState::CALIBRATING);
      } else {
        RunKind k = RunKind::DRY;
        bool restart = false;
        if (state_ == RobotState::FAULT && run_ != RunKind::NONE) {
          k = run_ == RunKind::DRY ? RunKind::DRY : RunKind::ACTUAL;
          restart = true;
        } else {
          switch (stage_) {
            case Stage::FRESH: k = RunKind::DRY; break;
            case Stage::DRY_PARTIAL: k = RunKind::DRY; restart = true; break;
            case Stage::DRY_DONE:
            case Stage::ACTUAL_DONE: k = RunKind::ACTUAL; break;
          }
          if (state_ == RobotState::FAULT && run_ == RunKind::NONE && stage_ == Stage::FRESH) restart = false;
        }
        if (!faults_.batteryOkToStart(vbat_)) {
          Log::printf("START refused: battery %.2f V", vbat_);
          fail(FaultCode::LOW_BATTERY);
        } else if (!sensors_.calibrated()) {
          // No stored calibration (first use or lost flash): calibrate on the
          // spot (the robot sits on the START line), then start automatically.
          Log::printf("START: no calibration stored - auto-calibrating first");
          armed_kind_ = k;
          armed_restart_ = restart;
          auto_start_ = true;
          cal_phase_ = 0;
          cal_checked_ = false;
          sensors_.beginCalibration();
          enter(RobotState::CALIBRATING);
        } else {
          armed_kind_ = k;
          armed_restart_ = restart;
          enter(RobotState::ARMED);
        }
      }
    }
  }
  btn_prev_ = b;
}

// ------------------------------------------------------------- calibration --
void Robot::calibrateStep(float dt) {
  (void)dt;
  sensors_.sampleCalibration(raw_);
  float sweep = cfg.calib_sweep_deg * kDeg;
  if (cal_phase_ == 0) {
    motion_.pivot(sweep, 3.0f, 20.0f, -1, 0);
    cal_l0_ = odo_.leftMm();
    cal_r0_ = odo_.rightMm();
    cal_g0_ = odo_.gyroTheta();
    cal_phase_ = 1;
  } else if (cal_phase_ == 1 && state_ms_ > 300.0f && !cal_checked_) {
    // Wiring check: a counter-clockwise pivot must move the left wheel
    // backwards and the right wheel forwards (and the gyro must see a left
    // turn). A reversed motor, encoder or gyro shows up here, before the
    // robot ever drives fast.
    cal_checked_ = true;
    float dl = odo_.leftMm() - cal_l0_, dr = odo_.rightMm() - cal_r0_;
    if (cfg.encoder_mode == 0) {
      if (absf(dl) < 2.0f || absf(dr) < 2.0f) {
        Log::printf("CAL wheel not moving or encoder not counting (dL=%.1f dR=%.1f mm)", dl, dr);
        fail(FaultCode::CALIBRATION_BAD);
        return;
      }
      if (dl > 0 || dr < 0) {
        Log::printf("CAL polarity error: commanded L- R+, measured dL=%.1f dR=%.1f mm -> check *_invert flags", dl, dr);
        fail(FaultCode::CALIBRATION_BAD);
        return;
      }
    } else if (cfg.encoder_mode == 1 && (absf(dl) < 2.0f || absf(dr) < 2.0f)) {
      Log::printf("CAL slot encoder not counting (dL=%.1f dR=%.1f mm) -> check the disc and sensor wiring", dl, dr);
      fail(FaultCode::CALIBRATION_BAD);
      return;
    }
    if (odo_.gyroActive()) {
      float dg = (odo_.gyroTheta() - cal_g0_) / kDeg;
      if (dg < -2.0f) {
        Log::printf("CAL robot turned RIGHT (%.1f deg) when told to turn left -> swap *_motor_invert or gyro_sign", dg);
        fail(FaultCode::CALIBRATION_BAD);
        return;
      }
      if (dg < 2.0f) {
        Log::printf("CAL robot not rotating (%.1f deg): a motor is reversed or not running", dg);
        fail(FaultCode::CALIBRATION_BAD);
        return;
      }
    }
  } else if (state_ms_ > 10000.0f) {
    Log::printf("CAL timeout");
    fail(FaultCode::CALIBRATION_BAD);
    return;
  } else if (motion_.done()) {
    if (cal_phase_ == 1) {
      motion_.pivot(-2.0f * sweep, 3.0f, 20.0f, -1, 0);
      cal_phase_ = 2;
    } else if (cal_phase_ == 2) {
      motion_.pivot(sweep, 3.0f, 20.0f, -1, 0);
      cal_phase_ = 3;
    } else {
      motion_.idle();
      if (sensors_.finishCalibration()) {
        Log::printf("CAL ok");
        fault_ = FaultCode::NONE;
        if (auto_start_) {
          auto_start_ = false;
          enter(RobotState::ARMED);
        } else {
          requestSave();
          enter(RobotState::IDLE);
        }
      } else {
        fail(FaultCode::CALIBRATION_BAD);
      }
    }
  }
}

// --------------------------------------------------------------- run start --
void Robot::startRun(RunKind kind, bool restart) {
  faults_.reset();
  fault_ = FaultCode::NONE;
  recoveries_ = 0;
  led_red_ = false;
  goal_peeked_ = false;
  heading_ = DIR_N;
  seg_x_ = seg_y_ = 0;
  seg_start_s_ = odo_.s();
  dist_since_anchor_ = 0;
  detector_.rearm(odo_.s());
  explorer_.reset();
  sensors_.resetTracking(0);
  last_event_s_ = -1e9f;
  if (restart) {
    ++restarts_;
    Log::printf("RESTART #%d (%s)", restarts_, kind == RunKind::DRY ? "dry run, from checkpoint" : "actual run, from START");
  }
  if (!restart) run_ms_ = 0;

  if (kind == RunKind::DRY) {
    route_ = Route();
    map_conflicts_ = 0;
    if (restart && graph_.nodeCount() > 1) {
      // Keep what we learned; the robot was placed at a checkpoint whose
      // position in the old map is unknown -> map locally, then re-join.
      if (!relocalizing_) old_graph_ = graph_;
      graph_.clear();
      graph_.addNode(0, 0, dirBit(DIR_N), NodeType::START);  // virtual restart node
      relocalizing_ = true;
      Log::printf("RELOC old map kept (%d nodes), mapping from restart point", old_graph_.nodeCount());
    } else {
      graph_.clear();
      graph_.addNode(0, 0, dirBit(DIR_N), NodeType::START);
      relocalizing_ = false;
      markVerified();
    }
    stage_ = Stage::DRY_PARTIAL;
    run_ = RunKind::DRY;
    enter(RobotState::DRY_RUN);
    departNode(0, DIR_N);
    return;
  }

  // Actual run: validate the stored route; replan if needed.
  bool ok = route_.valid && route_.n > 0 && graph_.goalNode() >= 0 && route_.steps[0].node == 0;
  for (int i = 0; ok && i < route_.n; ++i)
    ok = route_.steps[i].edge >= 0 && route_.steps[i].edge < graph_.edgeCount() &&
         route_.steps[i].node < graph_.nodeCount();
  if (!ok && graph_.goalNode() >= 0) {
    Log::printf("ROUTE stored route invalid - replanning from map");
    ok = Planner::shortest(graph_, 0, DIR_N, graph_.goalNode(), cfg.turn_tiebreak_mm, route_) && route_.n > 0;
  }
  if (!ok) {
    Log::printf("ROUTE unavailable - ACTUAL run falls back to exploration (no shortest-path bonus)");
    graph_.clear();
    graph_.addNode(0, 0, dirBit(DIR_N), NodeType::START);
    run_ = RunKind::ACTUAL_EXPLORE;
    enter(RobotState::ACTUAL_RUN);
    departNode(0, DIR_N);
    return;
  }
  run_ = RunKind::ACTUAL;
  route_idx_ = 0;
  heading_ = route_.start_heading;
  enter(RobotState::ACTUAL_RUN);
  departNode(0, route_.steps[0].exit);
}

// --------------------------------------------------------------- navigation --
float Robot::cruiseSpeed() const {
  if (run_ == RunKind::ACTUAL) return cfg.actual_speed_mm_s;
  return exp_.known ? cfg.explore_known_speed_mm_s : cfg.explore_speed_mm_s;
}

void Robot::departNode(int node, uint8_t exit_dir) {
  cur_node_ = node;
  pending_exit_ = exit_dir;
  next_known_edge_ = graph_.node(node).edge[exit_dir];
  seg_x_ = graph_.node(node).x;
  seg_y_ = graph_.node(node).y;
  int rel = relTurn(heading_, exit_dir);
  if (rel == REL_STRAIGHT) {
    beginEdge(node, exit_dir, next_known_edge_);
  } else {
    pending_corner_ = false;
    turn_is_deadend_ = false;
    pending_s_center_ = odo_.s();
    startTurn(rel, true);
  }
}

void Robot::beginEdge(int from, uint8_t exit_dir, int known) {
  from_node_ = from;
  from_exit_ = exit_dir;
  heading_ = exit_dir;
  nseg_ = 0;
  exp_k_ = 0;
  exp_.known = false;
  exp_.edge = -1;
  exp_.target_node = -1;
  if (known >= 0) {
    const Edge& e = graph_.edge(known);
    bool fwd = (e.a == from && e.da == exit_dir);
    exp_.known = true;
    exp_.edge = known;
    exp_.nseg = e.nseg;
    for (int i = 0; i < e.nseg; ++i) {
      exp_.len[i] = fwd ? e.seg_len[i] : e.seg_len[e.nseg - 1 - i];
      exp_.turn[i] = (i + 1 < e.nseg) ? (fwd ? e.seg_turn[i] : (int8_t)-e.seg_turn[e.nseg - 2 - i]) : 0;
    }
    exp_.target_node = graph_.otherEnd(known, from, exit_dir, &exp_.arrive_exit);
  }
  float acc = run_ == RunKind::ACTUAL ? cfg.actual_accel_mm_s2 : cfg.explore_accel_mm_s2;
  float dec = run_ == RunKind::ACTUAL ? cfg.actual_decel_mm_s2 : cfg.explore_decel_mm_s2;
  motion_.follow(cruiseSpeed(), acc, dec);
  nav_ = Nav::FOLLOW;
  planSpeed();
}

void Robot::planSpeed() {
  if (!exp_.known || exp_k_ >= exp_.nseg) {
    motion_.clearStopPoint();
    return;
  }
  float s_exp = seg_start_s_ + exp_.len[exp_k_];
  bool is_node = exp_k_ == exp_.nseg - 1;
  float v_end = run_ == RunKind::ACTUAL ? cfg.actual_approach_mm_s : cfg.approach_speed_mm_s;
  if (is_node && run_ == RunKind::ACTUAL) {
    int next = route_idx_ + 1;
    if (next >= route_.n) {
      v_end = cfg.actual_pass_speed_mm_s;  // final edge ends in the box
    } else {
      uint8_t arr_h = dirOpp(exp_.arrive_exit);
      if (relTurn(arr_h, route_.steps[next].exit) == REL_STRAIGHT) v_end = cfg.actual_pass_speed_mm_s;
    }
  }
  v_end = minf(v_end, cruiseSpeed());
  float margin = run_ == RunKind::ACTUAL ? cfg.actual_margin_mm : cfg.actual_margin_mm * 0.7f;
  motion_.setStopPoint(s_exp - margin, v_end);
}

void Robot::startTurn(int rel, bool from_stop) {
  (void)from_stop;
  pending_rel_ = rel;
  turn_s_stop_ = odo_.s();
  float ang = rel == REL_LEFT ? 90.0f : (rel == REL_RIGHT ? -90.0f : 180.0f);
  float cap = rel == REL_BACK ? cfg.uturn_capture_start_deg : cfg.turn_capture_start_deg;
  float om = (run_ == RunKind::ACTUAL ? cfg.actual_turn_omega_deg_s : cfg.turn_omega_deg_s) * kDeg;
  motion_.pivot(ang * kDeg, om, cfg.turn_alpha_deg_s2 * kDeg, cap * kDeg, cfg.turn_max_overshoot_deg * kDeg);
  nav_ = Nav::TURNING;
}

void Robot::afterTurn() {
  uint8_t old_heading = heading_;
  heading_ = dirAdd(heading_, pending_rel_);
  (void)old_heading;
  if (pending_rel_ == REL_BACK) {
    // Stopping error along the old heading becomes an offset along the new one.
    seg_start_s_ = odo_.s() + (turn_s_stop_ - pending_s_center_);
  } else {
    seg_start_s_ = odo_.s();
  }
  detector_.rearm(odo_.s());
  sensors_.resetTracking(0);
  if (pending_corner_) {
    float acc = run_ == RunKind::ACTUAL ? cfg.actual_accel_mm_s2 : cfg.explore_accel_mm_s2;
    float dec = run_ == RunKind::ACTUAL ? cfg.actual_decel_mm_s2 : cfg.explore_decel_mm_s2;
    motion_.follow(cruiseSpeed(), acc, dec);
    nav_ = Nav::FOLLOW;
    planSpeed();
  } else {
    beginEdge(cur_node_, pending_exit_, next_known_edge_);
  }
}

int Robot::mapArrival(float x, float y, uint8_t exits, uint8_t arrive_exit, NodeType type) {
  int m = graph_.matchNode(x, y, exits, arrive_exit, cfg.node_match_base_mm, cfg.node_match_drift_frac,
                           cfg.node_match_max_mm, odo_.s());
  if (m >= 0 && m == from_node_ && nseg_ < 4) {
    // A loop back to the node we just left needs at least three corners.
    Log::printf("MAP candidate node %d rejected: impossible self-loop with %d corners", m, nseg_ - 1);
    m = -1;
  }
  if (m >= 0) {
    int e = graph_.node(m).edge[arrive_exit];
    if (e >= 0) {
      int other = graph_.otherEnd(e, m, arrive_exit, nullptr);
      if (other != from_node_) {
        Log::printf("MAP candidate node %d rejected: its %c line leads to node %d, we came from %d", m,
                    dirChar(arrive_exit), other, from_node_);
        m = -1;
        ++map_conflicts_;
      }
    }
  }
  if (m >= 0) {
    Node& n = graph_.node(m);
    float err = sqrtf((n.x - x) * (n.x - x) + (n.y - y) * (n.y - y));
    if (n.exits != exits) Log::printf("MAP node %d exit set %x vs observed %x (union kept)", m, n.exits, exits);
    n.exits |= exits;
    n.odo = odo_.s();
    if (n.visits < 255) ++n.visits;
    Log::printf("MAP loop closure at node %d (odometry error %.0f mm after %.0f mm)", m, err, dist_since_anchor_);
    dist_since_anchor_ = 0;
    markVerified();
    return m;
  }
  int id = graph_.addNode(x, y, exits, type);
  if (id >= 0) graph_.node(id).odo = odo_.s();
  if (id >= 0)
    Log::printf("MAP new node %d at (%.0f, %.0f) exits=%c%c%c%c type=%d", id, x, y, exits & 1 ? 'N' : '-',
                exits & 2 ? 'E' : '-', exits & 4 ? 'S' : '-', exits & 8 ? 'W' : '-', (int)type);
  return id;
}

void Robot::onEvent(const TrackEvent& ev) {
  feature_s_ = odo_.s();
  if (ev.type == EventType::LINE_LOST) {
    Log::printf("EVENT line lost (last seen %s)", ev.left ? "left" : "right");
    rec_side_ = ev.left ? 1.0f : -1.0f;
    rec_phase_ = 0;
    enter(RobotState::RECOVERY);
    return;
  }
  if (ev.type == EventType::GOAL) {
    onGoal(ev);
    return;
  }
  uint8_t exits = dirBit(dirOpp(heading_));
  if (ev.type == EventType::JUNCTION) {
    if (ev.straight) exits |= dirBit(heading_);
    if (ev.left) exits |= dirBit(dirAdd(heading_, -1));
    if (ev.right) exits |= dirBit(dirAdd(heading_, 1));
  }
  Log::printf("EVENT %s L%d S%d R%d at s=%.0f (bar %.0f mm, entry offset %.1f mm) heading %c",
              ev.type == EventType::DEAD_END ? "DEAD_END" : "JUNCTION", ev.left, ev.straight, ev.right,
              ev.s_center - seg_start_s_, ev.crossbar_mm, ev.entry_pos_mm, dirChar(heading_));
  if (ev.type == EventType::DEAD_END && run_ != RunKind::ACTUAL && nseg_ == 0 && from_node_ >= 0 &&
      graph_.node(from_node_).edge[from_exit_] < 0 && ev.s_center - seg_start_s_ < 60.0f) {
    onPhantomStraight();
    return;
  }
  if (exp_.known) {
    float s_exp = seg_start_s_ + exp_.len[exp_k_ < exp_.nseg ? exp_k_ : exp_.nseg - 1];
    float tol = 60.0f + 0.15f * exp_.len[exp_k_ < exp_.nseg ? exp_k_ : exp_.nseg - 1];
    if (ev.type == EventType::JUNCTION && ev.s_center < s_exp - tol) {
      Log::printf("EVENT ignored: %.0f mm before the expected feature", s_exp - ev.s_center);
      return;
    }
    if (exp_k_ < exp_.nseg - 1) {
      if (ev.type == EventType::DEAD_END) {
        Log::printf("ROUTE mismatch: dead end where the map has a corner");
        onNodeArrival(ev, exits, NodeType::DEAD_END);
        return;
      }
      onCorner(ev, exp_.turn[exp_k_]);
      return;
    }
    onNodeArrival(ev, exits, ev.type == EventType::DEAD_END ? NodeType::DEAD_END : NodeType::JUNCTION);
    return;
  }
  if (ev.type == EventType::DEAD_END) {
    onNodeArrival(ev, exits, NodeType::DEAD_END);
    return;
  }
  bool corner = !ev.straight && (ev.left != ev.right);
  if (corner) onCorner(ev, ev.left ? +1 : -1);
  else onNodeArrival(ev, exits, NodeType::JUNCTION);
}

// The line ends right at the junction we were leaving straight: that exit was
// a phantom (a branch pattern misread while crossing). The axle has not yet
// reached the junction, so remove the exit, stop on the junction and choose
// again as if we had just arrived.
void Robot::onPhantomStraight() {
  Node& n = graph_.node(from_node_);
  n.exits &= (uint8_t)~dirBit(from_exit_);
  Log::printf("MAP node %d has no %c line: exit removed, choosing again", from_node_, dirChar(from_exit_));
  cur_node_ = from_node_;
  seg_x_ = n.x;
  seg_y_ = n.y;
  uint8_t exit_dir = explorer_.decide(graph_, cur_node_, heading_);
  if (exit_dir == DIR_NONE) {
    fail(FaultCode::NO_FRONTIER);
    return;
  }
  pending_exit_ = exit_dir;
  next_known_edge_ = n.edge[exit_dir];
  pending_rel_ = relTurn(heading_, exit_dir);
  pending_corner_ = false;
  turn_is_deadend_ = false;
  pending_s_center_ = seg_start_s_;
  last_event_s_ = seg_start_s_;
  Log::printf("NAV node %d: take %c (%c)", cur_node_, relTurnChar((uint8_t)pending_rel_), dirChar(exit_dir));
  motion_.setStopPoint(seg_start_s_, 0);
  nav_ = Nav::STOPPING;
}

void Robot::segmentEnd(float s_at, int turn_sign) {
  float len = s_at - seg_start_s_;
  if (len < 0) len = 0;
  if (nseg_ < kMaxEdgeSegs) {
    seg_len_[nseg_] = len;
    seg_turn_[nseg_] = (int8_t)turn_sign;
    ++nseg_;
  } else {
    seg_len_[kMaxEdgeSegs - 1] += len;  // overflow: fold into last segment
  }
  seg_x_ += dirDx(heading_) * len;
  seg_y_ += dirDy(heading_) * len;
  dist_since_anchor_ += len;
  seg_start_s_ = s_at;
}

void Robot::onCorner(const TrackEvent& ev, int sign) {
  segmentEnd(ev.s_center, sign);
  if (exp_.known) ++exp_k_;
  pending_corner_ = true;
  turn_is_deadend_ = false;
  pending_rel_ = sign > 0 ? REL_LEFT : REL_RIGHT;
  pending_s_center_ = ev.s_center;
  motion_.setStopPoint(ev.s_center, 0);
  nav_ = Nav::STOPPING;
}

uint8_t Robot::decideExit(int node) {
  if (run_ == RunKind::ACTUAL) {
    ++route_idx_;
    if (route_idx_ < route_.n && route_.steps[route_idx_].node == node) return route_.steps[route_idx_].exit;
    Log::printf("ROUTE desync at node %d (step %d) - replanning to the goal", node, route_idx_);
    run_ = RunKind::ACTUAL_EXPLORE;
  }
  if (run_ == RunKind::ACTUAL_EXPLORE && graph_.goalNode() >= 0) {
    Route r;
    if (Planner::shortest(graph_, node, heading_, graph_.goalNode(), cfg.turn_tiebreak_mm, r) && r.n > 0) {
      explorer_.setPlan(r);
    }
  }
  if (run_ == RunKind::DRY && cfg.defer_goal && goal_peeked_ && graph_.goalNode() >= 0 && !explorer_.backtracking()) {
    Route r;
    if (Planner::shortest(graph_, 0, DIR_N, graph_.goalNode(), cfg.turn_tiebreak_mm, r)) {
      float lb = Planner::unexploredLowerBound(graph_, 0, DIR_N, graph_.goalNode());
      bool time_up = run_ms_ > 140000.0f;
      if (lb >= r.length - 20.0f || time_up || !graph_.hasFrontier()) {
        Route g;
        if (Planner::shortest(graph_, node, heading_, graph_.goalNode(), cfg.turn_tiebreak_mm, g) && g.n > 0) {
          Log::printf("DEFER route proven (lb %.0f >= %.0f) or time up: heading to the end box", lb, r.length);
          explorer_.setPlan(g);
          goal_peeked_ = false;  // next goal event is final
        }
      }
    }
  }
  return explorer_.decide(graph_, node, heading_);
}

void Robot::onNodeArrival(const TrackEvent& ev, uint8_t exits, NodeType type) {
  segmentEnd(ev.s_center, 0);
  float edge_len = 0;
  for (int i = 0; i < nseg_; ++i) edge_len += seg_len_[i];
  uint8_t arrive_exit = dirOpp(heading_);
  uint16_t lens[kMaxEdgeSegs];
  for (int i = 0; i < nseg_; ++i) lens[i] = (uint16_t)clampf(seg_len_[i], 0, 65535);
  int node;
  if (exp_.known && exp_.target_node >= 0) {
    node = exp_.target_node;
    Node& n = graph_.node(node);
    float err = sqrtf((n.x - seg_x_) * (n.x - seg_x_) + (n.y - seg_y_) * (n.y - seg_y_));
    if ((n.exits | exits) != n.exits || (type == NodeType::DEAD_END) != (n.type == (uint8_t)NodeType::DEAD_END))
      Log::printf("MAP known node %d: observed exits %x, map %x", node, exits, n.exits);
    if (n.visits < 255) ++n.visits;
    n.odo = odo_.s();
    if (run_ == RunKind::DRY)
      graph_.addEdge(from_node_, from_exit_, node, exp_.arrive_exit, edge_len, seg_turn_, lens, (uint8_t)nseg_);
    Log::printf("NAV arrived at known node %d (pose error %.0f mm)", node, err);
    dist_since_anchor_ = 0;
    if (run_ == RunKind::DRY && err > cfg.node_match_max_mm) {
      // Far from where the map puts this node: we are not where we think.
      Log::printf("MAP arrival error %.0f mm > %.0f mm: map inconsistent", err, cfg.node_match_max_mm);
      map_conflicts_ += 2;
    } else if (run_ == RunKind::DRY && (n.exits | exits) == n.exits) {
      markVerified();
    } else if (run_ == RunKind::DRY && type != NodeType::DEAD_END) {
      // Consistent arrival that shows exits the map lacks: a branch missed on
      // an earlier (tilted) pass, or a restart point whose side lines were
      // never seen. Keep the union, as a loop closure does.
      n.exits |= exits;
      Log::printf("MAP node %d gains exits -> %x", node, n.exits);
    }
  } else {
    node = mapArrival(seg_x_, seg_y_, exits, arrive_exit, type);
    if (node < 0 || graph_.addEdge(from_node_, from_exit_, node, arrive_exit, edge_len, seg_turn_, lens,
                                   (uint8_t)nseg_) < 0) {
      fail(FaultCode::MAP_FULL);
      return;
    }
  }
  cur_node_ = node;
  seg_x_ = graph_.node(node).x;
  seg_y_ = graph_.node(node).y;
  if (run_ == RunKind::DRY && !relocalizing_ && map_conflicts_ >= 2) startSelfRelocalization(exits);
  if (run_ == RunKind::DRY && relocalizing_) tryRelocalize();
  node = cur_node_;

  uint8_t exit_dir;
  if (type == NodeType::DEAD_END && run_ != RunKind::ACTUAL) {
    exit_dir = arrive_exit;
    if (run_ == RunKind::DRY || run_ == RunKind::ACTUAL_EXPLORE) {
      // keep the explorer's bookkeeping consistent (it may hold a plan)
      if (explorer_.backtracking()) explorer_.reset();
    }
  } else {
    exit_dir = decideExit(node);
  }
  if (exit_dir == DIR_NONE) {
    if (run_ == RunKind::DRY) fail(FaultCode::NO_FRONTIER);
    else fail(FaultCode::ROUTE_MISMATCH);
    return;
  }
  int rel = relTurn(heading_, exit_dir);
  pending_exit_ = exit_dir;
  next_known_edge_ = graph_.node(node).edge[exit_dir];
  pending_s_center_ = ev.s_center;
  pending_corner_ = false;
  last_event_s_ = ev.s_center;
  Log::printf("NAV node %d: take %c (%c)", node, relTurnChar((uint8_t)rel), dirChar(exit_dir));
  if (type == NodeType::DEAD_END) {
    turn_is_deadend_ = true;
    pending_rel_ = REL_BACK;
    float dec = run_ == RunKind::ACTUAL ? cfg.actual_decel_mm_s2 : cfg.explore_decel_mm_s2;
    motion_.brake(dec);
    nav_ = Nav::STOPPING;
  } else if (rel == REL_STRAIGHT) {
    seg_start_s_ = ev.s_center;
    beginEdge(node, exit_dir, next_known_edge_);
  } else {
    turn_is_deadend_ = false;
    pending_rel_ = rel;
    motion_.setStopPoint(ev.s_center, 0);
    nav_ = Nav::STOPPING;
  }
}

void Robot::markVerified() {
  verified_nodes_ = graph_.nodeCount();
  verified_edges_ = graph_.edgeCount();
  map_conflicts_ = 0;
}

// Two topology conflicts since the last verified loop closure mean the map no
// longer matches the arena - typically the robot was bumped or carried onto a
// different line without a restart. Roll the map back to its last verified
// state and re-join it exactly as after a checkpoint restart.
void Robot::startSelfRelocalization(uint8_t here_exits) {
  Log::printf("MAP inconsistent (%d conflicts): rolling back to %d nodes and re-localising", map_conflicts_,
              verified_nodes_);
  old_graph_ = graph_;
  old_graph_.truncate(verified_nodes_, verified_edges_);
  graph_.clear();
  graph_.addNode(0, 0, here_exits, NodeType::START);  // virtual node: the junction we stand on
  cur_node_ = 0;
  seg_x_ = seg_y_ = 0;
  relocalizing_ = true;
  explorer_.reset();
  map_conflicts_ = 0;
  dist_since_anchor_ = 0;
}

void Robot::tryRelocalize(bool force) {
  Alignment a;
  bool ok = false;
  int n = Relocalizer::alignAnchored(old_graph_, graph_, 150.0f, a);
  int local_nodes = graph_.nodeCount() - 1;  // excluding the virtual restart node
  // An anchor needs at least two re-observed nodes as evidence (a checkpoint
  // can lie on a line the old map never reached, and then one coincidental
  // match makes a wrong anchor look unique), and must be unique or beat the
  // runner-up by two matches.
  if (n >= 1 && local_nodes >= 2 && a.matched >= 2 && (n == 1 || a.matched >= a.runner_up + 2)) {
    if (n > 1)
      Log::printf("RELOC anchor node %d clearly best (%d vs %d matches)", a.anchor_old, a.matched, a.runner_up);
    ok = true;
  } else {
    Alignment f;
    if (Relocalizer::align(old_graph_, graph_, 150.0f, f)) {
      a = f;
      ok = true;
    } else if (n >= 1 && force) {
      Log::printf("RELOC %d anchored candidate(s) still consistent: choosing anchor node %d (%d matches, most recent)",
                  n, a.anchor_old, a.matched);
      ok = true;
    }
  }
  if (!ok) return;
  int map[kMaxNodes];
  MazeGraph merged = old_graph_;
  if (!Relocalizer::merge(merged, graph_, a, map)) {
    Log::printf("RELOC merge failed (map full)");
    return;
  }
  Log::printf("RELOC aligned: anchor=%d rot=%d t=(%.0f, %.0f), %d nodes matched -> merged map %d nodes",
              a.anchor_old, a.rot, a.tx, a.ty, a.matched, merged.nodeCount());
  if (map[cur_node_] < 0) return;  // robot still on the unanchored restart point
  graph_ = merged;
  cur_node_ = map[cur_node_];
  heading_ = dirAdd(heading_, a.rot);
  seg_x_ = graph_.node(cur_node_).x;
  seg_y_ = graph_.node(cur_node_).y;
  relocalizing_ = false;
  explorer_.reset();
  dist_since_anchor_ = 0;
  markVerified();
}

void Robot::onGoal(const TrackEvent& ev) {
  segmentEnd(ev.s_center, 0);
  float edge_len = 0;
  for (int i = 0; i < nseg_; ++i) edge_len += seg_len_[i];
  uint8_t arrive_exit = dirOpp(heading_);
  goal_s_center_ = ev.s_center;
  goal_heading_ = heading_;
  Log::printf("EVENT GOAL: white for %.0f mm, box entry at (%.0f, %.0f), motor trim %+.3f", cfg.goal_confirm_mm,
              seg_x_, seg_y_, motion_.motorTrim());
  if (run_ == RunKind::DRY || run_ == RunKind::ACTUAL_EXPLORE) {
    int gnode = graph_.goalNode();
    if (gnode < 0) {
      gnode = graph_.addNode(seg_x_, seg_y_, dirBit(arrive_exit), NodeType::GOAL);
      if (gnode < 0) { fail(FaultCode::MAP_FULL); return; }
      graph_.setGoal(gnode);
    }
    uint16_t lens[kMaxEdgeSegs];
    for (int i = 0; i < nseg_; ++i) lens[i] = (uint16_t)clampf(seg_len_[i], 0, 65535);
    if (graph_.node(gnode).edge[arrive_exit] < 0 || from_node_ != gnode)
      graph_.addEdge(from_node_, from_exit_, gnode, arrive_exit, edge_len, seg_turn_, lens, (uint8_t)nseg_);
    cur_node_ = gnode;
    if (run_ == RunKind::DRY && relocalizing_) tryRelocalize(true);
    if (run_ == RunKind::DRY && cfg.defer_goal && !goal_peeked_ && !relocalizing_ && graph_.hasFrontier()) {
      Route r;
      if (Planner::shortest(graph_, 0, DIR_N, graph_.goalNode(), cfg.turn_tiebreak_mm, r)) {
        float lb = Planner::unexploredLowerBound(graph_, 0, DIR_N, graph_.goalNode());
        if (lb < r.length - 20.0f) {
          Log::printf("DEFER goal peeked: known route %.0f mm, unexplored bound %.0f mm -> keep exploring", r.length,
                      lb);
          goal_peeked_ = true;
          turn_is_deadend_ = true;
          pending_corner_ = false;
          pending_rel_ = REL_BACK;
          pending_exit_ = arrive_exit;
          next_known_edge_ = graph_.node(cur_node_).edge[arrive_exit];
          pending_s_center_ = ev.s_center;
          motion_.brake(cfg.explore_decel_mm_s2);
          nav_ = Nav::STOPPING;
          return;
        }
      }
    }
  }
  led_red_ = true;  // official requirement: red LED glows at the end zone
  Log::printf("LED red ON");
  motion_.straight(cfg.goal_stop_extra_mm, minf(maxf(odo_.v(), 200.0f), 400.0f), 3000.0f,
                   run_ == RunKind::ACTUAL ? cfg.actual_decel_mm_s2 : cfg.explore_decel_mm_s2);
  goal_.beginConfirm(odo_.s());
  nav_ = Nav::GOAL_DRIVE;
  enter(RobotState::GOAL_CONFIRMATION);
}

void Robot::finishGoal() {
  motion_.idle();
  nav_ = Nav::NONE;
  if (run_ == RunKind::DRY) {
    Log::printf("DRY RUN complete in %.1f s (robot clock)", run_ms_ / 1000.0f);
    enter(RobotState::MAP_VALIDATION);
  } else {
    Log::printf("ACTUAL RUN complete in %.1f s (robot clock)%s", run_ms_ / 1000.0f,
                run_ == RunKind::ACTUAL_EXPLORE ? " via fallback search" : "");
    if (run_ == RunKind::ACTUAL) stage_ = Stage::ACTUAL_DONE;
    requestSave();
    enter(RobotState::FINISHED);
  }
}

void Robot::runStep(float dt) {
  run_ms_ += dt * 1000.0f;
  switch (nav_) {
    case Nav::NONE:
      break;
    case Nav::FOLLOW: {
      motion_.holdHeading(detector_.crossing());
      TrackEvent ev;
      if (detector_.update(feat_, odo_.s(), ev)) onEvent(ev);
      break;
    }
    case Nav::STOPPING: {
      bool stopped = absf(odo_.v()) < 25.0f && motion_.vCmd() < 1.0f;
      if (motion_.mode() == MotionMode::BRAKE) stopped = motion_.done();
      if (stopped) startTurn(pending_rel_, true);
      break;
    }
    case Nav::TURNING:
      if (motion_.done()) {
        if (motion_.failed()) {
          if (!pending_corner_ && !turn_is_deadend_ && run_ != RunKind::ACTUAL) {
            // The exit we tried does not exist (false branch). Remove it, turn
            // back to the arrival heading and decide again.
            Node& n = graph_.node(cur_node_);
            if (n.edge[pending_exit_] < 0) n.exits &= (uint8_t)~dirBit(pending_exit_);
            Log::printf("TURN no line at node %d exit %c: exit removed, realigning", cur_node_, dirChar(pending_exit_));
            motion_.pivot(-motion_.pivotRotated(), 4.0f, cfg.turn_alpha_deg_s2 * kDeg, -1, 0);
            nav_ = Nav::REALIGN;
          } else {
            Log::printf("TURN failed to capture the line");
            rec_side_ = pending_rel_ == REL_RIGHT ? 1.0f : -1.0f;  // search back towards where we came from
            rec_phase_ = 0;
            rec_after_turn_ = true;
            enter(RobotState::RECOVERY);
          }
        } else {
          afterTurn();
        }
      }
      break;
    case Nav::REALIGN:
      if (motion_.done()) {
        uint8_t exit_dir = explorer_.decide(graph_, cur_node_, heading_);
        if (exit_dir == DIR_NONE) {
          fail(FaultCode::NO_FRONTIER);
          break;
        }
        departNode(cur_node_, exit_dir);
      }
      break;
    case Nav::GOAL_DRIVE: {
      int r = goal_.updateConfirm(feat_.n_on >= cfg.num_sensors - 2, odo_.s());
      if (r > 0 || (motion_.done() && r == 0)) {
        motion_.brake(cfg.explore_decel_mm_s2);
        nav_ = Nav::GOAL_STOP;
      } else if (r < 0) {
        Log::printf("GOAL false alarm: white lost during confirmation - reversing to the junction");
        led_red_ = false;
        motion_.straight(goal_s_center_ - odo_.s(), 200.0f, 1500.0f, 1500.0f);
        nav_ = Nav::REVERSE;
        enter(run_ == RunKind::DRY ? RobotState::DRY_RUN : RobotState::ACTUAL_RUN);
      }
      break;
    }
    case Nav::GOAL_STOP:
      if (motion_.done()) finishGoal();
      break;
    case Nav::REVERSE:
      if (motion_.done()) {
        TrackEvent ev;
        ev.type = EventType::JUNCTION;
        ev.left = ev.right = true;
        ev.straight = feat_.line_seen;
        ev.s_center = odo_.s();
        ev.s_detect = odo_.s();
        detector_.rearm(odo_.s());
        motion_.follow(cfg.approach_speed_mm_s, cfg.explore_accel_mm_s2, cfg.explore_decel_mm_s2);
        nav_ = Nav::FOLLOW;
        onNodeArrival(ev, (uint8_t)(dirBit(dirOpp(heading_)) | dirBit(dirAdd(heading_, 1)) |
                                    dirBit(dirAdd(heading_, -1)) | (ev.straight ? dirBit(heading_) : 0)),
                      NodeType::JUNCTION);
      }
      break;
  }
}

// ---------------------------------------------------------------- recovery --
void Robot::recoveryStep(float dt) {
  (void)dt;
  float sweep = cfg.recover_sweep_deg * kDeg;
  switch (rec_phase_) {
    case 0:
      if (++recoveries_ > cfg.max_recoveries) {
        fail(rec_after_turn_ ? FaultCode::TURN_FAILED : FaultCode::LINE_LOST);
        return;
      }
      motion_.brake(3000.0f);
      rec_phase_ = 1;
      break;
    case 1:
      if (motion_.done()) {
        motion_.pivot(rec_side_ * sweep, 4.0f, 30.0f, 0.0f, 0.0f);
        rec_phase_ = 2;
      }
      break;
    case 2:
    case 3:
      if (motion_.done()) {
        if (!motion_.failed()) {
          Log::printf("RECOVERY line re-acquired");
          rec_after_turn_ = false;
          detector_.rearm(odo_.s());
          sensors_.resetTracking(feat_.pos_mm);
          float acc = run_ == RunKind::ACTUAL ? cfg.actual_accel_mm_s2 : cfg.explore_accel_mm_s2;
          float dec = run_ == RunKind::ACTUAL ? cfg.actual_decel_mm_s2 : cfg.explore_decel_mm_s2;
          if (nav_ == Nav::TURNING) {
            // recovered at the end of a turn: treat as completed turn
            afterTurn();
          } else {
            motion_.follow(cfg.approach_speed_mm_s, acc, dec);
            nav_ = Nav::FOLLOW;
            planSpeed();
          }
          enter(run_ == RunKind::DRY ? RobotState::DRY_RUN : RobotState::ACTUAL_RUN);
        } else if (rec_phase_ == 2) {
          motion_.pivot(-rec_side_ * 2.0f * sweep, 4.0f, 30.0f, 0.0f, 0.0f);
          rec_phase_ = 3;
        } else {
          motion_.pivot(rec_side_ * sweep, 4.0f, 30.0f, -1, 0);  // back to the original heading
          rec_phase_ = 4;
        }
      }
      break;
    case 4:
      if (motion_.done()) {
        motion_.straight(-60.0f, 150.0f, 1000.0f, 1000.0f);  // back up onto the line
        rec_phase_ = 5;
      }
      break;
    case 5:
      if (motion_.done()) rec_phase_ = 0;
      break;
  }
}

}  // namespace mesh
