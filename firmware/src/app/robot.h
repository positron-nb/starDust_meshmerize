// robot.h - the complete competition controller ("robot brain").
//
// Robot::step() is called at 1 kHz with a HalInputs snapshot and fills
// HalOutputs. It contains NO hardware access, so the very same object runs
// on the ESP32 (hal_esp32 + main.cpp) and inside the PC simulator
// (sim_api.cpp, loaded by Python through ctypes).
//
// Competition state machine (docs/system_architecture.md has the diagram):
//
//  BOOT -> IDLE --short press--> ARMED --> DRY_RUN --goal--> GOAL_CONFIRMATION
//            |                                 |                     |
//            |--long press--> CALIBRATING      |fault          MAP_VALIDATION
//            |--very long---> CLEAR_MEMORY     v                     |
//            |                             RECOVERY --fail--> FAULT  ROUTE_PLANNING
//            |                                                  |    |
//            |                      (button = restart) <--------+    WAIT_FOR_ACTUAL_RUN
//            |                                                        |
//            +--short press (stage DRY_DONE)--> ARMED --> ACTUAL_RUN --goal--> FINISHED
//
// Stage (persisted) decides what a short press starts:
//   FRESH       -> new dry run
//   DRY_PARTIAL -> dry-run restart from a checkpoint (relocalisation)
//   DRY_DONE    -> actual run along the stored route (restart = from START)
#pragma once
#include "../encoders/odometry.h"
#include "../exploration/explorer.h"
#include "../exploration/relocalizer.h"
#include "../goal_detection/goal_detector.h"
#include "../maze/graph.h"
#include "../maze/junction_detector.h"
#include "../motion/motion_controller.h"
#include "../persistence/storage.h"
#include "../planner/planner.h"
#include "../safety/fault_monitor.h"
#include "../sensors/line_sensor.h"
#include "config.h"
#include "robot_types.h"

namespace mesh {

enum class RunKind : uint8_t { NONE = 0, DRY, ACTUAL, ACTUAL_EXPLORE };

class Robot {
 public:
  RobotConfig cfg;

  void init();
  // Load persisted data (call once after init, before the first step).
  void loadPersisted(const uint8_t* buf, size_t len);
  size_t serialize(uint8_t* buf, size_t cap) const;
  void step(const HalInputs& in, HalOutputs& out);

  // ---- introspection (simulator, debug) ----
  RobotState state() const { return state_; }
  Stage stage() const { return stage_; }
  RunKind runKind() const { return run_; }
  FaultCode fault() const { return fault_; }
  const MazeGraph& graph() const { return graph_; }
  const Route& route() const { return route_; }
  const LineFeatures& features() const { return feat_; }
  float poseX() const;
  float poseY() const;
  uint8_t heading() const { return heading_; }
  bool relocalizing() const { return relocalizing_; }
  int restartsUsed() const { return restarts_; }
  int routeIndex() const { return route_idx_; }
  float lowerBound() const { return lower_bound_; }
  bool ledRed() const { return led_red_; }
  const LineSensor& lineSensor() const { return sensors_; }
  const Odometry& odometry() const { return odo_; }
  float motorTrim() const { return motion_.motorTrim(); }  // learned L/R motor balance (no encoders)

 private:
  enum class Nav : uint8_t { NONE, FOLLOW, STOPPING, TURNING, REALIGN, GOAL_DRIVE, GOAL_STOP, REVERSE };
  struct Expect {          // oriented polyline of the edge being traversed (known edges)
    bool known = false;
    int edge = -1;
    int target_node = -1;
    uint8_t arrive_exit = 0;
    int nseg = 0;
    float len[kMaxEdgeSegs];
    int8_t turn[kMaxEdgeSegs];
  };

  // state machine pieces
  void enter(RobotState s);
  void handleButton(const HalInputs& in, float dt);
  void startRun(RunKind kind, bool restart);
  void runStep(float dt);
  void calibrateStep(float dt);
  void recoveryStep(float dt);
  void fail(FaultCode code);
  void requestSave() { save_pending_ = true; }

  // navigation
  void beginEdge(int from_node, uint8_t exit_dir, int known_edge);
  void departNode(int node, uint8_t exit_dir);
  void onPhantomStraight();
  void onEvent(const TrackEvent& ev);
  void onCorner(const TrackEvent& ev, int turn_sign);
  void onNodeArrival(const TrackEvent& ev, uint8_t exits_abs, NodeType type);
  void onGoal(const TrackEvent& ev);
  void finishGoal();
  void startTurn(int rel, bool corner_or_node_uturn);
  void afterTurn();
  void planSpeed();
  float cruiseSpeed() const;
  void segmentEnd(float s_at, int turn_sign);
  void setPoseAt(float s_center);
  int mapArrival(float x, float y, uint8_t exits, uint8_t arrive_exit, NodeType type);
  void tryRelocalize(bool force = false);
  void markVerified();
  void startSelfRelocalization(uint8_t here_exits);  // exits observed where we stand
  uint8_t decideExit(int node);
  void completeDryRun();

  // modules
  LineSensor sensors_;
  LineFeatures feat_{};
  Odometry odo_;
  MotionController motion_;
  JunctionDetector detector_;
  GoalDetector goal_;
  Explorer explorer_;
  FaultMonitor faults_;
  MazeGraph graph_;
  MazeGraph old_graph_;   // pre-restart map while relocalising
  Route route_;

  // state
  RobotState state_ = RobotState::BOOT;
  Stage stage_ = Stage::FRESH;
  RunKind run_ = RunKind::NONE;
  FaultCode fault_ = FaultCode::NONE;
  Nav nav_ = Nav::NONE;
  uint32_t last_us_ = 0;
  bool first_step_ = true;
  float vbat_ = 0;
  const uint16_t* raw_ = nullptr;
  bool rec_after_turn_ = false;
  float state_ms_ = 0;     // time in current state
  float run_ms_ = 0;       // time since the run started
  bool led_red_ = false;
  bool save_pending_ = false;
  int restarts_ = 0;
  int recoveries_ = 0;

  // button
  bool btn_prev_ = false;
  float btn_ms_ = 0;
  RunKind armed_kind_ = RunKind::NONE;
  bool armed_restart_ = false;

  // pose / edge under construction
  float seg_x_ = 0, seg_y_ = 0, seg_start_s_ = 0;
  uint8_t heading_ = DIR_N;
  int from_node_ = 0;
  uint8_t from_exit_ = DIR_N;
  int cur_node_ = 0;
  int nseg_ = 0;
  float seg_len_[kMaxEdgeSegs];
  int8_t seg_turn_[kMaxEdgeSegs];
  float dist_since_anchor_ = 0;
  Expect exp_;
  int exp_k_ = 0;          // index of the next expected feature (corner k or node)
  float last_event_s_ = -1e9f;
  float feature_s_ = 0;  // odometer at the last track event (no-encoder stall check)

  // pending turn
  int pending_rel_ = 0;
  bool pending_corner_ = false;
  int pending_turn_sign_ = 0;
  uint8_t pending_exit_ = 0;
  float pending_s_center_ = 0;
  float turn_s_stop_ = 0;
  bool turn_is_deadend_ = false;
  int next_known_edge_ = -1;
  bool goal_peeked_ = false;
  float lower_bound_ = 0;

  // actual run
  int route_idx_ = 0;

  // relocalisation
  bool relocalizing_ = false;
  int verified_nodes_ = 1, verified_edges_ = 0;
  int map_conflicts_ = 0;

  // recovery
  int rec_phase_ = 0;
  Nav rec_resume_nav_ = Nav::NONE;
  float rec_side_ = 1;

  // calibration
  int cal_phase_ = 0;
  bool cal_checked_ = false;
  bool auto_start_ = false;
  float cal_l0_ = 0, cal_r0_ = 0, cal_g0_ = 0;
  float cal_theta0_ = 0;

  // goal
  float goal_s_center_ = 0;
  uint8_t goal_heading_ = 0;
};

}  // namespace mesh
