// relocalizer.h - re-join the dry-run map after a restart from a checkpoint
// (or after the robot was displaced without a restart).
//
// Rule (Restarts #2): a dry-run restart begins at the checkpoint, and the
// team may not feed arena information to the robot. The robot therefore
// does not know where it has been placed. It keeps the pre-restart map,
// starts a new local map whose node 0 is the restart point, and after every
// new local node tests alignments of the local map onto the old one.
//
// ANCHORED alignment (preferred). The referee puts the robot on the last
// checkpoint it crossed, and in the sample arena every checkpoint is a
// junction. So local node 0 must coincide with an OLD junction, and the robot
// faces one of that junction's exits. Candidates are therefore
// (old junction o, rotation r with the robot's heading being an exit of o).
// A candidate stays consistent while every local node lands either on an old
// node with the same exit set or on unexplored ground. When only one is left
// it is accepted. If several are still consistent when the end box is
// reached, the one with the most matched nodes and then the most recently
// visited anchor wins (the robot failed shortly after crossing it).
//
// FREE alignment (fallback, if no anchored candidate is consistent):
// accepted when exactly one (old node, rotation) maps at least two local
// junctions onto old junctions with identical exit sets.
#pragma once
#include "../maze/graph.h"

namespace mesh {

struct Alignment {
  int rot = 0;        // quarter turns clockwise: dir_old = dir_local + rot
  float tx = 0, ty = 0;
  int matched = 0;
  int anchor_old = -1;  // old node that local node 0 sits on (anchored mode)
  float recency = 0;    // odometry mark of the anchor (higher = visited later)
  int runner_up = 0;    // anchored mode: matches of the second-best consistent candidate
};

class Relocalizer {
 public:
  // Returns the number of consistent anchored candidates; `best` is filled
  // with the preferred one when the count is > 0.
  static int alignAnchored(const MazeGraph& old_map, const MazeGraph& local, float radius, Alignment& best);
  // Free alignment; true only if exactly one alignment is consistent and it
  // re-observes at least three old nodes.
  static bool align(const MazeGraph& old_map, const MazeGraph& local, float radius, Alignment& out);
  // Merge `local` into `old_map` using `a`. In anchored mode local node 0
  // becomes a.anchor_old; otherwise node 0 and its edges are dropped.
  // map_local_to_old receives the node index mapping. False if the map is full.
  static bool merge(MazeGraph& old_map, const MazeGraph& local, const Alignment& a, int* map_local_to_old);
  static void transform(const Alignment& a, float x, float y, float& ox, float& oy);
  static uint8_t rotMask(uint8_t mask, int rot);
};

}  // namespace mesh
