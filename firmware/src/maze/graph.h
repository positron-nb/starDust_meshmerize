// graph.h - the discovered maze as a graph of junctions and line segments.
//
// Node  = a decision point: junction (3-4 exits), dead end, START, GOAL entry.
//         Plain 90-degree corners are NOT nodes; they are stored inside the
//         edge polyline because the robot never chooses anything there.
// Edge  = the line between two nodes, leaving node a in absolute direction
//         da and entering node b from direction db (db is the exit of b that
//         leads back along the edge). The polyline is a list of straight
//         segment lengths separated by +-90 degree corner turns.
//
// Positions are odometry estimates in mm (map frame: x east, y north, START
// at the origin facing north). Heading is discrete, so position error is
// distance-proportional only (wheel-diameter error, slip), typically 1-3 %.
#pragma once
#include <stdint.h>

#include "config.h"
#include "robot_types.h"

namespace mesh {

enum class NodeType : uint8_t { JUNCTION = 0, DEAD_END = 1, START = 2, GOAL = 3 };

struct Node {
  float x, y;
  uint8_t exits;      // bit d = a line leaves this node in absolute direction d
  uint8_t type;       // NodeType
  uint8_t visits;
  uint8_t flags;      // bit0: position fixed by loop closure
  int16_t edge[4];    // edge index per exit direction, -1 = unexplored
  float odo;          // odometry s when the robot was last on this node
};

struct Edge {
  uint8_t a, da, b, db;
  float length;               // total centreline length, mm
  uint8_t nseg;               // number of straight segments (corners = nseg-1)
  uint8_t traversals;
  int8_t seg_turn[kMaxEdgeSegs];   // turn AFTER segment i, travelling a->b: +1 left, -1 right
  uint16_t seg_len[kMaxEdgeSegs];  // mm
};

class MazeGraph {
 public:
  void clear();
  int nodeCount() const { return n_nodes_; }
  int edgeCount() const { return n_edges_; }
  const Node& node(int i) const { return nodes_[i]; }
  Node& node(int i) { return nodes_[i]; }
  const Edge& edge(int i) const { return edges_[i]; }
  Edge& edge(int i) { return edges_[i]; }

  int addNode(float x, float y, uint8_t exits, NodeType type);
  // Find a known node compatible with an observation. Returns -1 if none.
  // arrival_exit: the exit of the observed node the robot arrived through.
  // The radius for node i is min(max_radius, radius + drift_frac * (odo_now - node.odo)):
  // position error grows with the distance driven SINCE THAT NODE was last
  // visited, not with the total distance (a fresh node has a tight radius).
  int matchNode(float x, float y, uint8_t exits, uint8_t arrival_exit, float radius, float drift_frac = 0,
                float max_radius = 1e9f, float odo_now = 0) const;
  // Adds (or re-uses) the edge a/da <-> b/db. Returns edge index or -1 if full.
  int addEdge(int a, uint8_t da, int b, uint8_t db, float length, const int8_t* turns,
              const uint16_t* lens, uint8_t nseg);
  // Endpoint of an edge seen from `from` leaving in `dir`.
  int otherEnd(int e, int from, uint8_t from_dir, uint8_t* arrive_exit) const;
  uint8_t unexploredExits(int n) const;
  bool hasFrontier() const;
  int goalNode() const { return goal_; }
  void setGoal(int n) { goal_ = n; }
  int startNode() const { return 0; }
  bool full() const { return n_nodes_ >= kMaxNodes || n_edges_ >= kMaxEdges; }
  // Number of corner turns inside an edge (used as turn cost).
  int edgeTurns(int e) const { return edges_[e].nseg > 0 ? edges_[e].nseg - 1 : 0; }
  // Heading on arrival at the far end when traversing e from `from`.
  uint8_t arrivalHeading(int e, int from) const;
  // Consistency checks; returns number of problems found (logged).
  int validate() const;
  // Merge node `drop` into node `keep` (loop closure found late).
  void mergeNodes(int keep, int drop);
  // Drop every node >= n_nodes and edge >= n_edges (map rollback).
  void truncate(int n_nodes, int n_edges);

 private:
  Node nodes_[kMaxNodes];
  Edge edges_[kMaxEdges];
  int n_nodes_ = 0, n_edges_ = 0;
  int goal_ = -1;
};

}  // namespace mesh
