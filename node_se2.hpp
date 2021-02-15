// Copyright (c) 2020, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.
//
// Modifications copyright (C) 2021 Bartosz Meglicki <meglickib@gmail.com>

#ifndef NAV2_SMAC_PLANNER__NODE_SE2_HPP_
#define NAV2_SMAC_PLANNER__NODE_SE2_HPP_

#include <vector>
#include <queue>
#include <functional>
#include <memory>

#include <ompl/base/StateSpace.h>

#include "constants.hpp"
#include "types.hpp"
#include "collision_checker.hpp"

namespace nav2_smac_planner
{

// Need seperate pose struct for motion table operations

/**
 * @struct nav2_smac_planner::MotionPose
 * @brief A struct for poses in motion primitives
 */
struct MotionPose
{
  /**
   * @brief A constructor for nav2_smac_planner::MotionPose
   */
  MotionPose() {}

  /**
   * @brief A constructor for nav2_smac_planner::MotionPose
   * @param x X pose
   * @param y Y pose
   * @param theta Angle of pose
   */
  MotionPose(const float & x, const float & y, const float & theta)
  : _x(x), _y(y), _theta(theta)
  {}

  float _x;
  float _y;
  float _theta;
};

typedef std::vector<MotionPose> MotionPoses;

// Must forward declare
class NodeSE2;

/**
 * @struct nav2_smac_planner::MotionTable
 * @brief A table of motion primitives and related functions
 */
struct MotionTable
{
  /**
   * @brief A constructor for nav2_smac_planner::MotionTable
   */
  MotionTable() {}

  /**
   * @brief Initializing using Dubin model
   * @param size_x_in Size of costmap in X
   * @param size_y_in Size of costmap in Y
   * @param angle_quantization_in Size of costmap in bin sizes
   * @param search_info Parameters for searching
   */
  void initDubin(
    unsigned int & size_x_in,
    unsigned int & size_y_in,
    unsigned int & angle_quantization_in,
    SearchInfo & search_info);

  /**
   * @brief Initializing using Reeds-Shepp model
   * @param size_x_in Size of costmap in X
   * @param size_y_in Size of costmap in Y
   * @param angle_quantization_in Size of costmap in bin sizes
   * @param search_info Parameters for searching
   */
  void initReedsShepp(
    unsigned int & size_x_in,
    unsigned int & size_y_in,
    unsigned int & angle_quantization_in,
    SearchInfo & search_info);

  /**
   * @brief Get projections of motion models
   * @param node Ptr to SE2 node
   * @return A set of motion poses
   */
  MotionPoses getProjections(const NodeSE2 * node);

  /**
   * @brief Get a projection of motion model
   * @param node Ptr to SE2 node
   * @return A motion pose
   */
  MotionPose getProjection(const NodeSE2 * node, const unsigned int & motion_index);

  MotionPoses projections;
  unsigned int size_x;
  unsigned int num_angle_quantization;
  float num_angle_quantization_float;
  float bin_size;
  float change_penalty;
  float non_straight_penalty;
  float cost_penalty;
  float reverse_penalty;
  ompl::base::StateSpacePtr state_space;
};

/**
 * @class nav2_smac_planner::NodeSE2
 * @brief NodeSE2 implementation for graph
 */
class NodeSE2
{
public:
  typedef NodeSE2 * NodePtr;
  typedef std::unique_ptr<std::vector<NodeSE2>> Graph;
  typedef std::vector<NodePtr> NodeVector;
  /**
   * @class nav2_smac_planner::NodeSE2::Coordinates
   * @brief NodeSE2 implementation of coordinate structure
   */
  struct Coordinates
  {
    /**
     * @brief A constructor for nav2_smac_planner::NodeSE2::Coordinates
     */
    Coordinates() {}

    /**
     * @brief A constructor for nav2_smac_planner::NodeSE2::Coordinates
     * @param x_in X coordinate
     * @param y_in Y coordinate
     * @param theta_in Theta coordinate
     */
    Coordinates(const float & x_in, const float & y_in, const float & theta_in)
    : x(x_in), y(y_in), theta(theta_in)
    {}

    float x, y, theta;
  };

  typedef std::vector<Coordinates> CoordinateVector;

  /**
   * @brief A constructor for nav2_smac_planner::NodeSE2
   * @param index The index of this node for self-reference
   */
  explicit NodeSE2(const unsigned int index);

  /**
   * @brief A destructor for nav2_smac_planner::NodeSE2
   */
  ~NodeSE2();

  /**
   * @brief operator== for comparisons
   * @param NodeSE2 right hand side node reference
   * @return If cell indicies are equal
   */
  bool operator==(const NodeSE2 & rhs)
  {
    return this->_index == rhs._index;
  }

  /**
   * @brief setting continuous coordinate search poses (in partial-cells)
   * @param Pose pose
   */
  inline void setPose(const Coordinates & pose_in)
  {
    pose = pose_in;
  }

  /**
   * @brief Reset method for new search
   */
  void reset();

  /**
   * @brief Gets the accumulated cost at this node
   * @return accumulated cost
   */
  inline float & getAccumulatedCost()
  {
    return _accumulated_cost;
  }

  /**
   * @brief Sets the accumulated cost at this node
   * @param reference to accumulated cost
   */
  inline void setAccumulatedCost(const float cost_in)
  {
    _accumulated_cost = cost_in;
  }

  /**
   * @brief Sets the motion primitive index used to achieve node in search
   * @param reference to motion primitive idx
   */
  inline void setMotionPrimitiveIndex(const unsigned int & idx)
  {
    _motion_primitive_index = idx;
  }

  /**
   * @brief Gets the motion primitive index used to achieve node in search
   * @return reference to motion primitive idx
   */
  inline unsigned int & getMotionPrimitiveIndex()
  {
    return _motion_primitive_index;
  }

  /**
   * @brief Gets the costmap cost at this node
   * @return costmap cost
   */
  inline float & getCost()
  {
    return _cell_cost;
  }

  /**
   * @brief Gets if cell has been visited in search
   * @param If cell was visited
   */
  inline bool & wasVisited()
  {
    return _was_visited;
  }

  /**
   * @brief Sets if cell has been visited in search
   */
  inline void visited()
  {
    _was_visited = true;
    _is_queued = false;
  }

  /**
   * @brief Gets if cell is currently queued in search
   * @param If cell was queued
   */
  inline bool & isQueued()
  {
    return _is_queued;
  }

  /**
   * @brief Sets if cell is currently queued in search
   */
  inline void queued()
  {
    _is_queued = true;
  }

  /**
   * @brief Gets cell index
   * @return Reference to cell index
   */
  inline unsigned int & getIndex()
  {
    return _index;
  }

  /**
   * @brief Check if this node is valid
   * @param traverse_unknown If we can explore unknown nodes on the graph
   * @return whether this node is valid and collision free
   */
  template <typename CollisionChecker>	
  bool isNodeValid(const bool & traverse_unknown, CollisionChecker collision_checker);

  /**
   * @brief Get traversal cost of parent node to child node
   * @param child Node pointer to child
   * @return traversal cost
   */
  float getTraversalCost(const NodePtr & child);

  /**
   * @brief Get index at coordinates
   * @param x X coordinate of point
   * @param y Y coordinate of point
   * @param angle Theta coordinate of point
   * @param width Width of costmap
   * @param angle_quantization Number of theta bins
   * @return Index
   */
  static inline unsigned int getIndex(
    const unsigned int & x, const unsigned int & y, const unsigned int & angle,
    const unsigned int & width, const unsigned int angle_quantization)
  {
    return angle + x * angle_quantization + y * width * angle_quantization;
  }

  /**
   * @brief Get index at coordinates
   * @param x X coordinate of point
   * @param y Y coordinate of point
   * @param angle Theta coordinate of point
   * @return Index
   */
  static inline unsigned int getIndex(
    const unsigned int & x, const unsigned int & y, const unsigned int & angle)
  {
    return getIndex(
      x, y, angle, motion_table.size_x,
      motion_table.num_angle_quantization);
  }

  /**
   * @brief Get coordinates at index
   * @param index Index of point
   * @param width Width of costmap
   * @param angle_quantization Theta size of costmap
   * @return Coordinates
   */
  static inline Coordinates getCoords(
    const unsigned int & index,
    const unsigned int & width, const unsigned int angle_quantization)
  {
    return Coordinates(
      (index / angle_quantization) % width,    // x
      index / (angle_quantization * width),    // y
      index % angle_quantization);    // theta
  }

  /**
   * @brief Get cost of heuristic of node
   * @param node Node index current
   * @param node Node index of new
   * @return Heuristic cost between the nodes
   */
  static float getHeuristicCost(
    const Coordinates & node_coords,
    const Coordinates & goal_coordinates);

  /**
   * @brief Initialize motion models
   * @param motion_model Motion model enum to use
   * @param size_x Size of X of graph
   * @param size_y Size of y of graph
   * @param angle_quantization Size of theta bins of graph
   * @param search_info Search info to use
   */
  static void initMotionModel(
    const MotionModel & motion_model,
    unsigned int & size_x,
    unsigned int & size_y,
    unsigned int & angle_quantization,
    SearchInfo & search_info);

  /**
   * @brief Compute the wavefront heuristic
   * @param costmap Costmap to use to compute heuristic
   * @param start_x Coordinate of Start X
   * @param start_y Coordinate of Start Y
   * @param goal_x Coordinate of Goal X
   * @param goal_y Coordinate of Goal Y
   */
  template <typename CostmapT>
  static void computeWavefrontHeuristic(
    CostmapT * & costmap,
    const unsigned int & start_x, const unsigned int & start_y,
    const unsigned int & goal_x, const unsigned int & goal_y);

  /**
   * @brief Retrieve all valid neighbors of a node.
   * @param node Pointer to the node we are currently exploring in A*
   * @param validity_checker Functor for state validity checking
   * @param neighbors Vector of neighbors to be filled
   */
  template <typename CollisionChecker>	
  static void getNeighbors(
    const NodePtr & node,
    std::function<bool(const unsigned int &, nav2_smac_planner::NodeSE2 * &)> & validity_checker,
    CollisionChecker collision_checker,
    const bool & traverse_unknown,
    NodeVector & neighbors);

  NodeSE2 * parent;
  Coordinates pose;
  static double neutral_cost;
  static MotionTable motion_table;

private:
  float _cell_cost;
  float _accumulated_cost;
  unsigned int _index;
  bool _was_visited;
  bool _is_queued;
  unsigned int _motion_primitive_index;
  static std::vector<unsigned int> _wavefront_heuristic;
};

template <typename CollisionChecker>	
bool NodeSE2::isNodeValid(const bool & traverse_unknown, CollisionChecker collision_checker)
{
  if (collision_checker.inCollision(
      this->pose.x, this->pose.y, this->pose.theta * motion_table.bin_size, traverse_unknown))
  {
    return false;
  }

  _cell_cost = collision_checker.getCost();
  return true;
}

template <typename CollisionChecker>
void NodeSE2::getNeighbors(
  const NodePtr & node,
  std::function<bool(const unsigned int &, nav2_smac_planner::NodeSE2 * &)> & NeighborGetter,
  CollisionChecker collision_checker,
  const bool & traverse_unknown,
  NodeVector & neighbors)
{
  unsigned int index = 0;
  NodePtr neighbor = nullptr;
  Coordinates initial_node_coords;
  const MotionPoses motion_projections = motion_table.getProjections(node);

  for (unsigned int i = 0; i != motion_projections.size(); i++) {
    index = NodeSE2::getIndex(
      static_cast<unsigned int>(motion_projections[i]._x),
      static_cast<unsigned int>(motion_projections[i]._y),
      static_cast<unsigned int>(motion_projections[i]._theta),
      motion_table.size_x, motion_table.num_angle_quantization);

    if (NeighborGetter(index, neighbor) && !neighbor->wasVisited()) {
      // Cache the initial pose in case it was visited but valid
      // don't want to disrupt continuous coordinate expansion
      initial_node_coords = neighbor->pose;
      neighbor->setPose(
        Coordinates(
          motion_projections[i]._x,
          motion_projections[i]._y,
          motion_projections[i]._theta));
      if (neighbor->isNodeValid(traverse_unknown, collision_checker)) {
        neighbor->setMotionPrimitiveIndex(i);
        neighbors.push_back(neighbor);
      } else {
        neighbor->setPose(initial_node_coords);
      }
    }
  }
}

template <typename CostmapT>
void NodeSE2::computeWavefrontHeuristic(
  CostmapT * & costmap,
  const unsigned int & start_x, const unsigned int & start_y,
  const unsigned int & goal_x, const unsigned int & goal_y)
{
  unsigned int size = costmap->getSizeInCellsX() * costmap->getSizeInCellsY();
  if (_wavefront_heuristic.size() == size) {
    // must reset all values
    for (unsigned int i = 0; i != _wavefront_heuristic.size(); i++) {
      _wavefront_heuristic[i] = 0;
    }
  } else {
    unsigned int wavefront_size = _wavefront_heuristic.size();
    _wavefront_heuristic.resize(size, 0);
    // must reset values for non-constructed indices
    for (unsigned int i = 0; i != wavefront_size; i++) {
      _wavefront_heuristic[i] = 0;
    }
  }

  const unsigned int & size_x = motion_table.size_x;
  const int size_x_int = static_cast<int>(size_x);
  const unsigned int size_y = costmap->getSizeInCellsY();
  const unsigned int goal_index = goal_y * size_x + goal_x;
  const unsigned int start_index = start_y * size_x + start_x;
  unsigned int mx, my, mx_idx, my_idx;

  std::queue<unsigned int> q;
  q.emplace(goal_index);

  unsigned int idx = goal_index;
  _wavefront_heuristic[idx] = 2;

  static const std::vector<int> neighborhood = {1, -1,  // left right
    size_x_int, -size_x_int,  // up down
    size_x_int + 1, size_x_int - 1,  // upper diagonals
    -size_x_int + 1, -size_x_int - 1};  // lower diagonals

  while (!q.empty() || idx == start_index) {
    // get next one
    idx = q.front();
    q.pop();

    my_idx = idx / size_x;
    mx_idx = idx - (my_idx * size_x);

    // find neighbors
    for (unsigned int i = 0; i != neighborhood.size(); i++) {
      unsigned int new_idx = static_cast<unsigned int>(static_cast<int>(idx) + neighborhood[i]);
      unsigned int last_wave_cost = _wavefront_heuristic[idx];

      // if neighbor is unvisited and non-lethal, set N and add to queue
      if (new_idx > 0 && new_idx < size_x * size_y &&
        _wavefront_heuristic[new_idx] == 0 &&
        static_cast<float>(costmap->getCost(idx)) < CostmapT::INSCRIBED)
      {
        my = new_idx / size_x;
        mx = new_idx - (my * size_x);

        if ( (mx == 0 && mx_idx >= size_x - 1) || (mx >= size_x - 1 && mx_idx == 0) )
          continue;

        if ( (my == 0 && my_idx >= size_y - 1) || (my >= size_y - 1 && my_idx == 0) )
          continue;

        _wavefront_heuristic[new_idx] = last_wave_cost + 1;
        q.emplace(idx + neighborhood[i]);
      }
    }
  }
}

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__NODE_SE2_HPP_
