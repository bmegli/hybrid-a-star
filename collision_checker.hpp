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

#include "footprint-collision-checker/footprint_collision_checker.hpp"
#include "constants.hpp"

#ifndef NAV2_SMAC_PLANNER__COLLISION_CHECKER_HPP_
#define NAV2_SMAC_PLANNER__COLLISION_CHECKER_HPP_

namespace nav2_smac_planner
{

/**
 * @class nav2_smac_planner::GridCollisionChecker
 * @brief A costmap grid collision checker
 */
template<typename CostmapT, typename PointT>
class GridCollisionChecker
  : public fcc::FootprintCollisionChecker<CostmapT, PointT>
{
public:
  typedef typename fcc::FootprintCollisionChecker<CostmapT, PointT>::Footprint Footprint;

  /**
   * @brief A constructor for nav2_smac_planner::GridCollisionChecker
   * @param costmap The costmap to collision check against
   */
  GridCollisionChecker(CostmapT *costmap)
  : fcc::FootprintCollisionChecker<CostmapT, PointT>(costmap)
  {
  }

  /**
   * @brief Set the footprint to use with collision checker
   * @param footprint The footprint to collision check against
   * @param radius Whether or not the footprint is a circle and use radius collision checking
   */
  void setFootprint(const typename fcc::FootprintCollisionChecker<CostmapT, PointT>::Footprint &footprint, const bool &radius)
  {
    unoriented_footprint_ = footprint;
    footprint_is_radius_ = radius;
  }

  /**
   * @brief Check if in collision with costmap and footprint at pose
   * @param x X coordinate of pose to check against
   * @param y Y coordinate of pose to check against
   * @param theta Angle of pose to check against
   * @param traverse_unknown Whether or not to traverse in unknown space
   * @return boolean if in collision or not.
   */
  bool inCollision(
    const float & x,
    const float & y,
    const float & theta,
    const bool & traverse_unknown)
  {
    // Assumes setFootprint already set
    double wx, wy;
    this->costmap_->mapToWorld(static_cast<double>(x), static_cast<double>(y), wx, wy);

    if (!footprint_is_radius_) {
      // if footprint, then we check for the footprint's points
      footprint_cost_ = this->footprintCostAtPose(
        wx, wy, static_cast<double>(theta), unoriented_footprint_);
      if (traverse_unknown && footprint_cost_ == CostmapT::UNKNOWN) {
        return false;
      }

      // if occupied or unknown and not to traverse unknown space
      return footprint_cost_ >= CostmapT::OCCUPIED;
    } else {
      // if radius, then we can check the center of the cost assuming inflation is used
      footprint_cost_ = this->costmap_->getCost(
        static_cast<unsigned int>(x), static_cast<unsigned int>(y));

      if (traverse_unknown && footprint_cost_ == CostmapT::UNKNOWN) {
        return false;
      }

      // if occupied or unknown and not to traverse unknown space
      return footprint_cost_ >= CostmapT::INSCRIBED;
    }
  }

  /**
   * @brief Get cost at footprint pose in costmap
   * @return the cost at the pose in costmap
   */
  float getCost()
  {
    // Assumes inCollision called prior
    return static_cast<float>(footprint_cost_);
  }

protected:
  typename fcc::FootprintCollisionChecker<CostmapT, PointT>::Footprint unoriented_footprint_;
  double footprint_cost_;
  bool footprint_is_radius_;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__COLLISION_CHECKER_HPP_
