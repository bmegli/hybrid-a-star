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

#ifndef NAV2_SMAC_PLANNER__NODE_BASIC_HPP_
#define NAV2_SMAC_PLANNER__NODE_BASIC_HPP_

namespace nav2_smac_planner
{

/**
 * @class nav2_smac_planner::NodeBasic
 * @brief NodeBasic implementation for priority queue insertion
 */
template<typename NodeT>
class NodeBasic
{
public:
  /**
   * @brief A constructor for nav2_smac_planner::NodeBasic
   * @param cost_in The costmap cost at this node
   * @param index The index of this node for self-reference
   */
  explicit NodeBasic(const unsigned int index)
  : graph_node_ptr(nullptr), 
    index(index)
    
  {
  }

  typename NodeT::Coordinates pose;  // Used by NodeSE2
  NodeT * graph_node_ptr;
  unsigned int index;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__NODE_BASIC_HPP_
