// Copyright (c) 2021 Bartosz Meglicki <meglickib@gmail.com>
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
// limitations under the License.
//

#include <iostream>
#include <memory>

#include "../collision_checker.hpp"
#include "../a_star.hpp"

class MapMock
{
public:
	bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my)
	{
	  if (wx < origin_x || wy < origin_y)
		 return false;
		 
	  mx = static_cast<unsigned int>((wx - origin_x) / resolution);
	  my = static_cast<unsigned int>((wy - origin_y) / resolution);

	  if (mx < size_x && my < size_y)
		 return true;

	  return false;
	}

	void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
	{
		wx = origin_x + (mx + 0.5) * resolution;
		wy = origin_y + (my + 0.5) * resolution;		
	}

	double getCost(int x, int y)
	{		
		if(x > 40 && x <= 60 && y > 40 && y <= 60)
			return 254;
			
		return 0;
	}

	double getCost(int idx)
	{
		int x = idx % getSizeInCellsX();
		int y = idx / getSizeInCellsY();
		
		return getCost(x, y);
	}

	unsigned int getSizeInCellsX()
	{
		return size_x;
	}
	
	unsigned int getSizeInCellsY()
	{
		return size_y;
	}

	static constexpr double UNKNOWN = 255;
	static constexpr double OCCUPIED = 254;
	static constexpr double INSCRIBED = 253;
	static constexpr double FREE = 0;

private:
	double resolution = 0.1;
	double origin_x = 0.0;
	double origin_y = 0.0;
	double size_x = 100;
	double size_y = 100;
};

struct PointMock
{
	double x;
	double y;
};

using namespace fcc;
using namespace std;
using namespace nav2_smac_planner;

using namespace std;

int main(int /*argc*/, char **/*argv*/)
{
	MapMock *map = new MapMock();
	
	//initialize some footprint, just a vector<PointMock> here
	FootprintCollisionChecker<MapMock, PointMock>::Footprint footprint;
	
	footprint.push_back( {-1, -1} );
	footprint.push_back( {1, -1} );
	footprint.push_back( {1, 1});
	footprint.push_back( {-1, 1} );

	SearchInfo info;

	info.change_penalty = 1.2;
	info.non_straight_penalty = 1.4;
	info.reverse_penalty = 2.1;
	info.minimum_turning_radius = 2.0;  // in grid coordinates
	unsigned int size_theta = 72;

	AStarAlgorithm<MapMock, GridCollisionChecker<MapMock, PointMock>> a_star(nav2_smac_planner::MotionModel::DUBIN, info);

	int max_iterations = 10000;
	float tolerance = 10.0;
	int it_on_approach = 10;
	int num_it = 0;

	a_star.initialize(false, max_iterations, it_on_approach);
	a_star.setFootprint(footprint, true);

	// functional case testing
	a_star.createGraph(map->getSizeInCellsX(), map->getSizeInCellsY(), size_theta, map);
	a_star.setStart(10u, 10u, 0u);
	a_star.setGoal(80u, 80u, 40u);
	nav2_smac_planner::NodeSE2::CoordinateVector plan;
	bool found = a_star.createPath(plan, num_it, tolerance);

	cout << "found path: " << found << endl;
	cout << "num_it " << num_it << endl;
	cout << "path size " << plan.size() << endl;

	cout << "done!" << endl;

	return 0;
}
