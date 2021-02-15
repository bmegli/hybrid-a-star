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

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "../collision_checker.hpp"
#include "../a_star.hpp"
#include "../smoother.hpp"

class MapMock
{
public:
	bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my)
	{
	  if (wx < origin_x || wy < origin_y)
		 return false;
 
	  mx = static_cast<unsigned int>((wx - origin_x) / (world_x / size_x));
	  my = static_cast<unsigned int>((wy - origin_y) / (world_y / size_y));

	  if (mx < size_x && my < size_y)
		 return true;

	  return false;
	}

	void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
	{
		wx = origin_x + (mx + 0.5) * (world_x / size_x);
		wy = origin_y + (my + 0.5) * (world_y / size_y);
	}

	double getCost(int x, int y)
	{
		return data.at<uint8_t>(y, x);
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

	unsigned int getWorldX()
	{
		return world_x;
	}

	unsigned int getWorldY()
	{
		return world_y;
	}

	bool fromImage(const std::string &filename)
	{
		using namespace cv;

		data = imread(filename, IMREAD_GRAYSCALE);

		if(data.empty())
			return false;

		world_x = data.cols;
		world_y = data.rows;

		resize(data, data, Size(size_x, size_y));
		threshold(data, data, 127, 254, THRESH_BINARY_INV);

		return true;
	}

	const cv::Mat &getData()
	{
		return data;
	}

	static constexpr double UNKNOWN = 255;
	static constexpr double OCCUPIED = 254;
	static constexpr double INSCRIBED = 253;
	static constexpr double FREE = 0;

private:
	cv::Mat data;
	double origin_x = 0.0;
	double origin_y = 0.0;
	double size_x = 200;
	double size_y = 200;
	double world_x;
	double world_y;
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

struct State
{
	MapMock *map;
	Smoother<MapMock> *smoother;
	cv::Point start;
	cv::Point end;
};

void OnMouse(int event, int x, int y, int /*flags*/, void* userdata)
{
	State *state = (State*)userdata;

	if( event == cv::EVENT_LBUTTONDOWN )
	{
		unsigned int mx, my;
		state->map->worldToMap(x, y, mx, my);
		state->start = cv::Point(mx, my);
		cout << "mx " << mx << " my " << my << " wx " << x << " wy " << y <<  endl;
	}

	if( event == cv::EVENT_RBUTTONDOWN )
	{
		unsigned int mx, my;
		state->map->worldToMap(x, y, mx, my);
		state->end = cv::Point(mx, my);
		cout << "mx" << mx << " my" << my << " wx " << x << " wy " << y << endl;
	}
}

void Display(MapMock *map,
             const std::vector<Eigen::Vector2d> &pathWorld,
             const std::vector<Eigen::Vector2d> &pathSmooth)
{
	cv::Mat img;
	cv::resize(map->getData(), img, cv::Size(map->getWorldX(), map->getWorldY()));
	threshold(img, img, 127, 254, cv::THRESH_BINARY);

	img = ~img;

	for (size_t i = 0; i != pathWorld.size(); ++i)
		cv::circle(img, cv::Point(pathWorld[i].x(), pathWorld[i].y()), 5, 127, 1);

	for (size_t i = 0; i != pathSmooth.size(); ++i)
		cv::circle(img, cv::Point(pathSmooth[i].x(), pathSmooth[i].y()), 5, 0, 1);


	cv::imshow("map", img);
}

void Smooth(State *state, const SearchInfo &info, std::vector<Eigen::Vector2d> &path)
{
	//may require extra tuning
	SmootherParams smoother_params;
	smoother_params.max_curvature = 1.0f / info.minimum_turning_radius;
	smoother_params.curvature_weight = 30.0;
	smoother_params.distance_weight = 0.0;
	smoother_params.smooth_weight = 100.0;
	smoother_params.costmap_weight = 0.025;
	smoother_params.max_time = 0.1; //limit optimization time

	// Smooth plan
	if (!state->smoother->smooth(path, state->map, smoother_params))
	{
		cerr << "failed to smooth plan, Ceres could not find a usable solution to optimize." << endl;
	}
}

void MainLoop(State *state)
{
	MapMock *map = state->map;
	Smoother<MapMock> *smoother = state->smoother;

	do
	{
		cout << "LMB: start | RMB: goal | ANY: plan | ESC: quit" << endl;

		FootprintCollisionChecker<MapMock, PointMock>::Footprint footprint;

		//in world units
		footprint.push_back( {-5, -5} );
		footprint.push_back( {5, -5} );
		footprint.push_back( {5, 5});
		footprint.push_back( {-5, 5} );

		SearchInfo info;

		info.change_penalty = 1.2;
		info.non_straight_penalty = 1.4;
		info.reverse_penalty = 2.1;
		info.minimum_turning_radius = 5;  //in world units

		unsigned int size_theta = 72;

		AStarAlgorithm<MapMock, GridCollisionChecker<MapMock, PointMock>> a_star(nav2_smac_planner::MotionModel::DUBIN, info);

		int max_iterations = 100000;
		int it_on_approach = 100;

		a_star.initialize(false, max_iterations, it_on_approach);
		a_star.setFootprint(footprint, false);

		a_star.createGraph(map->getSizeInCellsX(), map->getSizeInCellsY(), size_theta, map);
		a_star.setStart(state->start.x, state->start.y, 0u);
		a_star.setGoal(state->end.x, state->end.y, 0u);

		NodeSE2::CoordinateVector path;

		float tolerance = 5.0;
		int num_it = 0;
		bool found;

		try
		{
			found = a_star.createPath(path, num_it, tolerance);
		}
		catch (const std::runtime_error & e)
		{
			cerr << "failed to plan: " << e.what() << endl;
			continue;
		}

		cout << "found " << found << endl;
		cout << "num_it " << num_it << " of max " << max_iterations << endl;
		cout << "path size " << path.size() << endl;

		if(!found)
			continue;

		// Convert to world coordinates
		std::vector<Eigen::Vector2d> path_world, path_smoothed;
		path_world.reserve(path.size());
		path_smoothed.reserve(path.size());

		for (int i = path.size() - 1; i >= 0; --i)
		{
			double wx, wy;
			map->mapToWorld(path[i].x, path[i].y, wx, wy);
			path_world.push_back(Eigen::Vector2d(wx, wy));
			path_smoothed.push_back(Eigen::Vector2d(wx, wy));
		}

		if(smoother)
			Smooth(state, info, path_smoothed);

		Display(map, path_world, path_smoothed);

	}while(cv::waitKey(0) != 27);
}

int main(int argc, char **argv)
{
	bool enableSmoother = false; //change to true to enable smoother

	MapMock *map = new MapMock();
	Smoother<MapMock> *smoother = nullptr;

	if(!map->fromImage("../maps/maze.png"))
	{
		cerr << "failed to load map, terminating" << endl;
		return 1;
	}

	if(enableSmoother)
	{
		smoother = new Smoother<MapMock>();
		OptimizerParams params; //may require extra tuning!
		params.debug = false; //enable extra output to console
		smoother->initialize(params);
	}

	State state {map, smoother, cv::Point(40, 100), cv::Point(180, 100)};

	cv::namedWindow("map", 1);
	cv::setMouseCallback("map", OnMouse, &state);
	MainLoop(&state);

	delete map;
	delete smoother;

	cout << "done!" << endl;

	return 0;
}
