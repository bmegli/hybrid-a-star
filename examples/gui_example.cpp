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

	bool displayToMap(double dx, double dy, unsigned int &mx, unsigned int &my)
	{
	  mx = static_cast<unsigned int>(dx * size_x / display_x);
	  my = static_cast<unsigned int>(dy * size_y / display_y);

	  if (mx < size_x && my < size_y)
		 return true;

	  return false;
	}

	void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
	{
		wx = origin_x + (mx + 0.5) * resolution;
		wy = origin_y + (my + 0.5) * resolution;		
	}

	void mapToDisplay(unsigned int mx, unsigned int my, double & dx, double & dy) const
	{
		dx = origin_x + (mx + 0.5) * display_x/size_x;
		dy = origin_y + (my + 0.5) * display_y/size_y;
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

	
	int getSizeInCellsX()
	{
		return size_x;
	}
	
	int getSizeInCellsY()
	{
		return size_y;
	}

	int getDisplayX()
	{
		return display_x;
	}
	
	int getDisplayY()
	{
		return display_y;
	}

	bool fromImage(const std::string &filename)
	{
		using namespace cv;
		
		data = imread(filename, IMREAD_GRAYSCALE);

		if(data.empty())
			return false;
			
		resize(data, data, Size(size_x, size_y));
		threshold(data, data, 127, 254, THRESH_BINARY_INV);

		return true;
	}
	
	const cv::Mat &getData()
	{
		return data;
	}
		
	double getResolution() const
	{
		return resolution;
	}

private:
	cv::Mat data;
	double resolution = 0.5;
	double origin_x = 0.0;
	double origin_y = 0.0;
	double size_x = 200;
	double size_y = 200;
	double display_x = 1000;
	double display_y = 1000;
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
	cv::Point start;
	cv::Point end;
};

void OnMouse(int event, int x, int y, int /*flags*/, void* userdata)
{
	State *state = (State*)userdata;
	
	if( event == cv::EVENT_LBUTTONDOWN )
	{
		unsigned int mx, my;
		state->map->displayToMap(x, y, mx, my);
		state->start = cv::Point(mx, my);
		cout << "mx " << mx << " my " << my << endl;
	}

	if( event == cv::EVENT_RBUTTONDOWN )
	{
		unsigned int mx, my;
		state->map->displayToMap(x, y, mx, my);
		state->end = cv::Point(mx, my);
		cout << "mx" << mx << " my" << my << endl;
	}	
}

void Display(MapMock *map, const NodeSE2::CoordinateVector &path)
{
	cv::Mat img;
	cv::resize(map->getData(), img, cv::Size(map->getDisplayX(), map->getDisplayY()));
	threshold(img, img, 127, 254, cv::THRESH_BINARY);
	
	img = ~img;
	
	for (size_t i = 0; i != path.size(); ++i)
	{
		double dx, dy;
		map->mapToDisplay(path[i].x, path[i].y, dx, dy);
		cv::circle(img, cv::Point(dx, dy), 10, 127, 1);
	}

	cv::imshow("map", img);
}

void MainLoop(State *state)
{
	MapMock *map = state->map;
	
	do
	{
		cout << "LMB: start | RMB: goal | ANY: plan | ESC: quit" << endl;
		
		FootprintCollisionChecker<MapMock*, PointMock>::Footprint footprint;

		footprint.push_back( {-1, -1} );
		footprint.push_back( {1, -1} );
		footprint.push_back( {1, 1});
		footprint.push_back( {-1, 1} );

		SearchInfo info;

		info.change_penalty = 1.2;
		info.non_straight_penalty = 1.4;
		info.reverse_penalty = 2.1;
		info.minimum_turning_radius = 4;  // in grid coordinates

		unsigned int size_theta = 72;

		AStarAlgorithm<MapMock, GridCollisionChecker<MapMock*, PointMock>> a_star(nav2_smac_planner::MotionModel::DUBIN, info);

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

		bool found = a_star.createPath(path, num_it, tolerance);

		cout << "found " << found << endl;
		cout << "num_it " << num_it << endl;
		cout << "path size " << path.size() << endl;
		
		Display(map, path);
		
	}while(cv::waitKey(0) != 27);
}

int main(int argc, char **argv)
{	
	MapMock *map = new MapMock();

	if(!map->fromImage("../maps/maze.png"))
	{
		cerr << "failed to load map, terminating" << endl;
		return 1;
	}

	State state {map, cv::Point(40, 100), cv::Point(180, 100)};

	cv::namedWindow("map", 1);
	cv::setMouseCallback("map", OnMouse, &state);
	MainLoop(&state);

	delete map;

	cout << "done!" << endl;

	return 0;
}