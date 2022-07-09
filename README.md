# Hybrid A*

C++ Hybrid A*
- rewritten from [ROS2 navigation2](https://github.com/ros-planning/navigation2/tree/378d53435856f4b3377fc5031b7118e4554410e5) stack
- with refactored out ROS2 dependencies

Implementation is mostly based on 2020 [nav2_smac_planner](https://github.com/ros-planning/navigation2/tree/main/nav2_smac_planner) by [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/) while at [Samsung Research](https://www.sra.samsung.com/). This is also the source for high level information about the algorithm.

## State

Functional:
- Hybrid A* already usable
- Smoother already usable

Note:
- interface and implementation subject to change
- CMake build working but preliminary

As far as I remember implementation (from original) throws exception if there is no possible path.
This exception is not caught in examples.

The code was refactored out of ROS2 in 2021 and original was probably updated since.

## Dependencies

- ompl
- eigen
- ceres
- opencv (only for GUI example)

Tested with system libraries on Ubuntu 18.04 and 20.04

## Building Instructions

```bash
sudo apt-get install libompl-dev libeigen3-dev libceres-dev libopencv-dev
# don't forget recursive, library uses submodules
git clone --recursive https://github.com/bmegli/hybrid-a-star.git
cd hybrid-a-star
mkdir build
cd build
cmake ..
make
```

For Ubuntu 20.04 or custom OpenCV build set OpenCV path in CMakeLists.txt before `cmake ..`

## Running Examples

```bash
./simple-example
```

```bash
./gui-example
```

Use:
- left mouse button -> start position
- right mouse button -> goal position
- any key -> plan
- esc -> quit


Optionally enable smoother in `gui_example.cpp` `main` and recompile.

## Using

See examples for now.

## License

A mix of various open sources licenses, mainly:
- Apache 2.0
- BSD 2.0
- BSD

See individual files.

