cmake_minimum_required(VERSION 2.8.3)
project(rovi_industrial)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
