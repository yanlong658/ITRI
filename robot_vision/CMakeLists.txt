cmake_minimum_required(VERSION 2.8.3)
project(robot_vision)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
#set(cv_bridge_DIR /usr/local/cv_bridge340/share/cv_bridge/cmake)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	sensor_msgs
	tf
	image_transport
	cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hw11
#  CATKIN_DEPENDS roscpp rospy std_msgs
DEPENDS EIGEN3 PCL
)

#inlude myself library
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
#INCLUDE_DIRECTORIES("/home/yanlong658/library/opencv_contrib/modules/xfeatures2d/include")


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(initial 
src/initial.cpp
src/process.cpp
src/parameter.cpp)
target_link_libraries(initial ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(grayImage src/grayImage.cpp)
target_link_libraries(grayImage ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(location src/location.cpp)
target_link_libraries(location ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
