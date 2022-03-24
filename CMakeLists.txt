cmake_minimum_required(VERSION 3.0.2)
project(mpl_calibration_toolbox)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  include
	${OpenCV_LIBRARY_DIRS}
)

add_executable(image_publisher src/image_publisher.cpp)
add_executable(joint_extrinsic_calibrator src/joint_extrinsic_calibrator.cpp)

target_link_libraries(image_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(joint_extrinsic_calibrator yaml-cpp ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBRARIES})
