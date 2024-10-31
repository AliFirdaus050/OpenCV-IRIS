cmake_minimum_required(VERSION 3.0.2)
project(robot_tracker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(robot_tracker src/robot_tracker.cpp)

target_link_libraries(robot_tracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)