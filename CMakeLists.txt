cmake_minimum_required(VERSION 2.8.3)
project(line_counter_gui)


find_package(catkin REQUIRED
  roscpp
  geometry_msgs
  std_msgs
)

find_package(OpenCV 3.0 REQUIRED COMPONENTS
  imgproc
  highgui
  imgcodecs
)
catkin_package()

add_definitions("-std=c++11")

include_directories(
   perception_lib/
)

add_executable(line_counter_gui src/line_counter_gui.cpp)

target_link_libraries(line_counter_gui
 ${catkin_LIBRARIES}
 ${OpenCV_LIBS}
)
