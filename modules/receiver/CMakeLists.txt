cmake_minimum_required(VERSION 2.8)
project(joystick)

include_directories(Peripherals/include)
file(GLOB SOURCES "Peripherals/src/*.cpp" "serial_node.cpp")

add_executable(joystick ${SOURCES})
find_package(glog 0.3.5 REQUIRED)
target_link_libraries(joystick ${catkin_LIBRARIES} glog::glog)
add_dependencies(joystick surgical_robot_generate_messages_cpp)