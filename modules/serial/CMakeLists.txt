cmake_minimum_required(VERSION 2.8)
project(serial_node)

include_directories(Peripherals/include)
file(GLOB SOURCES "Peripherals/src/*.cpp" "serial_node.cpp")

add_executable(serial_node  ${SOURCES})
find_package(glog 0.3.5 REQUIRED)
target_link_libraries(serial_node ${catkin_LIBRARIES} glog::glog)
add_dependencies(serial_node surgical_robot_generate_messages_cpp)

