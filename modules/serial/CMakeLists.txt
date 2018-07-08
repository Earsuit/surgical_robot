project(serial_node)

include_directories(Peripherals/include)
file(GLOB SOURCES "Peripherals/src/*.cpp" "serial_rx_node.cpp")

add_executable(serial_rx_node  ${SOURCES})
find_package(glog 0.3.5 REQUIRED)
target_link_libraries(serial_rx_node ${catkin_LIBRARIES} glog::glog)
add_dependencies(serial_rx_node surgical_robot_generate_messages_cpp)

