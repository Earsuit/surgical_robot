project(serial_node)

add_executable(serial_rx_node serial_rx_node.cpp)
target_link_libraries(serial_rx_node ${catkin_LIBRARIES})
add_dependencies(serial_rx_node surgical_robot_generate_messages_cpp)

