cmake_minimum_required(VERSION 2.8)
project(closeloop)

add_executable(${PROJECT_NAME} "closeloop_node.cpp")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} surgical_robot_generate_messages_cpp)