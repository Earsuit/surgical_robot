cmake_minimum_required(VERSION 2.8)
project(lse_log_node)

add_executable(${PROJECT_NAME}  ${SOURCES} "lse_log_node.cpp")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} surgical_robot_generate_messages_cpp)