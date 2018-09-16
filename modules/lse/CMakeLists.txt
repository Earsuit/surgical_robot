cmake_minimum_required(VERSION 2.8)
project(lse_node)

# need c++11 for to_String()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

add_executable(${PROJECT_NAME}_log "lse_node_log.cpp")
target_link_libraries(${PROJECT_NAME}_log ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_log surgical_robot_generate_messages_cpp)