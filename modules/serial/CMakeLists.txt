cmake_minimum_required(VERSION 2.8)
project(serial_node)

include_directories(RS-232)
file(GLOB SOURCES "RS-232/rs232.c")

add_executable(${PROJECT_NAME}  ${SOURCES} "serial_node.cpp")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} surgical_robot_generate_messages_cpp)

add_executable(${PROJECT_NAME}_LSE  ${SOURCES} "serial_node_LSE.cpp")
target_link_libraries(${PROJECT_NAME}_LSE ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_LSE surgical_robot_generate_messages_cpp)
