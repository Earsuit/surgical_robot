cmake_minimum_required(VERSION 2.8)
project(serial_node)

include_directories(Peripherals/include)
file(GLOB SOURCES "Peripherals/src/*.cpp")
find_package(glog 0.3.5 REQUIRED)

add_executable(${PROJECT_NAME}  ${SOURCES} "serial_node.cpp")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} glog::glog)
add_dependencies(${PROJECT_NAME} surgical_robot_generate_messages_cpp)

add_executable(${PROJECT_NAME}_LSE  ${SOURCES} "serial_node_LSE.cpp")
target_link_libraries(${PROJECT_NAME}_LSE ${catkin_LIBRARIES} glog::glog)
add_dependencies(${PROJECT_NAME}_LSE surgical_robot_generate_messages_cpp)
