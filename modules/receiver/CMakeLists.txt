cmake_minimum_required(VERSION 2.8)
project(joystick)

include_directories(Peripherals/include)
file(GLOB SOURCES "Peripherals/src/*.cpp" "joystick_node.cpp")

add_executable(${PROJECT_NAME} ${SOURCES})
find_package(glog 0.3.5 REQUIRED)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} glog::glog)
add_dependencies(${PROJECT_NAME} surgical_robot_generate_messages_cpp)