cmake_minimum_required(VERSION 2.8)
project(serial_node)

SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_MULTITHREADED ON)
SET(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.45.0 COMPONENTS bind ref)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()

include_directories(Peripherals/include)
file(GLOB SOURCES "Peripherals/src/*.cpp" "serial_node.cpp")

add_executable(serial_node  ${SOURCES})
find_package(glog 0.3.5 REQUIRED)
target_link_libraries(serial_node ${catkin_LIBRARIES} glog::glog)
if(Boost_FOUND)
    target_link_libraries(serial_node ${Boost_LIBRARIES})
endif()
add_dependencies(serial_node surgical_robot_generate_messages_cpp)

