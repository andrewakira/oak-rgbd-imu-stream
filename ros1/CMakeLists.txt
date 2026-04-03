cmake_minimum_required(VERSION 3.0.2)
project(oak_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -w   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w   -O3")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    cv_bridge
)
find_package(OpenCV REQUIRED)
find_package(depthai CONFIG REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../src
)


add_library(depthai_sensor
    ${CMAKE_SOURCE_DIR}/../src/DepthAISensor.cpp
)

target_link_libraries(depthai_sensor
    ${OpenCV_LIBS}
    depthai::opencv
)

add_executable(oak_publisher_node
    src/publisher.cpp
)

target_link_libraries(oak_publisher_node
    depthai_sensor
    ${catkin_LIBRARIES}
)