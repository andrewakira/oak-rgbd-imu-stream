cmake_minimum_required(VERSION 3.16)
project(oak_stream)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -w   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w   -O3")

find_package(OpenCV REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(Threads REQUIRED)

add_executable(oak_stream src/stereo_inertial_publisher.cpp)

target_link_libraries(oak_stream PRIVATE
    depthai::opencv
    ${OpenCV_LIBS}
    Threads::Threads
)

target_include_directories(oak_stream PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)