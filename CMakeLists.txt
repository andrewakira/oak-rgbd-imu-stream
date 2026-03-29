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

# building library
add_library(depthai_sensor
    src/DepthAISensor.cpp
)
target_include_directories(depthai_sensor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(depthai_sensor PUBLIC
    depthai::opencv
    ${OpenCV_LIBS}
    Threads::Threads
)

# building exe
add_executable(oak_stream 
    src/main.cpp
)

target_link_libraries(oak_stream PRIVATE
    depthai_sensor
)