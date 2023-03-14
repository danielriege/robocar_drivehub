cmake_minimum_required(VERSION 3.1)

project(robocar_drivehub LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS -pthread)

find_package(Boost COMPONENTS thread REQUIRED)

file(GLOB HEADER include/*.hpp include/*.h inluce/states/*.hpp) 
file(GLOB SOURCES src/*.cpp src/*.c src/states/*.cpp)

add_executable(robocar_drivehub
    ${SOURCES}
    ${HEADER})
target_link_libraries(robocar_drivehub Boost::thread swiftrobotc pigpio)
target_include_directories(robocar_drivehub PRIVATE include/ )


install(TARGETS robocar_drivehub DESTINATION bin)
