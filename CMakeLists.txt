cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS -pthread)

find_package(Boost COMPONENTS REQUIRED)

project(robocar_drivehub)

file(GLOB HEADER include/*.hpp include/*.h)
file(GLOB SOURCES src/*.cpp src/*.c)

include_directories( ${Boost_INCLUDE_DIRS} )

add_executable(robocar_drivehub
    ${SOURCES}
    ${HEADER})
target_link_libraries(robocar_drivehub ${Boost_LIBRARIES})
target_include_directories(robocar_drivehub PRIVATE include/ )



