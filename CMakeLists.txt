cmake_minimum_required(VERSION 3.10)

project(robotics VERSION 1.0)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
)


find_package(Eigen3)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(tmpf SHARED
    include/tmp/tmp.hpp
    src/tmp/tmp.cpp
)

add_library(robotics SHARED
    include/robotics/frame.hpp
    src/robotics/frame.cpp
)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_definitions(-DDEBUG)

add_executable(tmpmain 
    src/tmp/main.cpp
)

target_link_libraries(tmpmain
    tmpf
)

add_executable(tmp1
    src/tmp/tmp1.cpp
)

add_executable(main
    src/main.cpp
)
target_link_libraries(main
    robotics
)
