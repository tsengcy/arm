cmake_minimum_required(VERSION 3.10)

project(robotics VERSION 1.0)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
)


add_definitions(-DDEBUG)

find_package(Eigen3)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(tmpf SHARED
    include/tmp/tmp.hpp
    src/tmp/tmp.cpp
)

add_library(robotics SHARED
    include/robotics/robotarm.hpp
    src/robotics/robotarm.cpp
    include/robotics/endeffector.hpp
    src/robotics/endeffector.cpp
    include/robotics/frame.hpp
    src/robotics/frame.cpp
    include/robotics/dynframe.hpp
    src/robotics/dynframe.cpp
    include/robotics/dynrobotarm.hpp
    src/robotics/dynrobotarm.cpp
    include/robotics/utils.hpp
    src/robotics/utils.cpp
)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)



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

add_executable(test
    src/test.cpp
)
target_link_libraries(test
    robotics
)

add_executable(main2
    src/main2.cpp
)
target_link_libraries(main2
    robotics
)

add_executable(main3
    src/main3.cpp
)
target_link_libraries(main3
    robotics
)

add_executable(main4
    src/main4.cpp
)
target_link_libraries(main4
    robotics
)

add_executable(main5
    src/main5.cpp
)
target_link_libraries(main5
    robotics
)

add_executable(main6
    src/main6.cpp
)
target_link_libraries(main6
    robotics
)

add_executable(main7
    src/main7.cpp
)
target_link_libraries(main7
    robotics
)
