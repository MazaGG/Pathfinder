cmake_minimum_required(VERSION 3.10)
project(CMSC190)

set(CMAKE_CXX_STANDARD 17)

find_package(CGAL REQUIRED)
add_executable(test_obstacles
    src/main.cpp
)

target_link_libraries(test_obstacles CGAL::CGAL)
target_include_directories(test_obstacles PRIVATE ${CMAKE_SOURCE_DIR})