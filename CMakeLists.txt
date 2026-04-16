cmake_minimum_required(VERSION 3.10)
project(Pathfinder)

set(CMAKE_CXX_STANDARD 17)

find_package(CGAL REQUIRED)
add_executable(pathfinder src/main.cpp
)

target_link_libraries(pathfinder CGAL::CGAL)
target_include_directories(pathfinder PRIVATE ${CMAKE_SOURCE_DIR})