cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp)
target_include_directories(Rasterizer PUBLIC /usr/local/include)
target_link_libraries(Rasterizer ${OpenCV_LIBS})