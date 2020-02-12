cmake_minimum_required(VERSION 3.15)
project(MobileRobot)

set(CMAKE_CXX_STANDARD 14)

find_package (Eigen3  REQUIRED)
find_package(Python2 COMPONENTS Development NumPy)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories( "/usr/include/eigen3" )

add_executable(MobileRobot main.cpp)

target_include_directories(MobileRobot PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
target_link_libraries(MobileRobot Python2::Python Python2::NumPy ${OpenCV_LIBS})

