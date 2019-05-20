cmake_minimum_required(VERSION 3.14)
project(Eigen_BA)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")

add_executable(Eigen_BA main.cpp Problem.cpp readData.cpp CameraParameterization.cpp ProjectFactor.cpp)
target_link_libraries(Eigen_BA ${OpenCV_LIBRARIES})