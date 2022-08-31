cmake_minimum_required(VERSION 3.21.1)
project(main.cpp)

set(CMAKE_CXX_STANDARD 14)

add_executable(CameraCalibration main.cpp Chess.h Chess.cpp)

set(OpenCV_DIR C:/IDE/OpenCV/OpenCV544/opencv/buildforclion/install)
find_package(OpenCV REQUIRED)
target_link_libraries(CameraCalibration ${OpenCV_LIBS})