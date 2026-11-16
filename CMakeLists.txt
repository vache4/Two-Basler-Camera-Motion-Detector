cmake_minimum_required(VERSION 3.20)
project(DualBaslerMotionCapture LANGUAGES CXX)

find_package(pylon CONFIG REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs highgui)
find_package(Threads REQUIRED)

add_executable(badminton_landing_detector cpp/badminton_landing_detector.cpp)
target_compile_features(badminton_landing_detector PRIVATE cxx_std_17)
target_include_directories(badminton_landing_detector PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(badminton_landing_detector PRIVATE pylon::pylon ${OpenCV_LIBS} Threads::Threads)
if(WIN32)
    target_compile_definitions(badminton_landing_detector PRIVATE NOMINMAX)
endif()
