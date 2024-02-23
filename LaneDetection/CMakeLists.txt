cmake_minimum_required(VERSION 3.5)
project(LaneDetection)
set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CURL_INCLUDE_DIRS})
include_directories("/home/pi/Documents/mobis_hackathon/DrivenDist/json/include")

add_executable(LaneDetection main.cpp)


target_link_libraries(LaneDetection ${OpenCV_LIBS} ${CURL_LIBRARIES})

