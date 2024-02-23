cmake_minimum_required(VERSION 2.8)

find_library(WIRINGPI_LIB wiringPi)
find_package(CURL REQUIRED)

set(SOURCES 
	calDriven.cpp
)
# 실행 파일 생성
add_executable(drivenInfo ${SOURCES})
include_directories("/home/pi/Documents/mobis_hackathon/DrivenDist/json/include")

# wiringPi 라이브러리 링크
target_link_libraries(drivenInfo ${WIRINGPI_LIB} ${CURL_LIBRARIES} m)
