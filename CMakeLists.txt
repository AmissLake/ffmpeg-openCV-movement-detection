cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0069 NEW)
project(movement_detection CXX)

include(CheckIncludeFileCXX)
include(CheckIPOSupported)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-fpermissive")
  
message(STATUS "${CMAKE_BUILD_TYPE} profile")
set(CMAKE_CONFIGURATION_TYPES ${CMAKE_BUILD_TYPE} CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

add_executable(${PROJECT_NAME} "main.cpp")

check_ipo_supported(RESULT result)
if(result)
  set_property(TARGET ${PROJECT_NAME} PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

find_library(AVUTIL_LIBRARY avutil)
find_library(AVCODEC_LIBRARY avcodec)
find_library(AVFORMAT_LIBRARY avformat)
find_library(SWSCALE_LIBRARY swscale)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
                   
target_link_libraries(${PROJECT_NAME}
   PRIVATE
   ${AVCODEC_LIBRARY}
   ${AVFORMAT_LIBRARY}
   ${SWSCALE_LIBRARY}
   ${AVUTIL_LIBRARY}
   ${OpenCV_LIBS}
)
