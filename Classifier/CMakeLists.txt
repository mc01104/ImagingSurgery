cmake_minimum_required(VERSION 2.8)

project (Classifier)

set(CXX g++)
add_definitions(-std=c++11) 

set (OpenCV_DIR "/usr/local/")
find_package( OpenCV REQUIRED )

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../Utilities/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../LieGroup/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../Lib/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../Lib/Eigen/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../BagOfWords/)
include_directories(${OpenCV_INCLUDE_DIRS})


file(GLOB CPP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB CPP_HDRS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)


add_library(${PROJECT_NAME} STATIC ${CPP_SOURCES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} Utilities LieGroup ${OpenCV_LIBS})
