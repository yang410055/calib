
cmake_minimum_required(VERSION 3.2)
project(basic_calib_project)
#set(SRC “D:\SRC_TestConfig\VS_C\opencv_cmake_Pconfig”)
#set(SRC_LIST test1.cpp)
message(STATUS "this is a basic calibratoin project")

#####定义了工程，得到两个自定义的文件夹 工程名_BINARY_DIR(cmake中build文件夹) 工程名_SOURCE_DIR（cmake中src文件夹）
#######                       也可以写为PROJECT_BINARY_DIR 和 PROJECT_SOURCE_DIR

message(STATUS ${PROJECT_BINARY_DIR})
message(STATUS ${PROJECT_SOURCE_DIR})

######设置为debug版本
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-O0 -g")
#set(CMAKE_CXX_FLAGS_DEBUG “-O0 -Wall -g2 -ggdb”)


###set(CMAKE_M_PATH D:/Program Files/opencv/opencv/build)
###message(STATUS ${CMAKE_M_PATH})
##list(APPEND CMAKE_INCLUDE_PATH D:/Program Files/opencv/opencv/build)
message(STATUS -------${CMAKE_MODULE_PATH}-------------)
list(APPEND CMAKE_MODULE_PATH D:/Program Files/opencv/opencv/build/x64/vc14/lib)
####message(STATUS ${CMAKE_MODULE_PATH} -----------)
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )


find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
message(${OpenCV_INCLUDE_DIRS})
######link_directories()
######target_link_libraries(opencv_config ${Opencv_libraries})

#add_executable(opencv_config ${SRC_LIST})
add_subdirectory(src bin)