##########################################
# Top-level CMakeLists.txt for InfiniTAM #
##########################################

cmake_minimum_required(VERSION 2.8)

PROJECT(InfiniTAM)

#################################
# Specify the CMake module path #
#################################

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

#################################
# Add additional compiler flags #
#################################

IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
ENDIF()

IF(NOT MSVC_IDE)
  SET(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
  SET(CMAKE_CXX_FLAGS "-fPIC -O3 -march=native ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
  #SET(CMAKE_CXX_FLAGS "-fPIC -g ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
ENDIF()

#########################
# Suppress auto-linking #
#########################

IF(MSVC_IDE)
  ADD_DEFINITIONS(-DUSING_CMAKE=1)
ENDIF()




##########################################
# 齐龙:VO 依赖#
##########################################
# 增加PCL库的依赖
# FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters )
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

# 增加opencv的依赖
# find openCv
find_package (OpenCV REQUIRED)
#set(OpenCV_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS} /usr/local/include/opencv4/)
include_directories ("/usr/local/include/opencv4/")
MESSAGE("OpenCV_INCLUDE_DIRS:::::::::::" ${OpenCV_INCLUDE_DIRS})
MESSAGE("OpenCV_LIBS::::::::::::" ${OpenCV_LIBS})




######################
# Add subdirectories #
######################
ADD_SUBDIRECTORY(Apps)
ADD_SUBDIRECTORY(FernRelocLib)
ADD_SUBDIRECTORY(InputSource)
ADD_SUBDIRECTORY(ITMLib)
ADD_SUBDIRECTORY(MiniSlamGraphLib)
ADD_SUBDIRECTORY(ORUtils)


