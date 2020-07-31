CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
PROJECT(ZED_Camera)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)
set(CMAKE_CXX_STANDARD 14)

set(WINDOWS TRUE CACHE INTERNAL "")
set(PLATFORM_PREFIX "win64" CACHE INTERNAL "")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

IF (WIN32)
	MESSAGE(STATUS "Now is windows")
    if (CMAKE_HOST_WIN32)
    set(HOME_DIR $ENV{HOMEDRIVE}$ENV{HOMEPATH} CACHE INTERNAL "")
    else()
        set(HOME_DIR $ENV{HOME} CACHE INTERNAL "")
    endif()
    if ("TEST$ENV{MAKE_DEPS_WORKING_DIR}" STREQUAL "TEST")
        set(MAKE_DEPS_WORKING_DIR ${HOME_DIR}/makedeps)
    else()    
        set(MAKE_DEPS_WORKING_DIR $ENV{MAKE_DEPS_WORKING_DIR})	
    endif()

    if (NOT ROOT_DIR_DEFINED)
        add_definitions(-DROOT_DIR="${PROJECT_SOURCE_DIR}")
        set(ROOT_DIR_DEFINED ON)
    endif()

    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

    include(${MAKE_DEPS_WORKING_DIR}/${PLATFORM_PREFIX}/deps.cmake)
ELSEIF (APPLE)
	MESSAGE(STATUS "Now is Apple systens.")
ELSEIF (UNIX)
	MESSAGE(STATUS "Now is UNIX-like OS's.")
ENDIF ()




if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)


if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

if (NOT ROOT_DIR_DEFINED)
    add_definitions(-DROOT_DIR="${PROJECT_SOURCE_DIR}")
    set(ROOT_DIR_DEFINED ON)
endif()

SET(EXECUTABLE_OUTPUT_PATH ".")

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ZED 3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

if(PCL_FOUND)
    message("-------------------------PCL_FOUND------")
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
 endif()


include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

file(GLOB IMG_SRC_FILES src/main.cpp 
					src/point_cloud_unity.cpp 
					src/zed_unity.cpp
					src/zed_unity.hpp
					src/point_cloud_unity.hpp
					)
					
file(GLOB VIDEO_SRC_FILES src/recording_video_main.cpp 
					src/point_cloud_unity.cpp 
					src/zed_unity.cpp
					src/zed_unity.hpp
					src/point_cloud_unity.hpp
					)

ADD_EXECUTABLE(${PROJECT_NAME}  ${IMG_SRC_FILES})
ADD_EXECUTABLE(RecordingVideo  ${VIDEO_SRC_FILES})
add_definitions(-std=c++14 -O3)

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${ZED_LIBS} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
TARGET_LINK_LIBRARIES(RecordingVideo ${ZED_LIBS} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
