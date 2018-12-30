project(PoseTracking)

cmake_minimum_required(VERSION 3.5)
IF (WIN32 OR WIN64)
    set(Opencv_DIR F:/opencv310/Release/install)
ENDIF ()
find_package(OpenCV REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()


INCLUDE_DIRECTORIES ( ${OpenCV_INCLUDE_DIRS} )

add_executable(PoseTracking
    lineExtraction.cpp
	lineHiddenRemovelDef.cpp
	LinePoseEstimation.cpp
	lineProjectDef.cpp
	lsd.cpp
	RobustEstimation.cpp
	PoseTracker.cpp
	PoseTracking.cpp
    )

target_link_libraries(PoseTracking ${OpenCV_LIBS})
