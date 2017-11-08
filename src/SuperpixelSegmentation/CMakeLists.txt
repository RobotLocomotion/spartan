cmake_minimum_required(VERSION 3.5)
project(SuperpixelSegmentation)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIR})

# Taken from https://github.com/michaelchughes/KMeansRex/tree/master/
# but building here because their build system is weaker than ours
add_library(KMeansRexCore KMeansRexCore.cpp KMeansRexCoreInterface.h mersenneTwister2002.c)
target_link_libraries(KMeansRexCore
    ${Eigen3_LIBRARIES})
install(TARGETS KMeansRexCore DESTINATION lib)

find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_library(CVUTILS_LIBRARY cv-utils DOC "The cv-utils library")
find_path(CVUTILS_INCLUDE_DIR image_utils/jpeg.h PATH_SUFFIXES cv-utils DOC "Path to the cv-utils include directory")
set(CVUTILS_INCLUDE_DIRS ${CVUTILS_INCLUDE_DIR})
set(CVUTILS_LIBRARIES ${CVUTILS_LIBRARY})
include_directories(${CVUTILS_INCLUDE_DIRS})

find_package(drake REQUIRED)

add_subdirectory(ORUtils)
set(GSLICR_LIB
gSLICr_Lib/engines/gSLICr_core_engine.h
gSLICr_Lib/engines/gSLICr_seg_engine.h
gSLICr_Lib/engines/gSLICr_seg_engine_GPU.h
gSLICr_Lib/engines/gSLICr_seg_engine_shared.h
gSLICr_Lib/engines/gSLICr_core_engine.cpp
gSLICr_Lib/engines/gSLICr_seg_engine.cpp
gSLICr_Lib/engines/gSLICr_seg_engine_GPU.cu
gSLICr_Lib/objects/gSLICr_settings.h
gSLICr_Lib/objects/gSLICr_spixel_info.h
gSLICr_Lib/gSLICr_defines.h
gSLICr_Lib/gSLICr.h
)

cuda_add_library(gSLICr_lib
		${GSLICR_LIB}
		NVTimer.h
		OPTIONS -gencode arch=compute_30,code=compute_30)
target_link_libraries(gSLICr_lib ${CUDA_LIBRARY})
install(TARGETS gSLICr_lib DESTINATION lib)

# There must be a better way... gotta figure out
# how to make find_package work for common utils
include_directories(${CMAKE_PREFIX_PATH}/include)
link_directories(${CMAKE_PREFIX_PATH}/lib)

add_executable (superpixel_segmentation superpixel_segmentation.cc)
target_link_libraries (superpixel_segmentation 
	gSLICr_lib
    ${drake_LIBRARIES}
	${OpenCV_LIBS}
	${CVUTILS_LIBRARIES}
	rgbd_camera_driver
	KMeansRexCore)
install(TARGETS superpixel_segmentation DESTINATION bin)