cmake_minimum_required(VERSION 2.8.3)
project(ProbabilisticPlaneRefinement)

set(CMAKE_CXX_FLAGS "-O4 -g -pg -Wunknown-pragmas -Wno-unknown-pragmas -Wsign-compare -fPIC -fopenmp")

# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#OPENCV
link_directories(/opt/ros/hydro/lib)
include_directories(/opt/ros/hydro/include)
include_directories(/opt/ros/hydro/include/opencv)
include_directories(/opt/ros/hydro/include/opencv2)

include_directories(src)
include_directories(src/util)
include_directories(src/mygeometry)
include_directories(src/SurfaceInference)
include_directories(src/SegmentModel)
include_directories(src/SegmentModelFactory)
include_directories(src/Refinement)
include_directories(src/examples)

add_library(ppr_util src/util/util.cpp)

add_library(ppr_geometry src/pprgeometry/Surface.cpp src/pprgeometry/Plane.cpp)
target_link_libraries(ppr_geometry ppr_util ${PCL_LIBRARIES})

add_library(ppr_SurfaceInference src/SurfaceInference/SurfaceInference.cpp)
target_link_libraries(ppr_SurfaceInference ppr_util)

add_library(ppr_SegmentModel src/SegmentModel/SegmentModel.cpp src/SegmentModel/MultiTypeSegmentModel.cpp src/SegmentModel/GaussiansSegmentModel.cpp src/SegmentModel/ColorHistogramSegmentModel.cpp)
target_link_libraries(ppr_SegmentModel ppr_util)

add_library(ppr_SegmentModelFactory src/SegmentModelFactory/SegmentModelFactory.cpp src/SegmentModelFactory/AdaptiveSingleGaussiansSegmentModelFactory.cpp src/SegmentModelFactory/RGBDSegmentModelFactory.cpp src/SegmentModelFactory/ColorHistogramSegmentModelFactory.cpp)
target_link_libraries(ppr_SegmentModelFactory ppr_util ppr_SegmentModel opencv_calib3d opencv_gpu opencv_nonfree opencv_ts opencv_contrib opencv_highgui opencv_objdetect opencv_video opencv_core opencv_imgproc opencv_photo opencv_videostab opencv_features2d opencv_legacy opencv_stitching opencv_flann opencv_ml opencv_superres)

add_library(ppr_SurfaceRefinement src/Refinement/SurfaceRefinement.cpp )
target_link_libraries(ppr_SurfaceRefinement ppr_geometry ppr_util ppr_SurfaceInference ppr_SegmentModelFactory opencv_calib3d opencv_gpu opencv_nonfree opencv_ts opencv_contrib opencv_highgui opencv_objdetect opencv_video opencv_core opencv_imgproc opencv_photo opencv_videostab opencv_features2d opencv_legacy opencv_stitching opencv_flann opencv_ml opencv_superres ${PCL_LIBRARIES})

add_executable(ppr_pcd_tester src/examples/pcd_tester.cpp)
target_link_libraries(ppr_pcd_tester ppr_SurfaceRefinement ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

