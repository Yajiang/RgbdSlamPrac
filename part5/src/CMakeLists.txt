CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
#增加一个可执行的二进制
set(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
SET(G2O_ROOT /usr/local/include/g2o)
FIND_PACKAGE(G2O REQUIRED)
#CSparse
FIND_PACKAGE(CSparse REQUIRED)
INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})

FIND_PACKAGE(OpenCV 2.4.13 REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

FIND_PACKAGE(PCL REQUIRED COMPONENTS common io visualization filters)
ADD_DEFINITIONS(${PCL_DEFINITIONS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_LIBRARIES(${PCL_LIBRARY_DIRS})

add_library(slam_transform slam_transform.cpp)
target_link_libraries(slam_transform
${OpenCV_LIBS}
${PCL_LIBRARIES}
)

add_library(slam_parameters slam_parameters.cpp)
target_link_libraries(slam_parameters
${OpenCV_LIBS}
${PCL_LIBRARIES}
)
add_library(slam_estimate slam_estimate.cpp)
target_link_libraries(slam_estimate
slam_transform
slam_parameters
${OpenCV_LIBS}
${PCL_LIBRARIES}
)
add_library(join_pointcloud join_pointcloud.cpp)
target_link_libraries(join_pointcloud
slam_estimate
slam_transform
slam_parameters
${OpenCV_LIBS}
${PCL_LIBRARIES}
)
add_executable(visual_odometry visual_odometry.cpp)
target_link_libraries(visual_odometry
slam_transform
slam_estimate
slam_parameters
join_pointcloud
${OpenCV_LIBS}
${PCL_LIBRARIES}
g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension 
${CSPARSE_LIBRARY}
)