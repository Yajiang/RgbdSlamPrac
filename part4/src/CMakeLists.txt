CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
#增加一个可执行的二进制
set(OpenCV_FOUND 1)

FIND_PACKAGE(OpenCV 2.4.13 REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

FIND_PACKAGE(PCL REQUIRED COMPONENTS common io)
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
add_library(join_point_cloud join_point_cloud.cpp)
target_link_libraries(join_point_cloud
slam_estimate
${OpenCV_LIBS}
${PCL_LIBRARIES}
)
add_executable(visual_odometry visual_odometry.cpp)
target_link_libraries(visual_odometry
${OpenCV_LIBS}
${PCL_LIBRARIES}
)