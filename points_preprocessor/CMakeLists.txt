cmake_minimum_required(VERSION 2.8.3)
project(points_preprocessor)

add_compile_options(-std=c++11 -O3)

find_package(catkin REQUIRED
             COMPONENTS roscpp
                        sensor_msgs
                        pcl_ros
                        nodelet)

generate_dynamic_reconfigure_options(cfg/CompareMapFilter.cfg)

catkin_package(INCLUDE_DIRS
               include
               LIBRARIES
               compare_map_filter
               CATKIN_DEPENDS
               roscpp
               sensor_msgs
               pcl_ros
               nodelet
               DEPENDS
               Boost
               EIGEN3
               PCL)

# Resolve system dependency on yaml-cpp, which apparently does not provide a
# CMake find_package() module.

#
# Build ##
#

include_directories(${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    include)

# Compare Map Filter
add_library(compare_map_filter
            src/compare_map_filter/distance_based_compare_map_filter_nodelet.cpp
            src/compare_map_filter/voxel_based_approximate_compare_map_filter_nodelet.cpp
            src/compare_map_filter/voxel_distance_based_compare_map_filter_nodelet.cpp
            src/compare_map_filter/voxel_based_compare_map_filter_nodelet.cpp
            src/filter.cpp # copy from pcl_ros https://github.com/ros-
                           # perception/perception_pcl/issues/9
            )
target_link_libraries(compare_map_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(compare_map_filter ${catkin_EXPORTED_TARGETS})

add_executable(
  distance_based_compare_map_filter_node
  src/compare_map_filter/distance_based_compare_map_filter_node.cpp)
target_link_libraries(distance_based_compare_map_filter_node
                      compare_map_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(distance_based_compare_map_filter_node
                 ${catkin_EXPORTED_TARGETS})
add_executable(voxel_based_approximate_compare_map_filter_node
               src/compare_map_filter/voxel_based_approximate_compare_map_filter_node.cpp)
target_link_libraries(voxel_based_approximate_compare_map_filter_node
                      compare_map_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(voxel_based_approximate_compare_map_filter_node ${catkin_EXPORTED_TARGETS})
add_executable(voxel_distance_based_compare_map_filter_node
               src/compare_map_filter/voxel_distance_based_compare_map_filter_node.cpp)
target_link_libraries(voxel_distance_based_compare_map_filter_node
                      compare_map_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(voxel_distance_based_compare_map_filter_node ${catkin_EXPORTED_TARGETS})
add_executable(voxel_based_compare_map_filter_node
               src/compare_map_filter/voxel_based_compare_map_filter_node.cpp)
target_link_libraries(voxel_based_compare_map_filter_node
                      compare_map_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(voxel_based_compare_map_filter_node ${catkin_EXPORTED_TARGETS})


# Unit Tests ### if (CATKIN_ENABLE_TESTING) find_package(rostest REQUIRED)
# find_package(roslaunch REQUIRED)

# add_rostest_gtest(test_points_preprocessor test/test_points_preprocessor.test
# test/src/test_points_preprocessor.cpp)
# target_include_directories(test_points_preprocessor PRIVATE
# nodes/ray_ground_filter/include test/include)
# target_link_libraries(test_points_preprocessor ray_ground_filter_lib
# ${catkin_LIBRARIES}) endif ()

# install(TARGETS compare_map_filter ARCHIVE DESTINATION
# ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION
# ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION
# ${CATKIN_PACKAGE_BIN_DESTINATION} )

# install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
# PATTERN ".svn" EXCLUDE )
