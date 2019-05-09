cmake_minimum_required(VERSION 2.8.3)
project(perception_launch)

find_package(catkin REQUIRED
             COMPONENTS dynamic_object_visualization
                        euclidean_cluster
                        pointcloud_to_laserscan
                        shape_estimation)

catkin_package(# INCLUDE_DIRS include
               # LIBRARIES perception_launch
               CATKIN_DEPENDS
               dynamic_object_visualization
               euclidean_cluster
               pointcloud_to_laserscan
               shape_estimation
               # DEPENDS system_lib
               )
include_directories(# include
                    ${catkin_INCLUDE_DIRS})

install(FILES launch/perception.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
