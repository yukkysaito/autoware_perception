cmake_minimum_required(VERSION 2.8.3)
project(euclidean_cluster)

add_compile_options(-std=c++11 -O3)

find_package(catkin REQUIRED
             COMPONENTS pcl_ros
                        roscpp
                        sensor_msgs
                        nodelet
                        pcl_conversions
                        autoware_msgs
                        # tf2
                        # tf2_sensor_msgs
             )
find_package(PCL REQUIRED)

catkin_package(INCLUDE_DIRS
               include
               LIBRARIES
               euclidean_cluster
               CATKIN_DEPENDS
               pcl_ros
               roscpp
               sensor_msgs
               nodelet
               pcl_conversions
               autoware_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIR})

add_library(euclidean_cluster src/euclidean_cluster_nodelet.cpp)
target_link_libraries(euclidean_cluster ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(euclidean_cluster ${catkin_EXPORTED_TARGETS})

add_executable(euclidean_cluster_node src/euclidean_cluster_node.cpp)
target_link_libraries(euclidean_cluster_node
                      euclidean_cluster
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
