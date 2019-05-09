cmake_minimum_required(VERSION 2.8.3)
project(map_based_prediction)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        autoware_msgs
        pcl_ros
        roscpp
        tf2
        tf2_geometry_msgs
        rostest
        rosunit
        )

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS
            include
        CATKIN_DEPENDS
            autoware_msgs
            pcl_ros
            roscpp
            tf2
            tf2_geometry_msgs
        )

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        )

add_executable(map_based_prediction
        src/map_based_prediction_main.cpp
        src/map_based_prediction_ros.cpp
        src/vectormap_ros.cpp
        src/map_based_prediction.cpp
        )

add_dependencies(map_based_prediction
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(map_based_prediction
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        )

install(TARGETS map_based_prediction
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
        )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_shape_estimation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
# if (CATKIN_ENABLE_TESTING)
#     find_package(rostest REQUIRED)
#     add_rostest_gtest(test-lidar_shape_estimation 
#     test/test_lidar_shape_estimation.test 
#     test/src/test_lidar_shape_estimation.cpp
#     src/shape_estimator.cpp
#     src/model/bounding_box.cpp
#     src/model/convex_hull.cpp
#     src/model/cylinder.cpp)
#     target_link_libraries(test-lidar_shape_estimation 
#     ${catkin_LIBRARIES}
#     ${OpenCV_LIBS}
#     ${PCL_LIBRARIES})
# endif ()
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
