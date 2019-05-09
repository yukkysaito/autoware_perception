cmake_minimum_required(VERSION 2.8.3)
project(sensor_launch)

find_package(catkin REQUIRED
             COMPONENTS)

catkin_package(# INCLUDE_DIRS include
               # LIBRARIES perception_launch
               CATKIN_DEPENDS
               # DEPENDS system_lib
               )
include_directories(# include
                    ${catkin_INCLUDE_DIRS})

install(FILES launch/sensor.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
