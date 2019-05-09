cmake_minimum_required(VERSION 2.8.3)
project(dynamic_object_visualization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
             COMPONENTS roscpp
                        autoware_msgs
                        geometry_msgs
                        sensor_msgs
                        visualization_msgs)

catkin_package(INCLUDE_DIRS include
               # LIBRARIES dynamic_object_visualizer
               CATKIN_DEPENDS
               roscpp
               autoware_msgs
               geometry_msgs
               sensor_msgs
               visualization_msgs
               # DEPENDS system_lib
               )

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(dynamic_object_visualizer_node src/dynamic_object_visualizer.cpp
               src/main.cpp)

add_dependencies(dynamic_object_visualizer_node
                 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(dynamic_object_visualizer_node ${catkin_LIBRARIES})

install(TARGETS dynamic_object_visualizer_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING
        PATTERN "*.launch")

if(CATKIN_ENABLE_TESTING)

endif()
