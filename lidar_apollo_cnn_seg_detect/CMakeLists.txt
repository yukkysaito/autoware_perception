cmake_minimum_required(VERSION 2.8.3)
project(lidar_apollo_cnn_seg_detect)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
        # autoware_build_flags
        roscpp
        tf
        tf_conversions
        pcl_ros
        sensor_msgs
        geometry_msgs
        autoware_msgs
        )

find_package(CUDA)
find_package(OpenCV REQUIRED)

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        tf
        tf_conversions
        pcl_ros
        sensor_msgs
        geometry_msgs
        autoware_msgs
)

IF (CUDA_FOUND)
    include_directories(${CUDA_INCLUDE_DIRS})

    IF ("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "^arm")
        link_directories(/usr/lib/arm-linux-gnueabihf/tegra)
    endif ()
ENDIF ()

# Download caffemodel and prototxt
set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
if (NOT EXISTS "${PATH}")
  execute_process(COMMAND mkdir -p ${PATH})
endif()
set(FILE "${PATH}/apollo_cnn.caffemodel")
message(STATUS "Checking and downloading apollo_cnn.caffemodel")
if (NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(COMMAND wget "https://drive.google.com/uc?export=download&id=1Md3qOLcW6nWSLvnnZH3aHEEJPvblCBkY" -O ${PATH}/apollo_cnn.caffemodel)
endif()
set(FILE "${PATH}/apollo_cnn.prototxt")
message(STATUS "Checking and downloading apollo_cnn.prototxt")
if (NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(COMMAND wget "https://drive.google.com/uc?export=download&id=1doEp46CV1uc7851w6iYfuxZiM6Uc0b_N" -O ${PATH}/apollo_cnn.prototxt)
endif()


###CAFFE
set(CAFFE_PATH "/usr/local/caffe")

if (EXISTS "${CAFFE_PATH}")

    include_directories(
            include
            ${catkin_INCLUDE_DIRS}
            ${OpenCV_INCLUDE_DIRS}
            ${CAFFE_PATH}/include
    )

    add_executable(lidar_apollo_cnn_seg_detect
            nodes/lidar_apollo_cnn_seg_detect_node.cpp
            nodes/cnn_segmentation.cpp
            nodes/cluster2d.cpp
            nodes/feature_generator.cpp
            )
    target_link_libraries(lidar_apollo_cnn_seg_detect
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            ${CAFFE_PATH}/lib/libcaffe.so
            glog
            )
    add_dependencies(lidar_apollo_cnn_seg_detect
            ${catkin_EXPORTED_TARGETS}
            )

    install(TARGETS
            lidar_apollo_cnn_seg_detect
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
            )
    install(DIRECTORY launch/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
            PATTERN ".svn" EXCLUDE)
else ()
    message("'Caffe' is not installed. 'lidar_apollo_cnn_seg_detect' will not be built.")
endif ()