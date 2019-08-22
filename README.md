# new_perception

## How to use
1. start rosbag
2. pause rosbag
3. load pointcloud map, vector map, world map tf, vehicle tf
4. launch ndt matching
5. camera lidar calibration publisher
6. run perception 
 - camera lidar base
   ```roslaunch perception_launch perception.launch mode:="camera_lidar_fusion"```
 - only lidar
  ```roslaunch perception_launch perception.launch mode:="lidar"```
 - only camera (currently not implemented)
  ```roslaunch perception_launch perception.launch mode:="camera"```

## How to use TensorRT YOLOv3
2019/05/01
when run catkin_make, automaticaly download caffemodel and prototxt. but you need to install tensorrt.
## How to use TensorRT baidu_cnn_seg
2019/08/21
when run catkin_make, automaticaly download caffemodel and prototxt. but you need to install caffe.
