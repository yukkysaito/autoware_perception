# new_perception

## How to use
1. start rosbag
2. pause rosbag
3. load pointcloud map, vector map, world map tf, vehicle tf
5. launch ndt matching
6. camera lidar calibration publisher
7. run perception 
 - camera lidar base
   ```roslaunch perception_launch perception.launch mode:="camera_lidar_fusion"```
 - only lidar
  ```roslaunch perception_launch perception.launch mode:="lidar"```
 - only camera
  ```roslaunch perception_launch perception.launch mode:="camera"```

## How to use TensorRT YOLOv3
2019/05/01
when run catkin_make, automaticaly download caffemodel and prototxt. but you need to install tensorrt.

1. Download [caffemodel](https://drive.google.com/file/d/1KXsxBURDo72nCEZ5vUlmuneOfSbV4ycE/view?usp=sharing) and [prototxt](https://drive.google.com/file/d/106bRoHNWYmCRXdrd6XRfxl3CNZ8M0BAS/view?usp=sharing) into `tensorrt_yolo3/data/yolov3_416.caffemodel`/`tensorrt_yolo3/data/yolov3_416_trt.prototxt`.
2. `roslaunch tensorrt_yolo3 tensorrt_yolo3.launch`
3. The result is shown in `/perception/tensorrt_yolo3/classified_image` topic.
