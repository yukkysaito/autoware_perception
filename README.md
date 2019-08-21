# new_perception

## Prerequisite
1. Download [caffemodel](https://drive.google.com/file/d/1jEMkhuQmpUAc1rMvFdkggULHwQwzONer/view?usp=sharing) and [prototxt](https://drive.google.com/file/d/1XLW0dlic8rzhvMbzPh8co3AnmcjAyq9h/view?usp=sharing) into `tensorrt_yolo3/data/yolov3_416.caffemodel`/`tensorrt_yolo3/data/yolov3_416_trt.prototxt`.

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

## How to use baidu_cnn_seg + shape_estiomation + object_tracker + predictrion
1. Download [caffemodel](https://drive.google.com/file/d/1Md3qOLcW6nWSLvnnZH3aHEEJPvblCBkY/view?usp=sharing) and [prototxt](https://drive.google.com/file/d/1doEp46CV1uc7851w6iYfuxZiM6Uc0b_N/view?usp=sharing) into `lidar_apollo_cnn_seg/data/apollo_cnn.caffemodel`/`lidar_apollo_cnn_seg/data/apollo_cnn.prototxt`.
2. Load vectormap
3. `roslaunch perception_launch perception_baidu_seg.launch`
