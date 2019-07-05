# new_perception

## Prerequisite
1. Download [caffemodel](https://drive.google.com/file/d/1jEMkhuQmpUAc1rMvFdkggULHwQwzONer/view?usp=sharing) and [prototxt](https://drive.google.com/file/d/1XLW0dlic8rzhvMbzPh8co3AnmcjAyq9h/view?usp=sharing) into `tensorrt_yolo3/data/yolov3_416.caffemodel`/`tensorrt_yolo3/data/yolov3_416_trt.prototxt`.

## How to use
1. start rosbag
2. pause rosbag
3. load pointcloud map, vector map, world map tf, vehicle tf
5. launch ndt matching
7. camera lidar calibration publisher
8. `roslaunch perception_launch perception.launch`



## How to use PointPillars + Multi Object Tracker
1. Download [PointPillars](https://github.com/k0suke-murakami/kitti_pretrained_point_pillars) file into `lidar_point_pillars/data/`.
2. `roslaunch perception_launch perception_with_pointpillars.launch`

## How to use baidu_cnn_seg + shape_estiomation + object_tracker + predictrion
1. Download [caffemodel](https://drive.google.com/file/d/1Md3qOLcW6nWSLvnnZH3aHEEJPvblCBkY/view?usp=sharing) and [prototxt](https://drive.google.com/file/d/1doEp46CV1uc7851w6iYfuxZiM6Uc0b_N/view?usp=sharing) into `lidar_apollo_cnn_seg/data/apollo_cnn.caffemodel`/`lidar_apollo_cnn_seg/data/apollo_cnn.prototxt`.
2. `roslaunch perception_launch perception_baidu_seg.launch`
