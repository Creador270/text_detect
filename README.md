# Tex Tracker pkg for ros2

## Overview
A simple package for tracking text in various environmental conditions using [EasyOCR](https://github.com/JaidedAI/EasyOCR) iIt is designed to work in challenging scenarios such as high reflectivity, using two-stage kernel filtering and a NOT operation to enhance contrast.
## Instalation
**1.** Clone the repository into your current ROS2 workspace:
```bash
cd <ros2_ws>/src/
git clone https://github.com/Creador270/text_detect.git
```
**2.** build the package from your current ros2 workspace using colcon
```bash
cd <ros2_ws>/
colcon build --symlink-install --packages-select tex_detect
```
## Usage
Source your workspace and run the node:
 ```bash
ros2 run text_detect text_detect
```
The node publishes the detected bounding boxes around the text and the corresponding coordinates as an image topic using [cv_bridge](https://github.com/ros-perception/vision_opencv.git)
## Topics
* **Published topics**
    * `text_detect/image_out` – Image topic with detected text in bounding box
    * `text_detect/coordinates` - Coordinates to the Image on a list Int16MultiArray topic

## Dependencies
* EasyOCR >=1.7.2
* OpencCV >= 4.7
* cv_bridge
## Contributions
Feel free to contribute however you think is best—any commentary is welcome and interesting honestly, this is just a project I started because I was feeling bored.
Feel free to fork, raise an issue, or open a pull request!
