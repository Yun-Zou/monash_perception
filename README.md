# Monash Perception
This package handles the computer vision tasks and transforming it into data usable for the other modules. It also customises some of the AprilTags settings.

This package uses [AprilTags](https://github.com/AprilRobotics/apriltag) and [apriltags_ros](http://wiki.ros.org/apriltag_ros) to detect tags placed on objects which we are looking for. These then are converted into coordinates in the local frame of the drone for it to track. At the moment, it is only semi-reliable and works up to about 3m maximum, optimally in downwards angle.

AprilTag detection is run on the Intel RealSense T265 Camera image feed which is a grayscale fisheye stereo feed. The fisheye image feed provides a wide FOV but it also is considered 'distorted' and therefore we may want to undistort the image to run apriltag detection if we want the best accuracy, but lessens FOV

The AprilTag used was the Tag16h5 family of AprilTags on an A3 piece of paper with the ID of 0. If you would like to change it, change the tag_family and standalone_tags settings in both settings.yaml and tags.yaml respectively. You can also change some settings in settings.yaml to increase speed of detection at the cost of range/accuracy. Read more in apriltag_ros for info.

There is also a fake_pose node which publishes dummy targets to the topic with random movements which can be used for testing

## How to Run
How do we run the only computer vision package? Type each command in a linux terminal

Without Raw Images
```
roslaunch realsense2_camera rs_t265.launch
roslaunch continous_detection.launch
rosrun monash_perception apriltag_pose
```

With Undistorted Images
```
roslaunch realsense2_camera rs_t265.launch
roslaunch vision_to_mavros t265_fisheye_undistort.launch
roslaunch continous_detection_rect.launch
rosrun monash_perception apriltag_pose
```

Fake Pose
```
rosrun monash_perception fake_pose
```

## Issues

## Future Work
- Convert the detected AprilTag into local frame coordinates properly in found_target() in PerceptionController.cpp
- Find more reliable detection settings
- Use neural networks for detection at high altitudes
- Optimise to use the Graphics card instead of the CPU for these tasks
- Incorporate an additional better camera which can track better
