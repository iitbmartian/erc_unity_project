#!/bin/bash

source /opt/ros/humble/setup.bash && \
ros2 run tf2_ros static_transform_publisher 0 0 0.2635 0 0 0 1 panther/base_link body_link &
ros2 run tf2_ros static_transform_publisher 0 0 0.3235 0 0 0 1 panther/base_link panther/imu_link &
ros2 run tf2_ros static_transform_publisher 0 0 0.5435 0 0 0 1 panther/base_link laser_link &
ros2 run tf2_ros static_transform_publisher 0.207 0 0.4567 0 0 0 1 panther/base_link panther/camera_front &
ros2 run tf2_ros static_transform_publisher -0.207 0 0.4567 0 0 1 0 panther/base_link panther/camera_back &
ros2 run tf2_ros static_transform_publisher 0 0.207 0.4567 0 0 0.7071 0.7071 panther/base_link panther/camera_left &
ros2 run tf2_ros static_transform_publisher 0 -0.207 0.4567 0 0 -0.7071 0.7071 panther/base_link panther/camera_right &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 panther/camera_front front_camera_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 panther/camera_back back_camera_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 panther/camera_left left_camera_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 panther/camera_right right_camera_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 -0.5 0.5 -0.5 0.5 front_camera_link front_camera_link_optical &
ros2 run tf2_ros static_transform_publisher 0 0 0 -0.5 0.5 -0.5 0.5 back_camera_link back_camera_link_optical &
ros2 run tf2_ros static_transform_publisher 0 0 0 -0.5 0.5 -0.5 0.5 left_camera_link left_camera_link_optical &
ros2 run tf2_ros static_transform_publisher 0 0 0 -0.5 0.5 -0.5 0.5 right_camera_link right_camera_link_optical &
wait
