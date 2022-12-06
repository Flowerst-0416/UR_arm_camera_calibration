#!/usr/bin/env bash

cd ~/Calibration_ws
source devel/setup.bash
echo "[INFO] Calibrating camera-laser extrinsic. "
rosrun multi_calibration Extrinsic_filter.py
rosrun multi_calibration laser_calib -i ~/Calibration_ws/src/UR_arm_camera_calibration/multi_calibration/laser/Target_Image -c ~/Calibration_ws/src/UR_arm_camera_calibration/multi_calibration/cfg/laser_calib.yaml
