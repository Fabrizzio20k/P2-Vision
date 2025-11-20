#!/bin/bash

pkill -f lite.launch
killall -9 component_container
pkill -9 -f depthai
pkill -9 -f oakd
killall -9 ros2
pkill -9 -f rplidar

sleep 3

. /opt/ros/jazzy/setup.bash

ros2 launch turtlebot4_bringup lite.launch.py &

timeout=60
while [[ $timeout -gt 0 ]]; do
    if ros2 topic list | grep -q '/oakd/rgb/preview/image_raw'; then
        echo "Cámara lista"
        break
    fi
    sleep 1
    ((timeout--))
done

ros2 run rplidar_ros rplidar_composition &

ros2 service wait /oakd/start_camera --timeout 30

ros2 service call /oakd/start_camera std_srvs/srv/Trigger

sleep 2

python3 /home/ubuntu/sender.py
