#!/bin/bash

WORKSPACE="/home/mspace/code/Forest_LIO_LOOP_ws"
# BAG_DIR="/home/mspace/bagfiles"
BAG_DIR="/home/mspace/bagfiles/test"
# BAG_FILE="MulCapture-2025-04-04-14-49-52-one.bag"
BAG_FILE="1708347681_2024-02-19-13-07-08_1-002.bag"

gnome-terminal \
    --window --title="fast_lio" --command="bash -c 'cd $WORKSPACE; source devel/setup.bash; sleep 1; roslaunch fast_lio mapping_ouster64.launch; exec bash'" \
    --tab --title="loop_detector" --command="bash -c 'cd $WORKSPACE; source devel/setup.bash; sleep 3; roslaunch forest_loop_detector loop_detector.launch 2>&1 | tee -a $WORKSPACE/src/forest_loop_detector/loop_detector.log; exec bash'" \
    --tab --title="rosbag_play" --command="bash -c 'cd $BAG_DIR; sleep 7; rosbag play $BAG_FILE --topics /hesai/pandar /alphasense_driver_ros/imu; exec bash'"
