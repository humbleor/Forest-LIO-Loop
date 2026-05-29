#!/bin/bash

WORKSPACE="/home/mspace/code/Forest_LIO_LOOP_ws"
BAG_DIR="/home/mspace/bagfiles"
BAG_FILE="MulCapture-2025-04-04-14-49-52-one.bag"

gnome-terminal \
    --window --title="fast_lio" --command="bash -c 'cd $WORKSPACE; source devel/setup.bash; sleep 1; roslaunch fast_lio mapping_ouster64.launch; exec bash'" \
    --tab --title="loop_detector" --command="bash -c 'cd $WORKSPACE; source devel/setup.bash; sleep 3; roslaunch forest_loop_detector loop_detector.launch; exec bash'" \
    --tab --title="online_trunk_test" --command="bash -c 'cd $WORKSPACE; source devel/setup.bash; sleep 5; roslaunch forest_loop_detector online_trunk_test.launch; exec bash'" \
    --tab --title="rosbag_play" --command="bash -c 'cd $BAG_DIR; sleep 7; rosbag play $BAG_FILE --topics /os_cloud_node_1/imu /os_cloud_node_1/points; exec bash'"