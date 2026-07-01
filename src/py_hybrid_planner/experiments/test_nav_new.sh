#!/bin/bash
# Integration test on maps2: corner (0.8, 0.8) -> opposite (5.5, 2.7).
source /opt/ros/noetic/setup.bash
source /home/cjx/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger

cleanup() {
    echo "=== cleanup ==="
    /usr/bin/pkill -9 -f gzserver 2>/dev/null
    /usr/bin/pkill -9 -f gzclient 2>/dev/null
    /usr/bin/pkill -9 -f amcl 2>/dev/null
    /usr/bin/pkill -9 -f roslaunch 2>/dev/null
    /usr/bin/pkill -9 -f rosmaster 2>/dev/null
    sleep 2
}
trap cleanup EXIT

echo "=== starting Gazebo ==="
roslaunch py_hybrid_planner indoor_gazebo.launch gui:=false > /tmp/gazebo.log 2>&1 &
sleep 25

echo "=== starting nav ==="
roslaunch py_hybrid_planner hybrid_nav.launch open_rviz:=false > /tmp/nav.log 2>&1 &
sleep 25

echo "=== wait until TF ready ==="
for i in $(seq 1 20); do
    if timeout 1 rosrun tf tf_echo map base_footprint 2>&1 | grep -q Translation; then
        echo "TF ready after ${i}s"
        break
    fi
    sleep 1
done

echo "=== initial pose ==="
timeout 2 rosrun tf tf_echo map base_footprint 2>&1 | grep -E 'Translation|RPY .degree' | tail -3

echo "=== publish goal (5.5, 2.7) - opposite corner ==="
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 5.5, y: 2.7, z: 0.0}, orientation: {w: 1.0}}}" \
    > /dev/null 2>&1

echo "=== sampling pose every 3s for 90s ==="
for i in $(seq 1 30); do
    t=$((i*3))
    pos=$(timeout 2 rosrun tf tf_echo map base_footprint 2>&1 | grep Translation | tail -1)
    echo "  t=${t}s  ${pos}"
    sleep 3
done

echo "=== nav.log key events ==="
grep -iE 'planning|reached|stuck|recovery|emergency|fail|goal|no_prog|oscill' /tmp/nav.log | tail -40

echo "=== final pose ==="
timeout 2 rosrun tf tf_echo map base_footprint 2>&1 | grep Translation | tail -1
