#!/bin/bash
echo "暂停仿真..."
rosservice call /gazebo/pause_physics

echo "复位世界..."
rosservice call /gazebo/reset_world


echo "恢复仿真..."
rosservice call /gazebo/unpause_physics

echo "复位里程计"
rostopic pub -1 /controller/hero_chassis_controller/debug std_msgs/Float64MultiArray "data: [1.0, 0.0, 0.0]"

echo "完成"