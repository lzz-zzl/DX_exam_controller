#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from pynput import keyboard

# 速度参数（可以按需调整）
vx = 4   # 前进/后退速度 m/s
vy = 4   # 左移/右移速度 m/s
wz = 20.0   # 旋转速度 rad/s
yaw = 1.0
pitch = 2.0

current_keys = set()

def on_press(key):
    try:
        if key.char in ['w', 's', 'a', 'd', 'q', 'e','t','g','f','h']:
            current_keys.add(key.char)
    except AttributeError:
        pass

def on_release(key):
    try:
        if key.char in current_keys:
            current_keys.remove(key.char)
        # 按 ESC 退出
        if key == keyboard.Key.esc:
            return False
    except AttributeError:
        pass

def main():
    rospy.init_node('keyboard_teleop')
    pub = rospy.Publisher('/standard_controller/cmd_vel', Twist, queue_size=1)
    yaw_pub = rospy.Publisher('/standard_controller/gimbal/yaw_torque', Float64, queue_size=1)
    pitch_pub = rospy.Publisher('/standard_controller/gimbal/pitch_torque', Float64, queue_size=1)
    rate = rospy.Rate(20)  # 20Hz 发布频率

    # 启动键盘监听（非阻塞）
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    twist = Twist()
    yaw_msg = Float64();
    pitch_msg = Float64();
    while not rospy.is_shutdown():
        twist.linear.x = twist.linear.y = twist.angular.z = 0.0
        yaw_msg.data = 0.0
        pitch_msg.data = 0.0

        if 'w' in current_keys: twist.linear.x = vx
        if 's' in current_keys: twist.linear.x = -vx
        if 'a' in current_keys: twist.linear.y = vy
        if 'd' in current_keys: twist.linear.y = -vy
        if 'q' in current_keys: twist.angular.z = wz
        if 'e' in current_keys: twist.aqqeeeeeeeeeeeeeeeeeeeeeeeeeesswswwwswwswsngular.z = -wz
        if 'f' in current_keys: yaw_msg.data = yaw
        if 'h' in current_keys: yaw_msg.data = -yaw
        if 't' in current_keys: pitch_msg.data = pitch
        if 'g' in current_keys: pitch_msg.data = -pitch

        pub.publish(twist)
        yaw_pub.publish(yaw_msg)
        pitch_pub.publish(pitch_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass