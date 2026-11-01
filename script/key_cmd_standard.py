#!/usr/bin/env python3
import rospy
from pip._internal.operations.build import wheel
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from pynput import keyboard

# ============ 速度/目标参数 ============
vx = 4      # 前进/后退速度
vy = 4      # 左移/右移速度
wz = 10.0      # 角速度（rad/s），对应 vel_az_rela

# 云台目标每帧增量（20Hz 下，每秒 = 20 × 这个值）
yaw_step_per_frame   = 0.3
pitch_step_per_frame = 0.1

# ============ PID 增益初值与步长 ============
yaw_Kp = 4
yaw_Ki = 0.0
yaw_Kd = 0.0
gain_step = 1

# ============ 目标值（模块级）============
yaw_target:   float = 0.0
pitch_target: float = 0.0
state:   float = 0.0
debug_state:   float =0.0


current_keys = set()


def on_press(key):
    try:
        if key.char in ['w', 's', 'a', 'd', 'q', 'e',
                        't', 'g', 'f', 'h','y',
                        '7', '4', '1', '8', '5', '2',',','.','0',
                        '[',']','o','p'
                        'c','v','b','n']:
            current_keys.add(key.char)
    except AttributeError:
        pass


temp_state = debug_state
def on_release(key):
    # 只改 PID 值；目标值和角速度交给 main() 循环
    global yaw_Kp, yaw_Ki, yaw_Kd,state,debug_state
    global wheel_Kp,wheel_Ki
    try:
        ch = key.char

        pid_changed = False
        if   ch == '7': yaw_Kp += gain_step; pid_changed = True
        elif ch == '4': yaw_Ki += gain_step; pid_changed = True
        elif ch == '1': yaw_Kd += gain_step; pid_changed = True
        elif ch == '8': yaw_Kp -= gain_step; pid_changed = True
        elif ch == '5': yaw_Ki -= gain_step; pid_changed = True
        elif ch == '2': yaw_Kd -= gain_step; pid_changed = True
        elif ch == 'c': state = 0; pid_changed = True
        elif ch == 'v': state = 1; pid_changed = True
        elif ch == 'b': state = 2; pid_changed = True
        elif ch == 'n': state = 3; pid_changed = True
        elif ch == ',': state = 2; debug_state += 1;pid_changed = True
        elif ch == '.': state = 3; debug_state -= 1;pid_changed = True
        if debug_state>5: debug_state = 1
        if debug_state<1: debug_state = 5

        if ch == '0': state = 3; debug_state = 0;pid_changed = True
        if pid_changed:
            rospy.loginfo("[PID] Kp = %.3f | Ki = %.3f | Kd = %.3f | debug = %.0f",
                          yaw_Kp, yaw_Ki, yaw_Kd,debug_state)

        if ch in current_keys:
            current_keys.remove(ch)
        if key == keyboard.Key.esc:
            return False
    except AttributeError:
        pass
PI = 3.14159265358979323846

def main():
    global yaw_target, pitch_target,state,debug_state

    rospy.init_node('keyboard_teleop')
    pub = rospy.Publisher('/controller/simple_chassis_controller/debug', Float64MultiArray, queue_size=1)
    cmd_vel_pub = rospy.Publisher('/controller/simple_chassis_controller/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(20)

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    msg = Float64MultiArray()
    cmd_vel_msg = Twist()

    rospy.loginfo("[PID] init  Kp = %.3f | Ki = %.3f | Kd = %.3f",
                  yaw_Kp, yaw_Ki, yaw_Kd)
    rospy.loginfo("[TARGET] init  yaw = %.3f | pitch = %.3f",
                  yaw_target, pitch_target)

    while not rospy.is_shutdown():
        vel_x_rela  = 0.0
        vel_y_rela  = 0.0
        vel_az_rela = 0.0
        shoot_state = 0.0

        if 'w' in current_keys: vel_x_rela  =  vx
        if 's' in current_keys: vel_x_rela  = -vx
        if 'a' in current_keys: vel_y_rela  =  vy
        if 'd' in current_keys: vel_y_rela  = -vy
        if 'q' in current_keys: vel_az_rela =  wz;state = 1
        if 'e' in current_keys: vel_az_rela = -wz;state = 1

        # ---- 云台目标：按住持续增加 ----
        if 'f' in current_keys: yaw_target   += yaw_step_per_frame
        if 'h' in current_keys: yaw_target   -= yaw_step_per_frame
        if 'g' in current_keys: pitch_target += pitch_step_per_frame
        if 't' in current_keys: pitch_target -= pitch_step_per_frame
        if 'y' in current_keys: shoot_state = 1

        if yaw_target > PI:
            yaw_target -= 2*PI
        elif yaw_target < -PI:
            yaw_target += 2*PI
        if pitch_target > 0.5:
            pitch_target = 0.5
        elif pitch_target < -0.5:
            pitch_target = -0.5
        # vel_az_rela=18
        # vel_x_rela=3
        # 顺序必须和 C++ 回调严格一致：0~7
        msg.data = [
            yaw_target,      # 0
            pitch_target,    # 1
            vel_x_rela,      # 2
            vel_y_rela,      # 3
            vel_az_rela,     # 4  ← 新增
            yaw_Kp,          # 5
            yaw_Ki,          # 6
            yaw_Kd,          # 7
            state,           # 8
            debug_state,     # 9
        ]
        pub.publish(msg)

        cmd_vel_msg.linear.x = vel_x_rela
        cmd_vel_msg.linear.y = vel_y_rela
        # cmd_vel_msg.linear.x = 300
        # cmd_vel_msg.linear.y = 300
        cmd_vel_msg.linear.z = 0.0
        cmd_vel_msg.angular.x = 0.0
        cmd_vel_msg.angular.y = 0.0
        cmd_vel_msg.angular.z = vel_az_rela
        cmd_vel_pub.publish(cmd_vel_msg)

        debug_state = temp_state
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass