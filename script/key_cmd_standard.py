#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, PointStamped
from rm_msgs_exam.msg import GimbalCmd
from pynput import keyboard

PI = 3.14159265358979323846

# ============ 状态机 ============
cmd_state = 0  # 0: PID调试  1: debug_id选择  2: 底盘状态  3: 云台状态

# ============ 速度/目标参数 ============
vx = 4
vy = 4
wz = 10.0

yaw_step_per_frame = 0.3
pitch_step_per_frame = 0.1

# ============ PID 增益 ============
debug_kp = 4
debug_Ki = 0.0
debug_Kd = 0.0
gain_step = 0.1

# ============ 模块级状态 ============
yaw_target = 0.0
pitch_target = 0.0
gimbal_state = 1.0
chassis_state = 0.0
debug_id = 1.0

current_keys = set()


def on_press(key):
    try:
        if key.char in ['w', 's', 'a', 'd', 'q', 'e',
                        't', 'g', 'f', 'h', 'y',
                        '7', '4', '1', '8', '5', '2', ',', '.', '0',
                        '[', ']', 'o', 'p',
                        'c', 'v', 'b', 'n']:
            current_keys.add(key.char)
    except AttributeError:
        pass


def on_release(key):
    global debug_kp, debug_Ki, debug_Kd, chassis_state, debug_id, cmd_state, gimbal_state
    state_changed = False
    try:
        ch = key.char
        if ch == 'i':
            cmd_state = 0
            state_changed = True
        elif ch == 'o':
            cmd_state = 1
            state_changed = True
        elif ch == 'p':
            cmd_state = 2
            state_changed = True
        elif ch == '[':
            cmd_state = 3
            state_changed = True
        if cmd_state == 0:
            state_changed = True
            if ch == '7':
                debug_kp += gain_step;
            elif ch == '4':
                debug_Ki += gain_step;
            elif ch == '1':
                debug_Kd += gain_step;
            elif ch == '8':
                debug_kp -= gain_step;
            elif ch == '5':
                debug_Ki -= gain_step;
            elif ch == '2':
                debug_Kd -= gain_step;
        elif cmd_state == 1:
            if ch in '012345':
                debug_id = float(ch);
                state_changed = True
        elif cmd_state == 2:
            if ch in '0123':
                chassis_state = float(ch);
                state_changed = True
        elif cmd_state == 3:
            if ch in '123':
                gimbal_state = float(ch);
                state_changed = True

        if state_changed:
            rospy.loginfo("[STATE] cmd=%d debug=%.0f chassis=%.0f gimbal=%.0f | "
                          "Kp=%.3f Ki=%.3f Kd=%.3f",
                          cmd_state, debug_id, chassis_state, gimbal_state,
                          debug_kp, debug_Ki, debug_Kd)

        if ch in current_keys:
            current_keys.remove(ch)

    except AttributeError:
        pass

    if key == keyboard.Key.esc:
        return False


def build_gimbal_cmd(msg, gimbal_state, shoot_state):
    """根据 gimbal_state 填充 gimbalCmd 的各字段"""
    msg.mode = int(gimbal_state)

    # 默认清零
    msg.rate_yaw = 0.0
    msg.accel_yaw = 0.0
    msg.rate_pitch = 0.0
    msg.accel_pitch = 0.0
    # msg.pos_yaw = 0.0
    # msg.pos_pitch = 0.0
    msg.bullet_speed = 0.0
    msg.traj_yaw = 0.0
    msg.traj_pitch = 0.0
    msg.traj_frame_id = ""

    if msg.mode == 0:  # RATE
        msg.rate_yaw = 0.0
        msg.accel_yaw = 0.0
        msg.rate_pitch = 0.0
        msg.accel_pitch = 0.0
    elif msg.mode == 1:  # POS
        msg.pos_yaw = yaw_target
        msg.pos_pitch = pitch_target
    elif msg.mode == 2:  # TRACK
        msg.bullet_speed = 15.0
        # target_pos 需要外部填充，这里留空/保持默认
    # 如有 TRAJ 模式可加 elif msg.mode == 3

    return msg


def main():
    global yaw_target, pitch_target, chassis_state, debug_id, gimbal_state, cmd_state

    rospy.init_node('keyboard_teleop')
    pub = rospy.Publisher('/hero_gimbal_controller/sub_debug',
                          Float64MultiArray, queue_size=1)
    cmd_vel_pub = rospy.Publisher('/controller/hero_chassis_controller/cmd_vel',
                                  Twist, queue_size=1)
    gimbal_pub = rospy.Publisher('/controller/gimbal_controller/cmd',
                                 GimbalCmd, queue_size=1)
    rate = rospy.Rate(40)

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    msg = Float64MultiArray()
    cmd_vel_msg = Twist()
    gimbal_cmd_msg = GimbalCmd()

    rospy.loginfo("[PID] init  Kp=%.3f Ki=%.3f Kd=%.3f",
                  debug_kp, debug_Ki, debug_Kd)
    rospy.loginfo("[TARGET] init yaw=%.3f pitch=%.3f", yaw_target, pitch_target)

    while not rospy.is_shutdown():
        vel_x_rela = 0.0
        vel_y_rela = 0.0
        vel_az_rela = 0.0
        shoot_state = 0.0

        if 'w' in current_keys: vel_x_rela = vx
        if 's' in current_keys: vel_x_rela = -vx
        if 'a' in current_keys: vel_y_rela = vy
        if 'd' in current_keys: vel_y_rela = -vy
        if 'q' in current_keys:
            vel_az_rela = wz;
            chassis_state = 1
        if 'e' in current_keys:
            vel_az_rela = -wz;
            chassis_state = 1

        # ---- 云台目标：按住持续增加 ----
        if 'f' in current_keys: yaw_target += yaw_step_per_frame
        if 'h' in current_keys: yaw_target -= yaw_step_per_frame
        if 'g' in current_keys: pitch_target += pitch_step_per_frame
        if 't' in current_keys: pitch_target -= pitch_step_per_frame
        if 'y' in current_keys: shoot_state = 1.0

        # yaw 限幅到 [-PI, PI]
        if yaw_target > PI:
            yaw_target -= 2 * PI
        elif yaw_target < -PI:
            yaw_target += 2 * PI
        # pitch 限幅
        pitch_target = max(-0.5, min(0.5, pitch_target))

        # ---- debug 消息 ----
        msg.data = [
            float(cmd_state),
            debug_id,
            debug_kp,
            debug_Ki,
            debug_Kd,
            chassis_state,
            gimbal_state,
            shoot_state,
            yaw_target,
            pitch_target,
        ]
        pub.publish(msg)

        # ---- 底盘速度 ----
        cmd_vel_msg.linear.x = vel_x_rela
        cmd_vel_msg.linear.y = vel_y_rela
        cmd_vel_msg.linear.z = 0.0
        cmd_vel_msg.angular.x = 0.0
        cmd_vel_msg.angular.y = 0.0
        cmd_vel_msg.angular.z = vel_az_rela
        cmd_vel_pub.publish(cmd_vel_msg)

        # ---- 云台指令 ----
        gimbal_cmd_msg = build_gimbal_cmd(gimbal_cmd_msg, gimbal_state, shoot_state)
        gimbal_pub.publish(gimbal_cmd_msg)

        rate.sleep()

    listener.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
