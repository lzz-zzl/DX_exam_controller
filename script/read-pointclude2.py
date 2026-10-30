#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def callback(msg):
    # 按 fields 里的实际名称读取，顺序要和 fields 一致
    points = pc2.read_points(
        msg,
        field_names=("x", "y", "z", "intensity", "ring", "time"),
        skip_nans=True
    )

    header_sec = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
    print(f"帧 header.stamp = {header_sec:.9f}")
    print(f"point_step = {msg.point_step}, width = {msg.width}, height = {msg.height}")
    print("-" * 60)

    for i, pt in enumerate(points):
        x, y, z, intensity, ring, t = pt
        absolute = header_sec + t  # 假设 time 是相对秒偏移
        print(f"点{i:3d}: x={x:8.3f} y={y:8.3f} z={z:8.3f} "
              f"int={intensity:6.1f} ring={ring:3d} time={t:.6f} "
              f"绝对时间={absolute:.6f}")
        if i >= 4:
            break

    rospy.signal_shutdown("done")

rospy.init_node('check_point_time')
rospy.Subscriber('/points', PointCloud2, callback)
rospy.spin()