#!/usr/bin/env python3
"""Crop the contiguous visible LiDAR arc; never invent clear rays in blind sectors."""
import copy
import math


def crop_scan(msg, low=-89.9, high=119.9):
    metadata = (msg.angle_increment, msg.angle_min, msg.time_increment, msg.range_min, msg.range_max)
    if (not all(math.isfinite(v) for v in metadata) or msg.angle_increment <= 0 or
            msg.time_increment < 0 or not 0 <= msg.range_min < msg.range_max or len(msg.ranges) == 0):
        raise ValueError('Mapping needs a positive scan increment')
    first = max(0, int(math.ceil((math.radians(low)-msg.angle_min)/msg.angle_increment)))
    last = min(len(msg.ranges)-1, int(math.floor((math.radians(high)-msg.angle_min)/msg.angle_increment)))
    if last-first < 10:
        raise ValueError('No usable mapping scan arc')
    out = copy.copy(msg)
    out.header = copy.copy(msg.header)
    # First retained beam has a later acquisition time on a spinning LiDAR.
    out.header.stamp = msg.header.stamp + __import__('rospy').Duration(first*msg.time_increment)
    # Invalid near/NaN readings are unknown, not max-range free-space rays.
    out.ranges = [r if (msg.range_min <= r <= msg.range_max or r == float('inf')) else float('nan')
                  for r in msg.ranges[first:last+1]]
    out.intensities = msg.intensities[first:last+1] if len(msg.intensities) == len(msg.ranges) else []
    out.angle_min = msg.angle_min+first*msg.angle_increment
    out.angle_max = msg.angle_min+last*msg.angle_increment
    return out


def main():
    import rospy
    from sensor_msgs.msg import LaserScan
    rospy.init_node('test_mapping_scan')
    pub = rospy.Publisher('/test_mapping/scan', LaserScan, queue_size=1)
    def callback(msg):
        try:
            pub.publish(crop_scan(msg))
        except ValueError as exc:
            rospy.logwarn_throttle(5., str(exc))
    rospy.Subscriber('/scan', LaserScan, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
