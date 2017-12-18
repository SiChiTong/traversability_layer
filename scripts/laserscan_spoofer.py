#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def init():
    rospy.init_node('laserscan_spoofer')
    pub = rospy.Publisher('laserscan_spoofer/scan', LaserScan, queue_size=1)
    rate = rospy.Duration(0.1)
    msg = LaserScan()
    msg.header.frame_id = 'laser_link'
    msg.angle_min = -2.35619449615
    msg.angle_max = 2.09234976768
    msg.angle_increment = 0.00613592332229
    msg.time_increment = 0.0000976562732831
    msg.scan_time = 0.10000000149
    msg.range_min = 0.019999999553
    msg.range_max = 5.59999990463
    sz = int(float(msg.angle_max - msg.angle_min) / float(msg.angle_increment))
    msg.ranges = [float('NaN') for i in range(sz)]

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time(0)
        pub.publish(msg)
        rospy.sleep(rate)

if __name__ == '__main__':
    init()