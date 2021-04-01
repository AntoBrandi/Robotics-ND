#!/usr/bin/env python
'''simple_mover ROS Node'''
# license removed for brevity
import rospy
import math
from std_msgs.msg import Float64

def mover():
    '''simple_mover Publisher'''
    pub_j1 = rospy.Publisher('/simple_arm/joint_1_position_controller/command', Float64, queue_size=10)
    pub_j2 = rospy.Publisher('/simple_arm/joint_2_position_controller/command', Float64, queue_size=10)

    rospy.init_node('simple_mover', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    start_time = 0

    while not start_time:
        start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        elapsed = rospy.Time.now().to_sec() - start_time
        # apply a sin wave for each joint
        pub_j1.publish(math.sin(2*math.pi*0.1*elapsed)*(math.pi/2))
        pub_j2.publish(math.sin(2*math.pi*0.1*elapsed)*(math.pi/2))
        rate.sleep()

if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
