#!/usr/bin/env python
import rospy
from swarmtal_msgs.msg import *


def onboard_state_gen():
    print("Will generate simulation data for swarm_commander_state")
    pub = rospy.Publisher('/drone_commander/swarm_commander_state', drone_commander_state, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        state = drone_commander_state()
        state.vo_valid = True
        state.bat_vol = 17.0
        state.bat_remain = 1000
        pub.publish(state)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("pilot_sim_gen", anonymous=True)
    onboard_state_gen()
