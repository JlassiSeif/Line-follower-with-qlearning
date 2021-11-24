#!/usr/bin/env python3

import rospy
import numpy
from robot import robot


if __name__ == '__main__' :
    rospy.init_node('try',anonymous=True)
    # q_table=numpy.load('qtable.npy',mmap_mode='r')
    q_table=numpy.load('src/robot_q_learning/scripts/qtable.npy',mmap_mode='r')
    robot=robot()
    old_state = 6
    current_state=robot.computeState()
    while(not rospy.is_shutdown()) :
        current_state=robot.computeState()
        action=numpy.argmax(q_table[current_state])
        if not current_state == old_state :
            print(current_state,action,q_table[current_state,action])
        robot.move(action)
        if (rospy.is_shutdown()) :
            break
        old_state=current_state
    robot.move(9)
