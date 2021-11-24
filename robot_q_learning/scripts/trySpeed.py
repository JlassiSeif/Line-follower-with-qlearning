#!/usr/bin/env python3

import rospy
import numpy
from robot import robot
ACTIONS = {
            0 : 2,
            1 : 2,
            2 : 2, 
            3 : 2,
            4 : 0,
            5 : 1,
            6 : 1,
            7 : 1, 
            8 : 1
}

if __name__ == '__main__' :
    rospy.init_node('try',anonymous=True)
    # q_table=numpy.load('qtable.npy',mmap_mode='r')
    q_table=numpy.load('src/robot_q_learning/scripts/qtable_speed2.npy', mmap_mode='r')
    robot=robot()
    old_state = 10
    current_state=robot.computeState()
    while(not rospy.is_shutdown()) :
        current_state=robot.computeState()
        action = ACTIONS[current_state]
        speed =numpy.argmax(q_table[current_state,::,action])
        if not current_state == old_state :
            print(current_state,action,q_table[current_state,speed,action])
        robot.move(action,speed)
        if (rospy.is_shutdown()) :
            break
        old_state=current_state
    robot.move(9,15)
