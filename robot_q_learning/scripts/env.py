#!/usr/bin/env python3
import rospy
import numpy

from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

from time import sleep

from robot import robot

class env():
    def __init__(self):
        self.robot=robot()
        self.ModelPub = rospy.Publisher('/gazebo/set_model_state',ModelState,queue_size=10)

    def step(self,action, speed):
        reward=0
        c=False
        current_state=self.robot.computeState()
        while(self.robot.computeState()==current_state and c==False):
            c=self.robot.get_noContour()
            self.robot.move(action, speed)
            new_state=self.robot.computeState()
            if (rospy.is_shutdown()) :
                break
        self.robot.move(9,-10)
        new_state=self.robot.computeState()
        done = False
        if self.robot.get_noContour() == True :
            reward = -2
            done = True

        if  new_state == 3 or new_state == 5 :
            if current_state < 3 or current_state > 5 :
                reward = 0.5
            else :
                reward = -0.5
        
        elif  new_state == 2 or new_state == 6 :
            if current_state < 2 or current_state > 6 :
                reward = 0.5
            else :
                reward = -0.8

        elif  new_state == 1 or new_state == 7 :
            if current_state < 1 or current_state > 7 :
                reward = 0.5
            else :
                reward = -0.8

        elif new_state == 0 or new_state== 8 :
            reward = -2
        
        else :
            reward = 2
        
        
        return new_state, reward, done
        # if  new_state == 3 or new_state == 5 :
        #     if current_state < 3 or current_state > 5 :
        #         reward = 0.5
        #     else :
        #         reward = -0.5
        # elif  new_state == 2 or new_state == 6 :
        #     if current_state < 2 or current_state > 6 :
        #         reward = 0.5
        #     else :
        #         reward = -0.5
        # elif  new_state == 1 or new_state == 7 :
        #     if current_state < 1 or current_state > 7 :
        #         reward = 0.5
        #     else :
        #         reward = -0.5
        # elif new_state == 0 or new_state== 8 :
        #     reward = -1
        # else :
        #     reward = 2
        # return new_state, reward, done
       
    
    def reset(self):
        state_msg = ModelState()
        state_msg.pose.position.x=1
        state_msg.pose.position.y=-0.8
        state_msg.pose.position.z=0.1
        state_msg.pose.orientation.z=1
        state_msg.pose.orientation.w=0.0000463
        state_msg.model_name = "line_follower"
        state_msg.reference_frame='world'
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )
        except rospy.ServiceException:
            print("/gazebo/get_model_state service call failed")   
        sleep(0.1)
        return self.robot.computeState()

