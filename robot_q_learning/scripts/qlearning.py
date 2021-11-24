#!/usr/bin/env python3
import rospy
import numpy

from gazebo_msgs.msg import ModelState 
import matplotlib.pyplot as plt

from env import env

LEARNING_RATE = 0.2
DISCOUNT = 0.95
EPISODES = 600

epsilon = 1
START_EPSILON_DECAYING = 1
END_EPSILON_DECAYING = EPISODES//2
epsilon_decay_value = epsilon/(END_EPSILON_DECAYING - START_EPSILON_DECAYING)

q_table = numpy.random.uniform(low=-2, high=0, size=(9,3))
if __name__ == '__main__':
    rospy.init_node('qlearning', anonymous=True)
    top_forme=env()
    top_forme.reset()
    for episode in range(EPISODES):
        try:
            if episode % 200 == 0 :
                print(episode,"\n", q_table)
                numpy.save("src/robot_q_learning/scripts/qtable.npy",q_table)
            state = top_forme.reset()
            done = False
            while not done :
                if (rospy.is_shutdown()) :
                    break
                if numpy.random.random() > epsilon:
                    action = numpy.argmax(q_table[state])
                else :
                    action = numpy.random.randint(0,high=3)
                
                new_state, reward, done = top_forme.step(action,-10)
            
                if not done :
                    max_future_q = numpy.max(q_table[new_state])
                    current_q = q_table[state,action]

                    new_q = (1 - LEARNING_RATE) * current_q + LEARNING_RATE * (reward + DISCOUNT * max_future_q)
                    q_table[state,action]=new_q


                if done :
                    current_q = q_table[state,action]
                    new_q = (1 - LEARNING_RATE) * current_q + LEARNING_RATE * reward 
                    q_table[state,action]=new_q

                state=new_state

            if END_EPSILON_DECAYING >= episode >= START_EPSILON_DECAYING:
                epsilon -= epsilon_decay_value
        except KeyboardInterrupt :
            numpy.save("qtable.npy",q_table)

