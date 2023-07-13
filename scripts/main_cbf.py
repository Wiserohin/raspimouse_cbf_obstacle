#! /usr/bin/env python3
import rospy
import numpy as np
from raspi import *
from transform import *
from controller_obs import *

#This script used for experimenting with simulation.

N = 20

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        radius = 1.5
        xybound = radius*np.array([-1, 1, -1, 1])
        p_theta = 2*np.pi*(np.arange(0, 2*N, 2)/(2*N))
        p_circ = np.vstack([
            np.hstack([xybound[1]*np.cos(p_theta), xybound[1]*np.cos(p_theta+np.pi)]),
            np.hstack([xybound[3]*np.sin(p_theta), xybound[3]*np.sin(p_theta+np.pi)])
            ])
        print(p_circ)
        flag = 0
        x_goal = p_circ[:, :N] 
        while not rospy.is_shutdown():
            pose = getposition(N)
            #print(pose)
            #obs = get_obstacle_position(1)
            #print(obs)
            # print (np.column_stack((pose[0:2])))
            pose_si = uni_to_si_states(pose)
            print(pose_si)
            if(np.linalg.norm(x_goal - pose_si) < 0.05):
                flag = 1-flag

            if(flag == 0):
                x_goal = p_circ[:, :N]
            else:
                x_goal = p_circ[:, N:]


            print(x_goal)
            print(type(x_goal))
            input()
            dxi = si_position_controller(pose_si, x_goal)
            #dxi = si_barrier_cert(dxi, pose_si, np.transpose(np.array([[1, 0], [2, 0]])))
            dxi = si_barrier_cert(dxi, pose_si)
            dxu = si_to_uni_dyn(dxi, pose)
            k = set_velocities(N, dxu)
            put_velocities(N, k)

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass
