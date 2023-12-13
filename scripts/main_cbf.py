#! /usr/bin/env python3
import rospy
import numpy as np
from raspi import *
from transform import *
from controller_obs import *

#This script used for experimenting with simulation.
# identify agent angle wrt main coordinate system
# subtract

N = 1

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    # print(x)
    return [x, y]

def infer_obs_position(pose, scans):
    obs_pos = pol2cart(*scans)
    obs_pos[0] += pose[0][0]
    obs_pos[1] += pose[1][0]
    return obs_pos

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        radius = 2
        xybound = radius*np.array([-1, 1, -1, 1])
        p_theta = 2*np.pi*(np.arange(0, 2*N, 2)/(2*N))
        p_circ = np.vstack([
            np.hstack([xybound[1]*np.cos(p_theta), xybound[1]*np.cos(p_theta+np.pi)]),
            np.hstack([xybound[3]*np.sin(p_theta), xybound[3]*np.sin(p_theta+np.pi)])
            ])
        flag = 0
        x_goal = p_circ[:, :N] 
        while not rospy.is_shutdown():
            scans = get_lidar_values(N)
            pose = getposition(N)

            obstacle_pose = np.array([infer_obs_position(pose, scans)])
            pose_si = uni_to_si_states(pose)

            
            if(np.linalg.norm(x_goal - pose_si) < 0.05):
                flag = 1-flag

            if(flag == 0):
                x_goal = p_circ[:, :N]
            else:
                x_goal = p_circ[:, N:]


            #print(x_goal)
            #print(type(x_goal))
            dxi = si_position_controller(pose_si, x_goal)
            dxi = si_barrier_cert(dxi, pose_si, np.transpose(obstacle_pose))
            #dxi = si_barrier_cert(dxi, pose_si)
            dxu = si_to_uni_dyn(dxi, pose)
            k = set_velocities(N, dxu)
            put_velocities(N, k)

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass

