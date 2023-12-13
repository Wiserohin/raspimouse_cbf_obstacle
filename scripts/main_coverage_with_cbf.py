#! /usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
import numpy as np
from voronoi_gen import *
from raspi import *
from transform import *
from controller_obs import *

#Coverage control in certain area using Voronoi-based algorithm & Control barrier function for avoiding the collision.
#This script used for experimenting with simulation.

N = 4
LOG_DIR = '/home/robolab/Desktop/ro/logs'
obstacles = np.transpose(np.array([[1, 0], [2, 0]]))

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        x_traj = np.empty((0, N), float)
        y_traj = np.empty((0, N), float)

        fig = plt.figure(figsize=(5.7,5))
        fig.subplots_adjust(wspace=0, hspace=0, top=0.95, bottom=0.15)
        ax = plt.subplot()
        first = True
        #plt.axis('scaled')
        #ax.set_xlim(-6, 6)
        #ax.set_ylim(-6, 6)

        while not rospy.is_shutdown():
            plotting(fig, ax, 50)
            pose = getposition(N)
            pose_si = uni_to_si_states(pose)
            x_traj = np.append(x_traj, pose[0:1], axis=0)
            y_traj = np.append(y_traj, pose[1:2], axis=0)

            (area_shape, poly_shape, poly2pt, centroids) = gen_voronoi(pose)
            #print(coords_to_points(np.column_stack((pose[0:2]))))
            plot_voronoi_polys_with_points_in_area(ax, area_shape, poly_shape, coords_to_points(np.column_stack((pose[0:2]))), region_pts=poly2pt, voronoi_edgecolor='black', points_color='black', 
                                        points_markersize=30, voronoi_and_points_cmap=None)
            # print(points_to_coords(centroids[0]))
            # print(centroids.size)
            for x in np.column_stack((centroids)):
                c1 = points_to_coords(x)[0]
                #print(c1)
                ax.plot(c1[0],c1[1], 'rs', markersize = 12, alpha = 0.4)
            # input("Press <ENTER> to continue.")

            centroids = np.transpose(points_to_coords(centroids[0]))
            if(np.linalg.norm(centroids - pose_si) < 0.01):
             #   np.savetxt(LOG_DIR+'/X_traj.csv', x_traj, delimiter=' , ')
              #  np.savetxt(LOG_DIR+'/Y_traj.csv', y_traj, delimiter=' , ')
                rospy.signal_shutdown('End of testing')
                pass
            
            # print(np.transpose(points_to_coords(centroids[0])))
            dxi = si_position_controller(pose_si, centroids)
            dxi = si_barrier_cert(dxi, pose_si, obstacles)
            dxu = si_to_uni_dyn(dxi, pose)
            if first:
                first = False
                input()
            k = set_velocities(N, dxu)
            put_velocities(N, k)
            

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass
