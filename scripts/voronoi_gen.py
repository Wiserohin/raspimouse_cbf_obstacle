#! /usr/bin/env python
import numpy as np
from geovoronoi import coords_to_points, points_to_coords, voronoi_regions_from_coords
from geovoronoi.plotting import plot_voronoi_polys_with_points_in_area
from shapely.geometry import Polygon, Point
import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator

def gen_voronoi(pose, outer=[(10, 10), (10, -10), (-10, -10), (-10, 10)]):
    pose = np.column_stack((pose[0:2]))
    # print(pose)
    area_shape = Polygon(outer)
    poly_shapes, poly_to_pt_assignments = voronoi_regions_from_coords(pose, area_shape)
    # print(poly_shapes)
    print(poly_to_pt_assignments)
    new_coords = reshape_coords(pose)
    poly_centroids = np.array([poly_shapes[p].centroid for p in poly_shapes])
    # print(poly_centroids)
    new_centroids = match_pair(poly_shapes, new_coords, poly_centroids)
    # print(new_centroids)
    return area_shape, poly_shapes, poly_to_pt_assignments, new_centroids

def match_pair(poly_shapes, new_coords, centroids):
    new_centroids = []
    for n in centroids:
        #m = Point(n)
        new_centroids.append(n)
    sorted_centroids = []
    points = coords_to_points(new_coords)
    # print(points[1])
    for i, p in enumerate(points):
        for j, poly in enumerate(poly_shapes):
            # print(poly_shapes[poly])
            if p.within(poly_shapes[poly]):
                pair = new_centroids[j]
                sorted_centroids.append(pair)
    sorted_cen = np.column_stack((sorted_centroids))
    return sorted_cen

def reshape_coords(pose):
    coords = []
    for n in pose:
        m = Point(n)
        coords.append(m)
    return coords

def plotting(fig, ax, iter, save_fig=False):
    major_locator=MultipleLocator(4)
    ax.set_xlim([-12,12])
    ax.set_ylim([-12,12])
    font = {'size':13}
    ax.xaxis.set_major_locator(major_locator)
    ax.yaxis.set_major_locator(major_locator)
    #mngr = plt.get_current_fig_manager()
    #mngr.window.setGeometry(50,100,640, 545)
    # ax.set_xticks(np.arange(0, 2, step=0.2))
    # ax.set_y yticks(np.arange(0, 2, step=0.2))  
    ax.set_xlabel('x(m)', font, labelpad=15)
    ax.set_ylabel('y(m)', font, labelpad=15)    
    if save_fig:
        plt.savefig('/home/robolab/Coverage control/Trajectory_Log/Figure/FIG_'+str(iter)+'.png')
    #plt.title(str(j)  +"th itr")
    plt.tick_params(labelsize=13) #设置刻度字体大小
    plt.pause(0.001)
    ax.clear()
