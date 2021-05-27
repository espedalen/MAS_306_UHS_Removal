from numpy.core.defchararray import asarray
from numpy.core.fromnumeric import shape
import open3d as o3d
import numpy as np
import time

seconds_start = time.time()

def evaluate_edge(pcd, pcd_tree, point_no):
    #Point for evaluation
    point = pcd.normals[point_no]

    #Find its neighbors with distance less than 20mm"
    [k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[point_no], 20)

    #Compute mean value of neighbours
    mean = np.asarray(pcd.normals)[idx[1:], :].mean(axis=0)

    #Edge calssification if the difference is above 
    # 0.4 in any direction the point is an edge
    if point[0] - mean[0] > 0.4:
        print("Paint the point red.")
        pcd.colors[point_no] = [1, 0, 0]

    elif point[1] - mean[1] > 0.4:
        print("Paint the point red.")
        pcd.colors[point_no] = [1, 0, 0]

    elif point[2] - mean[2] > 0.4:
        print("Paint the point red.")
        pcd.colors[point_no] = [1, 0, 0]

#Import and downsize point cloud
pcd = o3d.io.read_point_cloud("IR/E-Golf_Model.pcd")
voxel_size = 5
pcd = pcd.voxel_down_sample(voxel_size)

pcd_array = np.asarray(pcd.points)
print(pcd_array.shape[0])

#Estimate normals
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=10, max_nn=30))

#Build tree
pcd_tree = o3d.geometry.KDTreeFlann(pcd)

#Evaluation loop
for i in range(pcd_array.shape[0]):
    
    iter = i
    print("Value %.2f" % iter)
    evaluate_edge(pcd, pcd_tree, iter)

o3d.visualization.draw_geometries([pcd])

seconds_end = time.time()

computation_time = seconds_end - seconds_start
print(computation_time)
