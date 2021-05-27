import open3d as o3d
import numpy as np

# Transformation matrices for each point cloud

mat_1 = np.array([[-0.03764635, -0.97439976,  0.22164806,  3.33859509 * 1000],
                 [-0.99170047,  0.00914288, -0.12824422,  5.62242371 * 1000],
                 [ 0.12293463, -0.22463642, -0.9666569,   1.66858798 * 1000],
                 [ 0.0,          0.0,          0.0,          1.0        ]])

mat_2 = np.array([[-0.03768519, -0.97438566,  0.22170342,  3.13782491 * 1000],
                 [-0.99169369,  0.00916339, -0.12829523,  5.08519672 * 1000],
                 [ 0.12297748, -0.22469671, -0.96663743,  1.66862846 * 1000],
                 [ 0.0,          0.0,          0.0,          1.0        ]])

mat_3 = np.array([[-0.03777397, -0.97436824,  0.22176486,  3.10480584 * 1000],
                 [-0.99168529,  0.00923185, -0.12835518,  4.22786681 * 1000],
                 [ 0.12301791, -0.22476944, -0.96661538,  1.6687094 * 1000],
                 [ 0.0,          0.0,          0.0,          1.0        ]])

mat_4 = np.array([[ 0.01933734, -0.97466984, -0.22281106,  4.19706297 * 1000],
                 [-0.99220078,  0.00873988, -0.12434314,  4.22812034 * 1000],
                 [ 0.12314085,  0.22347778, -0.96689917,  1.57773841 * 1000],
                 [ 0.0,          0.0,          0.0,          1.0        ]])

mat_5 = np.array([[ 0.01930838, -0.97468298, -0.22275608,  4.19702783 * 1000],
                 [-0.9922023,   0.00875913, -0.12432969,  4.86561984 * 1000],
                 [ 0.12313318,  0.2234197,  -0.96691357,  1.57783708 * 1000],
                 [ 0.0,          0.0,          0.0,          1.0        ]])

mat_6 = np.array([[ 0.01928472, -0.97468159, -0.2227642,   4.19703142 * 1000],
                 [-0.99220329,  0.00878206, -0.12432016,  5.31357862 * 1000],
                 [ 0.1231289,   0.22342485, -0.96691293,  1.57783863 * 1000],
                 [ 0.0,          0.0,          0.0,          1.0        ]])

mat_7 = np.array([[-0.4051488,  -0.87810879, -0.25451798,  4.13368431 * 1000],
                 [-0.90592054,  0.42308186, -0.01759905,  3.36780266 * 1000],
                 [ 0.12313582,  0.22344283, -0.96690789,  1.57783202 * 1000],
                 [ 0.0,          0.0,          0.0,          1.0        ]])

mat_8 = np.array([[-0.31512589, -0.91509224, -0.25159862,  3.61969127 * 1000],
                 [-0.94102781,  0.33567852, -0.04226807,  3.44733883 * 1000],
                 [ 0.12313544,  0.22344153, -0.96690824,  1.57783228 * 1000],
                 [ 0.0,          0.0,          0.0,          1.0        ]])


matrices = [0, mat_1, mat_2, mat_3, mat_4, mat_5, mat_6, mat_7, mat_8]
pcds = []
voxel_size = 2
i = 1

for i in range(8):
    i = i+1
    print("Processing pcl number %d." % i)
    # Read point clouds
    pcd = o3d.io.read_point_cloud("IR/VW-EGolf/Pic%d.pcd" % i)

    # Downsize point clouds
    pcd_down = pcd.voxel_down_sample(voxel_size = voxel_size)

    print("Transforming pcl number %d." % i)
    # Initial transform
    pcd_init = pcd_down.transform(matrices[i])

    # Add to list
    pcds.append(pcd_init)

# Visualize
o3d.visualization.draw_geometries(pcds)

# Save copy of stitched pointcloud
print(pcds)
pcd_combined = o3d.geometry.PointCloud()

for point_id in range(len(pcds)):
    pcd_combined += pcds[point_id]

#o3d.io.write_point_cloud("IR/E-Golf_Model.pcd", pcd_combined)
print("Point Cloud saved.")
o3d.visualization.draw_geometries([pcd_combined])
