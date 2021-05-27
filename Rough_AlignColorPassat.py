import open3d as o3d
import numpy as np
import copy

# Transform matrices
mat1 = np.array([[-0.0377622982, -0.96796581,  0.240974271, 3.44969499 * 1000],
                 [- 0.985415513, -0.003900978, -0.170120691, 6.73985317 * 1000],
                 [ 0.1659225,  -0.243883933, -0.955505286, 1.71990719 * 1000],
                 [ 0.0000000, 0.00000000, 0.00000000, 1.00000000]])

mat2 = np.array([[-0.0536614288, -0.928534459, 0.367347532,  3.20033576 * 1000],
                 [-0.988315655, -0.00317159013, -0.152388014, 6.73852872 * 1000],
                 [ 0.142662598, -0.371232675, -0.917514950,  1.70775487 * 1000],
                 [ 0.00000000, 0.00000000, 0.00000000, 1.00000000]])

mat3 = np.array([[ 0.0309121102, -0.999873330,  0.0156130765,  3.88724875 * 1000],
                 [-0.934321642, -0.00845267571, -0.356330775,  7.24987967 * 1000],
                 [ 0.356417611, -0.0134861417, -0.934229421,  1.53050629 * 1000],
                 [ 0.00000000, 0.00000000, 0.00000000, 1.00000000]])

mat4 = np.array([[-0.03680778, -0.99756927,  0.05916703,  3.47510652 * 1000],
                 [-0.93422404,  0.01332976, -0.35643761,  7.24544995 * 1000],
                 [ 0.35478252, -0.06839494, -0.93244383,  1.54212951 * 1000],
                 [ 0.0,        0.0,         0.0,          1.0,        ]])

mat5 = np.array([[ 0.00737985, -0.98975591, -0.14257899,  4.22791081 * 1000],
                 [-0.98496723,  0.0174133,  -0.17186138,  6.61318243 * 1000],
                 [ 0.17258359,  0.14170394, -0.97474863,  1.55267043 * 1000],
                 [ 0.0,        0.0,        0.0,        1.0        ]])

mat6 = np.array([[-0.02061423, -0.99800177, -0.05972874,  3.9985729 * 1000],
                 [-0.9931793,   0.013584,    0.11580306,  5.99286022 * 1000],
                 [-0.11476031,  0.06170854, -0.99147472,  1.6525075 * 1000],
                 [ 0.0,         0.0,         0.0,          1.0        ]])

mat7 = np.array([[ 0.0064842,  -0.98595401,  0.16689111,  3.46919449 * 1000],
                 [-0.9933272,   0.01286723,  0.11461024,  5.97931878 * 1000],
                 [-0.11514785, -0.16652063, -0.9792915,   1.69887682 * 1000],
                 [ 0.0,         0.0,         0.0,         1.0        ]])


def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target])

print("1. Load two point clouds and show initial pose")
source = o3d.io.read_point_cloud("IR/VW-passat/Pic7.pcd")
target = o3d.io.read_point_cloud("IR/ColorICP_Model/Stitch5.pcd")

# Initial alignment transform
print("Make inital transform with known matrices")
source = source.transform(mat7)
#target = target.transform(mat2)

# draw initial alignment
current_transformation = np.identity(4)
draw_registration_result_original_color(source, target, current_transformation)


# Colored ICP
voxel_radius = [2, 1, 0.5]
max_iter = [50, 30, 14]
current_transformation = np.identity(4)
print("3. Colored point cloud registration")
for scale in range(3):
    iter = max_iter[scale]
    radius = voxel_radius[scale]
    print([iter, radius, scale])

    print("3-1. Downsample with a voxel size %.2f" % radius)
    source_down = source.voxel_down_sample(radius)
    target_down = target.voxel_down_sample(radius)

    print("3-2. Estimate normal.")
    source_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
    target_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

    print("3-3. Applying colored point cloud registration")
    result_icp = o3d.pipelines.registration.registration_colored_icp(
        source_down, target_down, radius, current_transformation,
        o3d.pipelines.registration.TransformationEstimationForColoredICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                          relative_rmse=1e-6,
                                                          max_iteration=iter))
    current_transformation = result_icp.transformation
    print(result_icp)
draw_registration_result_original_color(source, target,
                                        result_icp.transformation)

source_transformed = o3d.geometry.PointCloud.transform(source, result_icp.transformation)
new_pointcloud = source_transformed + target
o3d.visualization.draw_geometries([new_pointcloud])
print("4.  Resultant point cloud saved.")
o3d.io.write_point_cloud("IR/ColorICP_Model/Stitch6.pcd", new_pointcloud)
