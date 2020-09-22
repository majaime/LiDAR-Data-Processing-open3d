# This code reads .bin file (specially obtained from LIDAR),
# stores that to a.csv file, and plots the output in a point-cloud format.

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import figure

""" Reading data """
inputPath = '/home/mohammadamin/Desktop/2011_09_26/2011_09_26_drive_0002_sync/velodyne_points/data/0000000000.bin'
outputPath = '/home/mohammadamin/Desktop/2011_09_26/2011_09_26_drive_0002_sync/velodyne_points/data/0000000000.csv'

num = np.fromfile(inputPath, dtype='float32', count=-1, sep='', offset=0)
new = np.asarray(num).reshape(-1, 4)

""" Storing the encrypted (.bin) file in (.csv) file """
# for i in range(0, len(new)): # len(new)
#     print(new[i])
#     with open(outputPath, 'a', newline='') as csvfile:
#         csv_writer = csv.writer(csvfile)
#         csv_writer.writerow(new[i])

""" Plotting the points using scatter """
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
X = num[0::4]
Y = num[1::4]
Z = num[2::4]
W = num[3::4]
#
# pointSize = 0.01 * (1 // 1)  # 0.01*(1//0.02)
# ax.scatter(X, Y, Z, s=pointSize, c=W)
# ax.set_title("Point Cloud Data Visualization")
# ax.set_xlabel("X")
# ax.set_ylabel("Y")
# ax.set_zlabel("Z")
# plt.show()


""" Plotting the points using pptk """
import pptk

# P = np.random.rand(10, 3)
# P = [X, Y, Z]
# rgb = pptk.rand(100, 3)
# v = pptk.viewer(P)
# v.close()
# v.set(point_size=0.01)

""" Plotting the points using mayavi """
# from mayavi import mlab
#
# pointCloud = mlab.points3d(X, Y, Z, W, scale_factor=.3, mode='point')  # Last parameter determines colorfulness in
# mlab.show()                                               # terms of how far is a point from the center


""" Filtering, segmenting, and visualizing the points using open3d """
import open3d as o3d

xyz = np.zeros((np.size(X), 3))
xyz[:, 0] = X
xyz[:, 1] = Y
xyz[:, 2] = Z
pc = o3d.geometry.PointCloud()
pc.points = o3d.utility.Vector3dVector(xyz)
# o3d.io.write_point_cloud("/home/mohammadamin/Desktop/2011_09_26/lidar.ply", pcd)
# z = np.sinc((np.power(X, 2) + np.power(Y, 2)))
# z_norm = (z - z.min()) / (z.max() - z.min())
# img = o3d.geometry.Image((z_norm * 255).astype(np.uint8))
# o3d.io.write_image("/home/mohammadamin/Desktop/2011_09_26/lidar.png", img)

# Downsampling
downpcd = pc.voxel_down_sample(voxel_size=0.1)

# Segmentation
plane_model, inliers = downpcd.segment_plane(distance_threshold=1, ransac_n=10, num_iterations=1000)
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
inlier_cloud = downpcd.select_by_index(inliers)
# inlier_cloud.paint_uniform_color([0, 0, 0])
outlier_cloud = downpcd.select_by_index(inliers, invert=True)
# outlier_cloud.paint_uniform_color([1, 0, 0])

# Clustering using DBSCAN
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(inlier_cloud.cluster_dbscan(eps=2, min_points=10, print_progress=True))
    # esp: Distance to neighbours in a cluster
    # min_points: Minimun number of points required to form a cluster
max_label = labels.max()
print(f"point cloud has {max_label + 1} cluster`s")
colors = plt.get_cmap("tab20")(labels/(max_label if max_label > 0 else 1))
colors[labels < 0] = 0
inlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

# Visualization
o3d.visualization.draw_geometries([inlier_cloud])


""" Filtering, segmenting, and visualizing the points using pcl """
# import pcl.pcl_visualization
#
# xyz = np.zeros((np.size(X), 3))
# xyz[:, 0] = np.reshape(X, -1)
# xyz[:, 1] = np.reshape(Y, -1)
# xyz[:, 2] = np.reshape(Z, -1)
# pc = pcl.PointCloud()
# pcd = pc.from_list(xyz)
# Voxel filter
# voxel = pc.make_voxel_grid_filter()
# voxel.set_leaf_size(0.5, 0.5, 0.5)
# voxel_filter = voxel.filter()
# Statistical outlier filter
# outlier = pc.make_statistical_outlier_filter()
# outlier.set_mean_k(300)
# outlier.set_std_dev_mul_thresh(0.01)
# outlier_filter = outlier.filter()
# Segmentation
# seg = pc.make_segmenter()
# seg.set_model_type(pcl.SACMODEL_PLANE)
# seg.set_method_type(pcl.SAC_RANSAC)
# seg.set_MaxIterations(1000)
# seg.set_distance_threshold(1)
# inliers, plane_model = seg.segment()
# [a, b, c, d] = plane_model
# print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
# inlier_cloud = pc.extract(inliers, negative=False)
# outlier_cloud = pc.extract(inliers, negative=True)
# Clustering using KdTree and Eculidean
# tree = pc.make_kdtree()
# ec = pc.make_EuclideanClusterExtraction()
# ec.set_ClusterTolerance(0.02)
# ec.set_MinClusterSize(10)
# ec.set_MaxClusterSize(25000)
# ec.set_SearchMethod(tree)
# cluster_indices = ec.Extract()
# print(cluster_indices)
# color_cluster_point_list = []
# cloud_cluster = pcl.PointCloud()
# for j, indices in enumerate(cluster_indices):
#     # cloudsize = indices
#     print('indices = ' + str(len(indices)))
#     # cloudsize = len(indices)
#     points = np.zeros((len(indices), 3), dtype=np.float32)
#     # points = np.zeros((cloudsize, 3), dtype=np.float32)
#
#     # for indice in range(len(indices)):
#     for i, indice in enumerate(indices):
#         # print('dataNum = ' + str(i) + ', data point[x y z]: ' + str(cloud_filtered[indice][0]) + ' ' + str(cloud_filtered[indice][1]) + ' ' + str(cloud_filtered[indice][2]))
#         # print('PointCloud representing the Cluster: ' + str(cloud_cluster.size) + " data points.")
#         points[i][0] = pc[indice][0]
#         points[i][1] = pc[indice][1]
#         points[i][2] = pc[indice][2]
#
#     cloud_cluster.from_array(points)
#     ss = "cloud_cluster_" + str(j) + ".pcd";
#     pcl.save(cloud_cluster, ss)
#
# get_color_list = []
# cluster_color = get_color_list(len(cluster_indices))
# color_cluster_point_list = []
# for j, indices in enumerate(cluster_indices):
#         for i, indice in enumerate(indices):
#             color_cluster_point_list.append([
#                                             pc[indice][0],
#                                             pc[indice][1],
#                                             pc[indice][2]])

#Create new cloud containing all clusters, each with unique color
# cluster_cloud = pcl.PointCloud_PointXYZRGB()
# cluster_cloud.from_list(color_cluster_point_list)
# Visualization
# visual = pcl.pcl_visualization.CloudViewing()
# visual.ShowMonochromeCloud(cluster_indices)
