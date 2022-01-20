import numpy as np 
import open3d as o3d

from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
from functools import reduce
import math
import os
import random
random.seed(10)

from SH import PolygonClipper
origin = []
refvec = []

# assign directory
directory = '/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/training_data/curvature_grad/verify_network/orig_planes.PCD.test'
 
def get_triangles_vertices(triangles, vertices):
    triangles_vertices = []
    for triangle in triangles:
        new_triangles_vertices = [vertices[triangle[0]], vertices[triangle[1]], vertices[triangle[2]]]
        triangles_vertices.append(new_triangles_vertices)
    return np.array(triangles_vertices)


def volume_under_triangle(triangle):
    p1, p2, p3 = triangle
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    x3, y3, z3 = p3
    return abs((z1+z2+z3)*(x1*y2-x2*y1+x2*y3-x3*y2+x3*y1-x1*y3)/6)



orig_pred_results = []
# iterate over files in
# that directory
for filename in sorted(os.scandir(directory), key=lambda f: f.name):
    if filename.is_file() & (filename.name[-4:] == ".pcd"):
        # load pcd file
        orig_pcd = o3d.io.read_point_cloud(filename.path)
        pred_pcd = o3d.io.read_point_cloud(filename.path.replace("orig_planes.PCD.test","pred_planes.PCD.test").replace("_orig.pcd","_pred.pcd"))
        # o3d.io.write_point_cloud(filename.path[:-4]+'_vis.pcd', pcd)

        # normalize point cloud
        orig_point_cloud = np.asarray(orig_pcd.points)
        pred_point_cloud = np.asarray(pred_pcd.points)

        orig_norm_pcd = orig_point_cloud/orig_point_cloud[2].max()
        pred_norm_pcd = pred_point_cloud/pred_point_cloud[2].max()
        
        orig_o3d_pcd = o3d.geometry.PointCloud()
        pred_o3d_pcd = o3d.geometry.PointCloud()
        
        orig_o3d_pcd.points = o3d.utility.Vector3dVector(orig_norm_pcd)
        pred_o3d_pcd.points = o3d.utility.Vector3dVector(pred_norm_pcd)
        # o3d.io.write_point_cloud(filename.path[:-4]+'_vis_norm.pcd', o3d_pcd)    

        # estimate plane
        orig_plane_model, orig_inliers = orig_o3d_pcd.segment_plane(distance_threshold=0.175,
                                                ransac_n=3,
                                                num_iterations=100)
        pred_plane_model, pred_inliers = pred_o3d_pcd.segment_plane(distance_threshold=0.175,
                                                ransac_n=3,
                                                num_iterations=100)
        #############################
        # orig plane translation
        [a, b, c, d] = orig_plane_model

        orig_plane_pcd = orig_o3d_pcd.select_by_index(orig_inliers)
        orig_plane_pcd.paint_uniform_color([1.0, 0, 0])
        # o3d.io.write_point_cloud(filename.path[:-4]+'_vis_plane_o3d.pcd', plane_pcd)

        # translate plane to xy axes
        orig_plane_pcd = orig_plane_pcd.translate((0,0,d/c))

        cos_theta = c / math.sqrt(a**2 + b**2 + c**2)
        sin_theta = math.sqrt((a**2+b**2)/(a**2 + b**2 + c**2))
        u_1 = b / math.sqrt(a**2 + b**2 )
        u_2 = -a / math.sqrt(a**2 + b**2)

        orig_rotation_matrix = np.array([[cos_theta + u_1**2 * (1-cos_theta), u_1*u_2*(1-cos_theta), u_2*sin_theta],
                                    [u_1*u_2*(1-cos_theta), cos_theta + u_2**2*(1- cos_theta), -u_1*sin_theta],
                                    [-u_2*sin_theta, u_1*sin_theta, cos_theta]])

        ##############################
        # pred plane translation 
        [a, b, c, d] = pred_plane_model

        pred_plane_pcd = pred_o3d_pcd.select_by_index(pred_inliers)
        pred_plane_pcd.paint_uniform_color([1.0, 0, 0])
        # o3d.io.write_point_cloud(filename.path[:-4]+'_vis_plane_o3d.pcd', plane_pcd)

        # translate plane to xy axes
        pred_plane_pcd = pred_plane_pcd.translate((0,0,d/c))

        cos_theta = c / math.sqrt(a**2 + b**2 + c**2)
        sin_theta = math.sqrt((a**2+b**2)/(a**2 + b**2 + c**2))
        u_1 = b / math.sqrt(a**2 + b**2 )
        u_2 = -a / math.sqrt(a**2 + b**2)

        pred_rotation_matrix = np.array([[cos_theta + u_1**2 * (1-cos_theta), u_1*u_2*(1-cos_theta), u_2*sin_theta],
                                    [u_1*u_2*(1-cos_theta), cos_theta + u_2**2*(1- cos_theta), -u_1*sin_theta],
                                    [-u_2*sin_theta, u_1*sin_theta, cos_theta]])


        orig_plane_pcd.rotate(orig_rotation_matrix)
        pred_plane_pcd.rotate(pred_rotation_matrix)

        # o3d.io.write_point_cloud(filename.path[:-4]+'_vis_plane_o3d_on_XY.pcd', plane_pcd)

        # ###################
        # ################### BELOW
        # ###################
        
        orig_xyz = np.asarray(orig_plane_pcd.points)
        orig_xyz[:,2] = orig_xyz[:,2] - orig_xyz[:,2].max()
        # orig_o3d_pcd.points = o3d.utility.Vector3dVector(orig_xyz)
        # o3d.io.write_point_cloud(filename.path[:-4]+'_vis_orig_below_xy.pcd', orig_o3d_pcd)

        pred_xyz = np.asarray(pred_plane_pcd.points)
        pred_xyz[:,2] = pred_xyz[:,2] - pred_xyz[:,2].max()
        # pred_o3d_pcd.points = o3d.utility.Vector3dVector(pred_xyz)
        # o3d.io.write_point_cloud(filename.path[:-4]+'_vis_pred_below_xy.pcd', pred_o3d_pcd)

        # orig_o3d_pcd.points = o3d.utility.Vector3dVector(orig_xyz)
        # o3d.io.write_point_cloud(filename.path[:-4]+'_vis_orig_project_XY.pcd', orig_o3d_pcd)
        # pred_o3d_pcd.points = o3d.utility.Vector3dVector(pred_xyz)
        # o3d.io.write_point_cloud(filename.path[:-4]+'_vis_pred_project_XY.pcd', pred_o3d_pcd)
        
        # If the point is P(x1,y1,z1) and the plane is ax+by+cz+d = 0 =>  dist = Abs(a*x1+b*y1+c*z1+d) / Sqrt(a^2+b^2+c^2)
        # XY_plane_coeff = [0., 0., 1., -0.]
        # [a, b, c, d] = XY_plane_coeff

        # orig_p2pl_dist = np.zeros((orig_xyz.shape[0],1))
        # orig_p2pl_dist[:,0] = abs(a*orig_xyz[:,0] + b*orig_xyz[:,1] + c*orig_xyz[:,2] + d) / math.sqrt(a**2 + b**2 + c**2)
        # in this case the actual distance of a point to the plane XY (in our case) is actually abs(Z)

        orig_distance_mean_below_XY = np.mean(abs(orig_xyz[:,2]))

        pred_distance_mean_below_XY = np.mean(abs(pred_xyz[:,2]))


        print(f"The mean distance (w.r.t XY) of the BELOW surface is: {round(orig_distance_mean_below_XY, 8)} m for ORIG file: {filename.name}")
        print(f"The mean distance (w.r.t XY) of the BELOW surface is: {round(pred_distance_mean_below_XY, 8)} m for PRED file: {filename.name}")

        # ###################
        # ################### ABOVE
        # ###################

        orig_xyz = np.asarray(orig_plane_pcd.points)
        orig_xyz[:,2] = orig_xyz[:,2] - orig_xyz[:,2].min()
        # orig_o3d_pcd.points = o3d.utility.Vector3dVector(orig_xyz)
        # o3d.io.write_point_cloud(filename.path[:-4]+'_vis_orig_above_xy.pcd', orig_o3d_pcd)


        pred_xyz = np.asarray(pred_plane_pcd.points)
        pred_xyz[:,2] = pred_xyz[:,2] - pred_xyz[:,2].min()
        # pred_o3d_pcd.points = o3d.utility.Vector3dVector(pred_xyz)
        # o3d.io.write_point_cloud(filename.path[:-4]+'_vis_pred_above_xy.pcd', pred_o3d_pcd)

        orig_distance_mean_above_XY = np.mean(abs(orig_xyz[:,2]))
        pred_distance_mean_above_XY = np.mean(abs(pred_xyz[:,2]))


        # orig_distance_pointwise_above = abs(orig_xyz[:,2])
        # pred_distance_pointwise_above = abs(pred_xyz[:,2])


        print(f"The mean distance (w.r.t XY) of the ABOVE surface is: {round(orig_distance_mean_above_XY, 8)} m for ORIG file: {filename.name}")
        print(f"The mean distance (w.r.t XY) of the ABOVE surface is: {round(pred_distance_mean_above_XY, 8)} m for PRED file: {filename.name}")

        if orig_distance_mean_below_XY + orig_distance_mean_above_XY == 0.0: orig_distance_mean_above_XY = 0.1
        orig_mean_dist_grad = orig_distance_mean_below_XY/(orig_distance_mean_below_XY + orig_distance_mean_above_XY)
        print(f"The mean distance gradient is: {round(orig_mean_dist_grad*100, 8)} % for ORIG file: {filename.name}")
        # orig_mean_dist_pointwise = np.mean(orig_distance_pointwise_below/(orig_distance_pointwise_below+orig_distance_pointwise_above))
        # print(f"The pointwise mean distance gradient is: {round(orig_mean_dist_pointwise*100, 8)} % for ORIG file: {filename.name}")

        if pred_distance_mean_below_XY + pred_distance_mean_above_XY == 0.0: pred_distance_mean_above_XY = 0.1
        pred_mean_dist_grad = pred_distance_mean_below_XY/(pred_distance_mean_below_XY + pred_distance_mean_above_XY)
        print(f"The mean distance gradient is: {round(pred_mean_dist_grad*100, 8)} % for PRED file: {filename.name}")
        print("---------")
        orig_pred_results.append([orig_mean_dist_grad,pred_mean_dist_grad])

np.savetxt("curv_grad_orig_pred_test.csv",np.array(orig_pred_results), delimiter=",")
