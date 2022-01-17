from turtle import shape
import open3d as o3d
import numpy as np
import math
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
import math
from functools import reduce
import os
import random
random.seed(10)

from SH import PolygonClipper
origin = []
refvec = []

# assign directory
directory = '/home/marian/calibration_ws/monodepth-FPN/MonoDepth-FPN-PyTorch/dataset/training_data/training_data/curvature_grad/verify_network/test_orig_planes.PCD'
 
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


def clockwiseangle_and_distance(point):
    # Vector between point and the origin: v = p - o
    vector = [point[0]-origin[0], point[1]-origin[1]]
    # Length of vector: ||v||
    lenvector = math.hypot(vector[0], vector[1])
    # If length is zero there is no angle
    if lenvector == 0:
        return -math.pi, 0
    # Normalize vector: v/||v||
    normalized = [vector[0]/lenvector, vector[1]/lenvector]
    dotprod  = normalized[0]*refvec[0] + normalized[1]*refvec[1]     # x1*x2 + y1*y2
    diffprod = refvec[1]*normalized[0] - refvec[0]*normalized[1]     # x1*y2 - y1*x2
    angle = math.atan2(diffprod, dotprod)
    # Negative angles represent counter-clockwise angles so we need to subtract them 
    # from 2*pi (360 degrees)
    if angle < 0:
        return 2*math.pi+angle, lenvector
    # I return first the angle because that's the primary sorting criterium
    # but if two vectors have the same angle then the shorter distance should come first.
    return angle, lenvector


# iterate over files in
# that directory
for filename in os.scandir(directory):
    if filename.is_file() & (filename.name[-4:] == ".pcd"):
        # load pcd file
        orig_pcd = o3d.io.read_point_cloud(filename.path)
        pred_pcd = o3d.io.read_point_cloud(filename.path.replace("test_orig_planes.PCD","test_pred_planes.PCD").replace("_orig.pcd","_pred.pcd"))
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


        # transform to mesh 
        orig_downpcd = orig_plane_pcd.voxel_down_sample(voxel_size=0.0001)
        pred_downpcd = pred_plane_pcd.voxel_down_sample(voxel_size=0.0001)

        # ####################################
        # #################################### - BELOW the XY plane
        # ####################################
        orig_xyz = np.asarray(orig_downpcd.points)
        orig_xyz[:,2] = orig_xyz[:,2] - orig_xyz[:,2].max()
        # orig_plane_pcd.points = o3d.utility.Vector3dVector(orig_xyz)
        # o3d.io.write_point_cloud(filename.path[:-4]+'_vis_plane_from_measure_below.pcd', orig_plane_pcd)

        pred_xyz = np.asarray(pred_downpcd.points)
        pred_xyz[:,2] = pred_xyz[:,2] - pred_xyz[:,2].max()

        orig_xy_catalog = orig_xyz[:,:2]
        pred_xy_catalog = pred_xyz[:,:2]

        orig_tri = Delaunay(np.array(orig_xy_catalog))
        pred_tri = Delaunay(np.array(pred_xy_catalog))


        # intersecting area

        # continue from here 
        # orig = orig_tri.convex_hull
        # pred = pred_tri.convex_hull[:-10]
                        
        # orig_poly = list(map(list,orig))
        # origin = orig_poly[0]
        # refvec = [0, 1]
        # orig_clockwise = np.array(sorted(orig_poly, key=clockwiseangle_and_distance))

        # pred_poly = list(map(list,pred))
        # origin = orig_poly[0]
        # refvec = [0, 1]
        # pred_clockwise = np.array(sorted(pred_poly, key=clockwiseangle_and_distance))

        # clip = PolygonClipper(warn_if_empty = False)

        # clipped_polygon = clip(orig_clockwise,pred_clockwise)
        
        # plt.triplot(orig_xy_catalog[:,0], orig_xy_catalog[:,1], orig_tri.simplices)
        # plt.triplot(pred_xy_catalog[:,0], pred_xy_catalog[:,1], orig_tri.simplices)

        surface = o3d.geometry.TriangleMesh()
        surface.vertices = o3d.utility.Vector3dVector(orig_xyz)
        surface.triangles = o3d.utility.Vector3iVector(orig_tri.simplices)
        surface.paint_uniform_color([0, 0, 1.0])

        # o3d.io.write_triangle_mesh(filename.path[:-4]+'_vis_mesh_orig_below.ply', surface)

        volume_orig_below = reduce(lambda a, b:  a + volume_under_triangle(b), get_triangles_vertices(surface.triangles, surface.vertices), 0)
        print(f"The volume of the surface BELOW XY is: {round(volume_orig_below, 8)} m3 for ORIG file: {filename.name}")


        surface = o3d.geometry.TriangleMesh()
        surface.vertices = o3d.utility.Vector3dVector(pred_xyz)
        surface.triangles = o3d.utility.Vector3iVector(pred_tri.simplices)
        surface.paint_uniform_color([0, 1.0, 0])

        # o3d.io.write_triangle_mesh(filename.path[:-4]+'_vis_mesh_pred_below.ply', surface)

        volume_pred_below = reduce(lambda a, b:  a + volume_under_triangle(b), get_triangles_vertices(surface.triangles, surface.vertices), 0)
        print(f"The volume of the surface BELOW XY is: {round(volume_pred_below, 8)} m3 for PRED file: {filename.name}")


        # ####################################
        # #################################### - ABOVE the XY plane
        # ####################################
        orig_xyz = np.asarray(orig_downpcd.points)
        orig_xyz[:,2] = orig_xyz[:,2] - orig_xyz[:,2].min()
        pred_xyz = np.asarray(pred_downpcd.points)
        pred_xyz[:,2] = pred_xyz[:,2] - pred_xyz[:,2].min()

        orig_xy_catalog = orig_xyz[:,:2]
        pred_xy_catalog = pred_xyz[:,:2]

        orig_tri = Delaunay(np.array(orig_xy_catalog))
        pred_tri = Delaunay(np.array(pred_xy_catalog))


        # intersecting area

        # continue from here 
        # orig = orig_tri.convex_hull
        # pred = pred_tri.convex_hull[:-10]
                        
        # orig_poly = list(map(list,orig))
        # origin = orig_poly[0]
        # refvec = [0, 1]
        # orig_clockwise = np.array(sorted(orig_poly, key=clockwiseangle_and_distance))

        # pred_poly = list(map(list,pred))
        # origin = orig_poly[0]
        # refvec = [0, 1]
        # pred_clockwise = np.array(sorted(pred_poly, key=clockwiseangle_and_distance))

        # clip = PolygonClipper(warn_if_empty = False)

        # clipped_polygon = clip(orig_clockwise,pred_clockwise)
        
        # plt.triplot(orig_xy_catalog[:,0], orig_xy_catalog[:,1], orig_tri.simplices)
        # plt.triplot(pred_xy_catalog[:,0], pred_xy_catalog[:,1], orig_tri.simplices)

        surface = o3d.geometry.TriangleMesh()
        surface.vertices = o3d.utility.Vector3dVector(orig_xyz)
        surface.triangles = o3d.utility.Vector3iVector(orig_tri.simplices)
        surface.paint_uniform_color([0, 1.0, 1.0])

        # o3d.io.write_triangle_mesh(filename.path[:-4]+'_vis_mesh_orig_above.ply', surface)

        volume_orig_above = reduce(lambda a, b:  a + volume_under_triangle(b), get_triangles_vertices(surface.triangles, surface.vertices), 0)
        print(f"The volume of the surface ABOVE XY is: {round(volume_orig_above, 8)} m3 for ORIG file: {filename.name}")

        surface = o3d.geometry.TriangleMesh()
        surface.vertices = o3d.utility.Vector3dVector(pred_xyz)
        surface.triangles = o3d.utility.Vector3iVector(pred_tri.simplices)
        surface.paint_uniform_color([1.0, 1.0, 0])
        # o3d.io.write_triangle_mesh(filename.path[:-4]+'_vis_mesh_pred_above.ply', surface)

        volume_pred_above = reduce(lambda a, b:  a + volume_under_triangle(b), get_triangles_vertices(surface.triangles, surface.vertices), 0)
        print(f"The volume of the surface ABOVE XY is: {round(volume_pred_above, 8)} m3 for PRED file: {filename.name}")

        curvature_gradient = volume_orig_below/(volume_orig_below + volume_orig_above)
        print(f"The curvature gradient is: {round(curvature_gradient*100, 8)} % for ORIG file: {filename.name}")
        curvature_gradient = volume_pred_below/(volume_pred_below + volume_pred_above)
        print(f"The curvature gradient is: {round(curvature_gradient * 100, 8)} % for PRED file: {filename.name}")
        print("---------")


