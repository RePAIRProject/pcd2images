import open3d as o3d
from visualization import VisOpen3D
import numpy as np
import matplotlib.pyplot as plt
import vedo as vd
import os
import random
import pdb
from utils import *


def get_pcl_from_mesh(mesh, est_normals=False):
    pcl = o3d.geometry.PointCloud()
    pcl.points = o3d.utility.Vector3dVector(np.asarray(mesh.vertices))
    if est_normals:
        pcl.estimate_normals()
    return pcl

def main():
    w = 1000
    h = 1000
    group = 28
    #rp_dataset_folder = '/home/luca/RePAIR/dataset/'
    rp_dataset_folder = '/home/lucap/datasets/repair'
    group_folder = os.path.join(rp_dataset_folder, f'group_{group}')
    output_folder = os.path.join(group_folder, 'facing_up')
    if not os.path.exists(output_folder):
        os.mkdir(output_folder)

    files = os.listdir(os.path.join(group_folder, 'processed'))
    frags = [file for file in files if file[-4:] == '.obj']
    print(f'in group {group} found {len(frags)} fragments (.obj)')

    #create window
    window_visible = False

    debug = False
    for frag in frags:
        print(frag)
        mesh = o3d.io.read_triangle_mesh(os.path.join(
            group_folder, 'processed', frag), enable_post_processing=True)
        pcd = get_pcl_from_mesh(mesh, est_normals=True)

        # align face up to the center
        # segment plane (it should be the top surface)
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.75,
                                                 ransac_n=3,
                                                 num_iterations=10000)

        # we take the mean value of the normals of the plane
        inlier_cloud = pcd.select_by_index(inliers)

        # select only center ones
        bbox = inlier_cloud.get_oriented_bounding_box()
        bbox_s = bbox.scale(0.8, bbox.center)
        plane_cloud = inlier_cloud.crop(bbox_s)
        plane_cloud.orient_normals_to_align_with_direction(np.array([bbox.center[0], bbox.center[1], bbox.center[2]*2]))
        normal_vector = np.mean(np.asarray(plane_cloud.normals), axis=0)

        #pdb.set_trace()
        if debug:
            #inlier_cloud.paint_uniform_color([1.0, 0, 0])
            plane_cloud.paint_uniform_color([1.0, 0, 0])
            o3d.visualization.draw_geometries([plane_cloud])
            #outlier_cloud = pcd.select_by_index(inliers, invert=True)

        # also the mean point of the initial pcl to move the fragment to the origin
        #pdb.set_trace()
        mean_point = np.mean(np.asarray(pcd.points), axis=0)
        # rotate with z facing up
        align_face_up(mesh, normal_vector, mean_point)
        align_face_up(pcd, normal_vector, mean_point)
        #print("mesh", mesh)
        mesh.scale(center=(0, 0, 0), scale=0.01)
        pcd.scale(center=(0, 0, 0), scale=0.01)
        #print("mesh", mesh)
        max_bounds = mesh.get_max_bound()
        mesh.translate(np.array([0, 0, -max_bounds[2]]))
        pcd.translate(np.array([0, 0, -max_bounds[2]]))
        #print("mesh", mesh)
        #pdb.set_trace()
        o3d.io.write_point_cloud(os.path.join(
            output_folder, f'{frag[:-4]}_pcl.ply'), pcd)
        o3d.io.write_triangle_mesh(os.path.join(
            output_folder, f'{frag[:-4]}_mesh.obj'), mesh)
        #pdb.set_trace()
        # vis = VisOpen3D(width=w, height=h, visible=window_visible)
        # vis.add_geometry(mesh)
        # vis.load_view_point("single_fragment_vp.json")
        #
        # # save to file
        # col_img = os.path.join(output_folder, f"{frag[:-4]}.png")
        # vis.capture_screen_image(col_img)
        # depth_img = os.path.join(output_folder,f"{frag[:-4]}_depth.png")
        # vis.capture_depth_image(depth_img)
        # print('saved', col_img)
        # del vis


if __name__ == "__main__":
    main()
