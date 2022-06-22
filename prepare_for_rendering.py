import open3d as o3d
from visualization import VisOpen3D
import numpy as np
import matplotlib.pyplot as plt
import vedo as vd
import os
import random
import pdb
from utils import *


def main():
    w = 1000
    h = 1000
    group = 19
    rp_dataset_folder = '/home/luca/RePAIR/dataset/'
    #'/home/palma/Unive/RePAIR/Datasets/RePAIR_dataset'
    group_folder = os.path.join(rp_dataset_folder, f'group_{group}')
    output_folder = os.path.join(group_folder, 'facing_up')
    if not os.path.exists(output_folder):
        os.mkdir(output_folder)

    files = os.listdir(os.path.join(group_folder, 'processed'))
    frags = [file for file in files if file[-4:] == '.obj']
    print(f'in group {group} found {len(frags)} fragments (.obj)')

    #create window
    window_visible = False

    for frag in frags:
        print(frag)
        mesh = open3d.io.read_triangle_mesh(os.path.join(
            group_folder, 'processed', frag), enable_post_processing=True)
        pcd = open3d.io.read_point_cloud(os.path.join(
            group_folder, 'processed', f'{frag[:-4]}.ply'))
        mesh_ply = open3d.io.read_triangle_mesh(os.path.join(
            group_folder, 'processed', f'{frag[:-4]}.ply'), enable_post_processing=True)
        print("pcd", pcd)
        print("mesh", mesh_ply)
        pdb.set_trace()
        # o3d.visualization.draw_geometries([mesh])
        # pdb.set_trace()
        # align face up to the center
        # segment plane (it should be the top surface)
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.05,
                                                 ransac_n=3,
                                                 num_iterations=10000)
        # we take the mean value of the normals of the plane
        inlier_cloud = pcd.select_by_index(inliers)
        #inlier_cloud.paint_uniform_color([1.0, 0, 0])
        #outlier_cloud = pcd.select_by_index(inliers, invert=True)
        inlier_cloud.estimate_normals()
        normal_vector = np.mean(np.asarray(inlier_cloud.normals), axis=0)
        # also the mean point to move the fragment to the origin
        mean_point = np.mean(np.asarray(pcd.points), axis=0)
        pdb.set_trace()
        # rotate with z facing up
        align_face_up(mesh, normal_vector, mean_point)
        mesh.scale(center=(0, 0, 0), scale=0.01)
        max_bounds = mesh.get_max_bound()
        mesh.translate(np.array([0, 0, -max_bounds[2]]))
        open3d.io.write_triangle_mesh(os.path.join(
            output_folder, f'{frag[:-4]}.obj'), mesh)
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
