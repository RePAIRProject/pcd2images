import open3d as o3d
from visualization import VisOpen3D
import numpy as np
import matplotlib.pyplot as plt
import vedo as vd
import os
import random
import pdb
import time
import pymeshlab


def main():
    w = 1000
    h = 1000
    group = 28
    #rp_dataset_folder = '/home/luca/RePAIR/dataset/'
    rp_dataset_folder = '/home/lucap/datasets/repair'
    #rp_dataset_folder = '/Users/Palma/Documents/Projects/Unive/RePAIR/Datasets/RePAIR_dataset'
    group_folder = os.path.join(rp_dataset_folder, f'group_{group}')
    output_folder = os.path.join(group_folder, 'only_top_surfaces')
    if not os.path.exists(output_folder):
        os.mkdir(output_folder)

    input_folder_name = 'facing_up'  # 'processed'
    files = os.listdir(os.path.join(group_folder, input_folder_name))
    frags = [file for file in files if file[-4:] == '.obj']
    print(f'in group {group} found {len(frags)} fragments (.obj)')

    ms = pymeshlab.MeshSet()
    for frag in frags:
        print(frag)
        #mesh = open3d.io.read_triangle_mesh(os.path.join(group_folder,'processed',frag), enable_post_processing=True)
        mesh_path = os.path.join(
            group_folder, input_folder_name, f'{frag[:-8]}pcl.ply')
        pcd = o3d.io.read_point_cloud(mesh_path)

        # mesh_ply = open3d.io.read_triangle_mesh(os.path.join(
        #     group_folder, 'processed', f'{frag[:-4]}.obj'))  # , enable_post_processing=True)

        ms.load_new_mesh(os.path.join(
            group_folder, input_folder_name, f'{frag[:-4]}.obj'))
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.025,
                                                 ransac_n=3,
                                                 num_iterations=1000)
        top_surface = pcd.select_by_index(inliers)
        m = ms.current_mesh()
        vertices_num = m.vertex_matrix().shape[0]
        selected_vertices = np.zeros((vertices_num))
        for j in inliers:
            selected_vertices[j] = 1
        print(f"{np.sum(selected_vertices)}/{vertices_num} ({np.sum(selected_vertices)/vertices_num*100}%) belonging to the plane")
        # o3d.visualization.draw_geometries([top_surface])
        # pdb.set_trace()
        m.add_vertex_custom_scalar_attribute(selected_vertices, 'selected')
        #pdb.set_trace()

        # new version
        ms.compute_selection_by_condition_per_vertex(condselect='selected==0')
        ms.meshing_remove_selected_vertices()
        # old version
        #ms.conditional_vertex_selection(condselect='selected==0')
        #ms.delete_selected_vertices()

        # ms.print_filter_parameter_list('conditional_vertex_selection')
        # ms.apply_filter('conditional_vertex_selection',
        #                 condselect="(selected == 1)")
        # this should save only top surface
        output_path = os.path.join(
            output_folder, f'{frag[:-4]}_meshlab')
        o3d.io.write_point_cloud(output_path + ".ply", top_surface)
        ms.save_current_mesh(output_path + ".obj")
        ms.clear()
        #pdb.set_trace()


if __name__ == "__main__":
    main()
