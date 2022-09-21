import open3d as o3d
from visualization import VisOpen3D
import numpy as np
import matplotlib.pyplot as plt
import vedo as vd
import os, random, pdb
from utils import *
from sklearn.cluster import KMeans

def extract_features(pcd, voxel_size):
    # print(":: Downsample with a voxel size %.3f." % voxel_size)
    # pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd, pcd_fpfh

def segment(pcd, features):
    X = np.transpose(features.data)
    labels = KMeans(n_clusters=2).fit_predict(X)
    grp0 = np.where(labels==0)
    grp1 = np.where(labels==1)
    sub_pcd0 = pcd.select_by_index(grp0[0])
    sub_pcd1 = pcd.select_by_index(grp1[0])
    sub_pcd0.estimate_normals()
    sub_pcd1.estimate_normals()
    std0 = np.std(np.asarray(sub_pcd0.normals), axis=0)
    std1 = np.std(np.asarray(sub_pcd1.normals), axis=0)
    if np.argmin(np.concatenate([std0, std1])) < 3:
        intactcloud = sub_pcd0
        intact_indices = grp0[0]
        fracturedcloud = sub_pcd1
        fractured_indices = grp1[0]
    else:
        intactcloud = sub_pcd1
        intact_indices = grp1[0]
        fracturedcloud = sub_pcd0
        fractured_indices = grp0[0]
    return intactcloud, fracturedcloud, intact_indices, fractured_indices

def main():
    w = 1000
    h = 1000
    group = 19
    rp_dataset_folder = '/home/luca/RePAIR/dataset/'
    rp_dataset_folder = '/home/palma/Unive/RePAIR/Datasets/RePAIR_dataset'
    group_folder = os.path.join(rp_dataset_folder, f'group_{group}')
    output_folder = os.path.join(group_folder, 'only_surfaces')
    if not os.path.exists(output_folder):
        os.mkdir(output_folder)

    files = os.listdir(os.path.join(group_folder, 'processed'))
    frags = [file for file in files if file[-4:] == '.obj']
    print(f'in group {group} found {len(frags)} fragments (.obj)')

    #create window
    window_visible = False
    clean_mesh = False
    voxel_size = 0.5
    for frag in frags:
        print(frag)
        #mesh = open3d.io.read_triangle_mesh(os.path.join(group_folder,'processed',frag), enable_post_processing=True)
        pcd = open3d.io.read_point_cloud(os.path.join(group_folder,'processed',f'{frag[:-4]}.ply'))
        mesh_ply = open3d.io.read_triangle_mesh(os.path.join(group_folder,'processed',f'{frag[:-4]}.ply'), enable_post_processing=True)
        # extract features
        pcd, features = extract_features(pcd, voxel_size=voxel_size)
        # cluster to divide
        intact, fractured, int_ind, fract_ind = segment(pcd, features)
        mesh_ply_intact = mesh_ply.select_by_index(int_ind, cleanup=False)

        # print(mesh_ply_intact, intact)
        # open3d.io.write_triangle_mesh(os.path.join(output_folder,f'{frag[:-4]}_intact_{voxel_size}.obj'), mesh_ply_intact)
        # open3d.io.write_point_cloud(os.path.join(output_folder,f'{frag[:-4]}_intact_{voxel_size}.ply'), intact)

        # segement plane
        plane_model, inliers = intact.segment_plane(distance_threshold=0.35,
                                                         ransac_n=3,
                                                         num_iterations=1000)
        top_surface = intact.select_by_index(inliers)
        mesh_ply_top_surf = mesh_ply_intact.select_by_index(inliers, cleanup=False)
        # print(mesh_ply_top_surf, top_surface)

        # clean up
        if clean_mesh:
            mesh_ply_top_surf.remove_unreferenced_vertices()
            mesh_ply_top_surf.remove_degenerate_triangles()


        #print("pcd", pcd)
        #print("mesh", mesh_ply)
        #pdb.set_trace()
        # o3d.visualization.draw_geometries([mesh])
        # pdb.set_trace()
        # align face up to the center
        # segment plane (it should be the top surface)
        # plane_model, inliers = pcd.segment_plane(distance_threshold=0.05,
        #                                      ransac_n=3,
        #                                      num_iterations=10000)
        # # we take the mean value of the normals of the plane
        # inlier_cloud = pcd.select_by_index(inliers)
        # #inlier_cloud.paint_uniform_color([1.0, 0, 0])
        # #outlier_cloud = pcd.select_by_index(inliers, invert=True)
        top_surface.estimate_normals()
        normal_vector = np.mean(np.asarray(top_surface.normals), axis=0)
        # also the mean point to move the fragment to the origin
        mean_point = np.mean(np.asarray(top_surface.points), axis=0)
        #pdb.set_trace()
        # rotate with z facing up
        align_face_up(mesh_ply_top_surf, normal_vector, mean_point)
        mesh_ply_top_surf.scale(center=(0,0,0), scale=0.01)
        max_bounds = mesh_ply_top_surf.get_max_bound()
        mesh_ply_top_surf.translate(np.array([0, 0, -max_bounds[2]]))

        open3d.io.write_triangle_mesh(os.path.join(output_folder,f'{frag[:-4]}_top_surface_vs{voxel_size}.obj'), mesh_ply_top_surf)
        open3d.io.write_point_cloud(os.path.join(output_folder,f'{frag[:-4]}_top_surface_vs{voxel_size}.ply'), top_surface)
        #open3d.io.write_triangle_mesh(os.path.join(output_folder,f'{frag[:-4]}.obj'), mesh)
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
