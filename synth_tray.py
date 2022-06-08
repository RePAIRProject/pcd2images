import open3d
from visualization import VisOpen3D
import numpy as np
import matplotlib.pyplot as plt
import vedo as vd
import os, random, pdb
from utils import *

def main():
    w = 1920
    h = 1280
    f_x = 4
    f_y = 2
    output_folder = 'scenes'
    fragments_in_the_scene = f_x * f_y
    positions_x = np.linspace(-3, 3, f_x)
    positions_y = np.linspace(-1.5, 1.5, f_y)
    ps = []
    for px in positions_x:
        for py in positions_y:
            ps.append(np.array([px, py]))
    #pdb.set_trace()
    possible_scenes = 2
    variants = 5

    print(f"Generating {possible_scenes * variants} scenes with {fragments_in_the_scene} fragments..")

    frags = []
    group = '2nd'
    rp_dataset_folder = '/home/palma/Unive/RePAIR/Datasets/RePAIR_dataset'
    bg_plane = open3d.io.read_triangle_mesh(os.path.join(rp_dataset_folder, group, 'bg_plane.ply'))
    bg_plane.paint_uniform_color([0.4, 0.4, 0.4])#normal_vector = np.mean(np.asarray(bg_plane.normals), axis=0)
    # also the mean point to move the fragment to the origin
    # mean_point = np.mean(np.asarray(bg_plane.points), axis=0)
    # bg_plane.translate(-mean_point)
    # print(mean_point)
    # print(np.mean(np.asarray(bg_plane.points), axis=0))
    # # rotate with z facing up
    # #align_face_up(bg_plane, normal_vector, mean_point)
    bg_plane.scale(center=(0,0,0), scale=2)
    # open3d.visualization.draw_geometries([bg_plane])

    bg_plane.translate([0, 0, -1])


    model_folder = os.path.join(rp_dataset_folder, group, 'gt_3d_models')
    start_num = 123
    end_num = 137
    nums = np.linspace(123, 137, 15).astype(int)

    for i in range(possible_scenes):
        print(f'generating scene {i}')
        frags = []
        random.shuffle(nums)
        for j in range(fragments_in_the_scene):
            #fprint(f'reading fragment RPf_{nums[j]:05d}')
            pcd = open3d.io.read_point_cloud(os.path.join(model_folder,f'RPf_{nums[j]:05d}.ply'))
            # align face up to the center
            # segment plane (it should be the top surface)
            plane_model, inliers = pcd.segment_plane(distance_threshold=0.2,
                                                 ransac_n=5,
                                                 num_iterations=1000)
            # we take the mean value of the normals of the plane
            inlier_cloud = pcd.select_by_index(inliers)
            #inlier_cloud.paint_uniform_color([1.0, 0, 0])
            #outlier_cloud = pcd.select_by_index(inliers, invert=True)
            inlier_cloud.estimate_normals()
            normal_vector = np.mean(np.asarray(inlier_cloud.normals), axis=0)
            # also the mean point to move the fragment to the origin
            mean_point = np.mean(np.asarray(pcd.points), axis=0)
            # rotate with z facing up
            align_face_up(pcd, normal_vector, mean_point)
            pcd.scale(center=(0,0,0), scale=0.01)
            # move to a position
            pos = create_pos(ps[j][0], ps[j][1])
            pcd.translate(pos)
            frags.append(pcd)

        for k in range(variants):

            # create window
            window_visible = True

            vis = VisOpen3D(width=w, height=h, visible=window_visible)

            # point cloud
            for frag in frags:
                shake_it(frag)
                #print('adding to vis')
                vis.add_geometry(frag)
            vis.add_geometry(bg_plane)

            # update view
            # vis.update_view_point(intrinsic, extrinsic)

            # save view point to file
            # vis.save_view_point("view_point.json")
            vis.load_view_point("robot_view_point.json")


            # # capture images
            # depth = vis.capture_depth_float_buffer(show=False)
            # image = vis.capture_screen_float_buffer(show=False)
            # print(type(depth))
            # print(type(image))
            #pdb.set_trace()
            # save to file
            col_img = os.path.join(output_folder, f"scene_{i}_variants_{k}_with_{fragments_in_the_scene}_frags.png")
            vis.capture_screen_image(col_img)
            depth_img = os.path.join(output_folder, f"scene_{i}_variants_{k}_with_{fragments_in_the_scene}_frags_depth.png")
            vis.capture_depth_image(depth_img)
    #
    # np_image = np.array(image)
    # pdb.set_trace()
    # plt.imsave(np_image, 'frags_numpy.png')

    # # # draw camera
    # if window_visible:
    #     vis.load_view_point("robot_view_point.json")
    #     intrinsic = vis.get_view_point_intrinsics()
    #     extrinsic = vis.get_view_point_extrinsics()
    #     vis.draw_camera(intrinsic, extrinsic, scale=0.5, color=[0.8, 0.2, 0.8])
    #     # vis.update_view_point(intrinsic, extrinsic)
    # #
    # # if window_visible:
    #     vis.load_view_point("view_point.json")
    #     vis.run()

            del vis


if __name__ == "__main__":
    main()
