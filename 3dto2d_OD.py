import os
import natsort
import glob
import vedo
import copy
import numpy as np
import json
import jsbeautifier

import open3d as o3d

def save_json_file(filename, metadata):
    opts = jsbeautifier.default_options()
    opts.indent_size = 4
    data = jsbeautifier.beautify(json.dumps(metadata, sort_keys=True), opts)

    with open(filename, 'w') as fp:
        fp.write(data)

# def create_pcds(vd_pcds):
#     pcds_ = []
#     for vd_pcd in vd_pcds:
#         pcd = o3d.geometry.PointCloud()
#         pcd.points = o3d.utility.Vector3dVector(vd_pcd.points)
#         pcd.colors = o3d.utility.Vector3dVector(vd_pcd.pointcolors[:, 0:3]/255)
#
#     #     pcd.normals = mesh_model.vertex_normals
#     #     pcd.paint_uniform_color([np.round(np.random.uniform(low=0, high=1), 1), np.round(np.random.uniform(low=0, high=1), 1), np.round(np.random.uniform(low=0, high=1), 1)]) # give each point cloud a random color
#         pcds_.append(pcd)
#
#     # if VISUALIZE:
#     #     o3d.visualization.draw_geometries(pcds_)  # plot A and B
#
#     return pcds_
#
# def vedo2open3d(vd_mesh):
#     """
#     Return an `open3d.geometry.TriangleMesh` version of
#     the current mesh.
#
#     Returns
#     ---------
#     open3d : open3d.geometry.TriangleMesh
#       Current mesh as an open3d object.
#     """
#
#     if isinstance(vd_mesh, vedo.Mesh):
#
#         # create from numpy arrays
#         o3d_mesh = o3d.geometry.TriangleMesh(
#             vertices=o3d.utility.Vector3dVector(vd_mesh.points),
#             triangles=o3d.utility.Vector3iVector(vd_mesh.cells))
#
#         # I need to add some if check here in case color and normals info are not existing
#         # o3d_mesh.vertex_colors = o3d.utility.Vector3dVector(vd_mesh.pointdata["RGB"]/255)
#         # o3d_mesh.vertex_normals = o3d.utility.Vector3dVector(vd_mesh.pointdata["Normals"])
#         return o3d_mesh
#     else:
#         o3d_pcd = o3d.geometry.PointCloud()
#         o3d_pcd.points = o3d.utility.Vector3dVector(vd_mesh)
#
#         return o3d_pcd

# def get_cutplane(mesh):
#     ## requires open3d-0.19.0
#
#     mesh.compute_triangle_normals()
#     mesh.compute_vertex_normals()
#
#     # Define reference normal (unit vector)
#     ref_normal = np.array([0, 0, 1])  # E.g., upward-facing normals
#
#     # Tolerance in degrees
#     tolerance_deg = 30
#     tolerance_rad = np.deg2rad(tolerance_deg)
#
#     # Get normals as numpy array
#     normals = np.asarray(mesh.vertex_normals)
#
#     # Compute dot product with reference normal
#     dot_products = normals @ ref_normal
#
#     # Clamp values to avoid floating point issues with arccos
#     dot_products = np.clip(dot_products, -1.0, 1.0)
#
#     # Compute angles between normals and reference
#     angles = np.arccos(dot_products)
#
#     # Filter indices within tolerance
#     mask = angles < tolerance_rad
#     filtered_indices = np.where(mask)[0]
#
#     # Select filtered point cloud
#     filtered_pcd = mesh.select_by_index(filtered_indices)
#
#     # Visualize (optional)
#     o3d.visualization.draw_geometries([filtered_pcd])
#
#
#     pcd = mesh.sample_points_uniformly(number_of_points=100000, use_triangle_normal=True)
#
#     plane_model, inliers = pcd.segment_plane(distance_threshold=1., ransac_n=3, num_iterations=1000, probability=1.0)
#
#     [a, b, c, d] = plane_model
#     # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
#
#     inlier_cloud = pcd.select_by_index(inliers)
#     bbox2 = inlier_cloud.get_oriented_bounding_box(robust=True)
#
#     inlier_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.005 * 2, max_nn=100))
#
#     # np_pts = np.asarray(bbox.get_box_points())
#     # np_pts[::2, 2] += 10
#     # bbox2 = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(np_pts), robust=True)
#
#     inlier_cloud.paint_uniform_color([1.0, 0, 0])
#     outlier_cloud = pcd.select_by_index(inliers, invert=True)
#     outlier_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.005 * 2, max_nn=100))
#     outlier_cloud.paint_uniform_color([0, 0, 1.0])
#
#     # Compute the centroid of the inlier points
#     center_point = np.mean(inlier_cloud.points, axis=0) - [0., 0., 1.]
#     normal = np.array([a, b, c])
#
#     # Create the plane in vedo using the center as anchor
#     plane = vedo.Plane(pos=center_point, normal=normal, s=(350,350), c='green', alpha=0.5)
#
#     # vbox = vedo.Box(pos=bbox2.get_center(), length=bbox2.extent[1], width=bbox2.extent[0], height=bbox2.extent[2])
#     vbox = vedo.Box(np.column_stack((bbox2.get_min_bound(), bbox2.get_max_bound())).flatten())
#
#     return center_point, normal

# parameters to modify for affecting the output
# # Set the tolerance angle for the surface cut plane, the smaller the angle, the more aggressive, the more faces are removed
tol_angle = 60
# Set the height offset of the plane where everything below is gonna be removed
z_offset = 1.5

# HARD CODED PATH
input_folder_ = '/run/user/1000/gvfs/sftp:host=gpu1.dsi.unive.it,user=luca.palmieri/home/ssd/datasets/RePAIR_v2/2_Exported_OBJ/OPEN_DISCOVERY'
 # '/home/lucap/code/RePair_3D_new/PUZZLES/SOLVED'

def main():
    global z_offset
    # Iterate over all folders
    # for f in os.scandir('/media/pose_est_rp/repair_gt/3D_Fragments/assembled_objects/'):
    # for f in os.scandir('/media/pose_est_rp/Solved/3D_fragments/'):
    puzzles = os.listdir(input_folder_)
    puzzles_sorted = natsort.natsorted(puzzles)
    print("Full list of puzzles:\n")
    for puz in puzzles_sorted:
        print(puz)
    # breakpoint()
    for j, folder_name in enumerate(puzzles_sorted):
        print("-" * 50)
        print(folder_name)      

        # if f.name is not "puzzle_0000003_RP_group_3":
        #     continue

        # input_folder = '/media/pose_est_rp/repair_gt/3D_Fragments/assembled_objects/'+f.name+'/'
        input_folder = os.path.join(input_folder_, folder_name) + '/'
        output_folder = input_folder.replace("2_Exported_OBJ", "3_Rendered_2D")
        # output_folder = output_folder.replace("SOLVED", "SOLVED_Updated")

        if os.path.exists(output_folder):
            continue

        print(f"\nWe use:\n\ttol_angle: {tol_angle}\n\tz_offset: {z_offset}")

        os.makedirs(output_folder, exist_ok=True)
        mesh_files = natsort.natsorted(glob.glob(input_folder + "*.obj"))
        texture_files = natsort.natsorted(glob.glob(input_folder + "*.png"))
        print(f"Found {len(mesh_files)} meshes and\n{len(texture_files)} textures.")
        print(f"Will save in {output_folder}")

        # Create a list to store mesh images and their positions
        fragments = []

        # Load and project meshes
        meshes_projected = []
        filenames = []

        x0m, x1m, y0m, y1m = (1e10, 0, 1e10, 0)
        for i, (mesh_file, texture_file) in vedo.progressbar(enumerate(zip(mesh_files, texture_files))):
            m = vedo.Mesh(mesh_file).texture(texture_file).lighting("off")
            m.rotate_x(90)
            # if i == 6:
            #     z_offset = 2.
            #     m = set_cutplane(m, tol_angle)
            # elif i == 11:
            #     z_offset = .1
            #     m = set_cutplane(m, tol_angle)
            # elif i == 12:
            #     z_offset = .8
            #     m = set_cutplane(m, tol_angle)
            # elif i == 1:
            #     z_offset = .8
            #     m = set_cutplane(m, tol_angle)
            # elif i == 9:
            #     z_offset = .8
            #     m = set_cutplane(m, tol_angle)
            # elif i == 4:
            #     z_offset = .5
            #     m = set_cutplane(m, tol_angle)
            # else:
            #     z_offset = 1.
            #     m = set_cutplane(m, tol_angle)

            m = set_cutplane(m, tol_angle)

            # o3d_mesh = vedo2open3d(m)
            # # o3d.visualization.draw_geometries([o3d_mesh])
            # center_point, normal = get_cutplane(o3d_mesh)
            # # cutting_ids = m.find_cells_in_bounds(cutting_box.bounds())
            # # m.cut_with_box(get_cutplane_bbox(cutting_box)) # this solution creates artifacts on the surface of the mesh, look at https://github.com/marcomusy/vedo/issues/856
            # # m.delete_cells(cutting_ids)
            # ids = m.find_cells_in_bounds(zbounds=(-10, x))
            # m.cut_with_plane(origin=center_point, normal=normal)
            mx = m.project_on_plane("z").alpha(1)
            meshes_projected.append(mx)

            x0, x1, y0, y1, _, _ = m.bounds()
            x0m = min(x0m, x0)
            x1m = max(x1m, x1)
            y0m = min(y0m, y0)
            y1m = max(y1m, y1)

            # Store the image and its position in the unified space
            fragments.append({
                'idx': i,
                'filename': mesh_file.split('/')[-1],
                'position': mx.center_of_mass().tolist(),
                'tol_angle': tol_angle,
            })
            filenames.append((mesh_file.split('/')[-1]).split('.')[0])

        mesh_data = {'fragments': fragments}
        # mesh_data['tol_angle'] = tol_angle

        box = vedo.Box([x0m, x1m, y0m, y1m, 0, 0]).alpha(0)
        # breakpoint()
        
        
        
        # sizex = 2000
        # sizey = 2000 #(y1m - y0m) * sizex / (x1m - x0m)

        x_range = x1m - x0m
        y_range = y1m - y0m

        # Scale factor to fit in sizex/sizey canvas
        max_dimension = max(x_range, y_range)
        scale_factor = 6
        sizex = max_dimension * scale_factor
        sizey = max_dimension * scale_factor
        # breakpoint()
        # scale_factor_x = sizex / max_dimension
        # scale_factor_y = sizex / max_dimension

        # Create a LinearTransform object for mapping pixels to cartesian coordinates
        tform = vedo.LinearTransform()
        # Inverse the scale factor to map back to original coordinates
        tform.scale(1/scale_factor)

        # Calculate position for placement
        pos_x, pos_y, _ = box.center_of_mass()
        place_x = pos_x - (sizex / 2) / scale_factor
        place_y = pos_y - (sizey / 2) / scale_factor
        tform.translate([place_x, place_y, 0])

        mesh_data['transform'] = tform.matrix.tolist()

        # breakpoint()
        # 2D pixel data
        for j, frag in enumerate(fragments):
            frag_data = mesh_data['fragments'][j]
            reprojected = vedo.Points(np.asarray([frag_data['position']]), c='green', r=10).apply_transform(np.linalg.inv(tform.matrix)).points[0]
            reprojected[1] = sizey - reprojected[1]
            frag_data['pixel_position'] = reprojected.tolist()

        # # Create adjacency matrix for connected pieces
        # tiles = copy.deepcopy(meshes_projected)
        # dists = {}
        # for i, tile1 in enumerate(tiles):
        #     tile1.subsample(0.02)
        #     for j, tile2 in enumerate(tiles):
        #         if i <= j:
        #             continue
        #         dist = np.min(tile1.distance_to(tile2))
        #         dists[(i, j)] = dist

        # # Find connections between pieces
        # connections = []
        # lines = []
        # for i, j in dists:
        #     if dists[(i, j)] < 8:  # Threshold for connection
        #         connections.append((i, j))
        #         line = vedo.Line(meshes_projected[i].center_of_mass(), meshes_projected[j].center_of_mass()).lw(4)
        #         lines.append(line)

        # mesh_data['adjacency'] = connections

        vedo.settings.screenshot_transparent_background = True
        vedo.settings.use_parallel_projection = True

        plotter = vedo.Plotter(offscreen=True, size=(sizex, sizey))

        # Convert dictionary to a JSON string and write to file
        save_json_file(output_folder+'data.json', mesh_data)

        # Process each mesh separately to create individual images
        texts = []
        for i, mesh in enumerate(meshes_projected):

            texts.append(vedo.Text3D(str(i), pos=mesh.center_of_mass(), c='black', s=10))
            plotter.show([box, meshes_projected[i]], zoom="tightest")
            plotter.screenshot(output_folder+"{}.png".format(filenames[i]))

            # Clear plotter and for the next mesh
            plotter.clear()

        # plotter.show([box, meshes_projected], zoom="tightest")
        # plotter.screenshot(output_folder + "preview.png")
        # plotter.clear()
        # plotter.show([box, meshes_projected, lines, texts], zoom="tightest")
        # plotter.screenshot(output_folder + "adjacency_preview.png")
        # plotter.clear()
        # plotter.close()


        # breakpoint()
        # # Code to evaluate the saved images and whether they are correct
        # image_files = natsort.natsorted(glob.glob(output_folder + "*.png"))
        # image_files = list(filter(lambda k: 'preview' not in k, image_files))
        
        # # Load saved images
        # images = []
        # images_transformed = []
        # for i, image_file in vedo.progressbar(enumerate(image_files)):
        #     img = vedo.Image(image_file, channels=4)
        #     img_transformed = vedo.Image(image_file, channels=4).apply_transform(tform)
        #     # img_transformed = img.clone().apply_transform(tform)
        #     images.append(img)
        #     images_transformed.append(img_transformed)
        
        # # check if the screenshots are correct
        # plt = vedo.Plotter(size=(sizex, sizey))
        # # Without transformation
        # vedo.show(meshes_projected, images, axes=1, zoom="tightest", interactive=True).close()
        # plt.close()
        
        # # With transformation
        # vedo.show([meshes_projected, images_transformed], N=2, axes=1, zoom="tightest", interactive=True).close()
        # breakpoint()

    return 0


def set_cutplane(m, tol_angle=30):
    # Ensure normals are available
    m.compute_normals()

    # Reference normal and tolerance
    ref_normal = np.array([0, 0, 1])
    tolerance_deg = tol_angle
    tolerance_rad = np.deg2rad(tolerance_deg)

    # Get normals and compute angles
    normals = m.cell_normals
    # normals = m.point_normals
    dot_products = normals @ ref_normal
    dot_products = np.clip(dot_products, -1.0, 1.0)
    angles = np.arccos(dot_products)

    # Mask points whose normals are within the angular tolerance
    mask = angles < tolerance_rad
    ids = np.where(mask)[0]
    m = m.extract_cells(ids)

    # mask = angles > tolerance_rad
    # ids = np.where(mask)[0]
    # m.delete_cells(ids)
    # m.delete_cells_by_point_index(ids)

    test_m = m.clone().clean().extract_largest_region()

    ids_ = test_m.inside_points(m.points, invert=True, tol=0.9, return_ids=True)

    # # Create an inverted mask instead of calling inside_points(invert=True)
    # mask = np.ones(test_m.points.shape[0], dtype=bool)
    # mask[ids] = False
    # ids = np.where(mask)[0]
    m.delete_cells_by_point_index(ids_)

    # test_m.split(flag=True)
    #
    # # Remove small isolated mesh islands
    # test_m.compute_connections()
    # cc = test_m.connected_components()
    # # Keep only largest component (or components above size threshold)
    # main_component = cc.sort_cells_by_size()[0]  # Largest
    # main_component.show()
    #
    # # # Parameters
    # # radius = 3.05
    # # min_neighbors = 25
    # #
    # # # KD-tree search
    # # from scipy.spatial import cKDTree
    # # tree = cKDTree(test_m.points)
    # # neighbors = tree.query_ball_point(test_m.points, r=radius)
    # # mask_pnts = [len(n) <= min_neighbors for n in neighbors]
    # # ids_ = np.where(mask_pnts)[0]
    # #
    # # test_m.delete_cells_by_point_index(ids_)
    #
    # o3d_mesh = vedo2open3d(m.clone().clean().points)
    # o3d.visualization.draw_geometries([o3d_mesh])
    #
    # print("Statistical oulier removal")
    # o3d_mesh_t = o3d.t.geometry.PointCloud.from_legacy(o3d_mesh)
    # # cl, ind = o3d_mesh_t.remove_statistical_outliers(nb_neighbors=20, std_ratio=2.0)
    # cl, ind = o3d_mesh_t.remove_radius_outliers(nb_points=56, search_radius=1.)
    #
    # o3d.visualization.draw_geometries([o3d_mesh_t.to_legacy()])
    #
    # # display_inlier_outlier(voxel_down_pcd, ind)

    # plane = get_plane(m)

    return cut_with_plane(m)

def cut_with_plane(vedo_pcd):

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(vedo_pcd.points)
    # pcd.colors = o3d.utility.Vector3dVector(vedo_pcd.pointcolors[:, 0:3]/255)
    pcd.paint_uniform_color([0.5, 0.5, 0.5])

    plane_model, inliers = pcd.segment_plane(distance_threshold=1, ransac_n=100, num_iterations=1000)
    [a, b, c, d] = plane_model
    # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    # outlier_cloud = pcd.select_by_index(inliers, invert=True)

    offset = np.array([0,0,z_offset])
    pos = (np.mean(np.asarray(inlier_cloud.points), axis=0) - offset).tolist()
    normal = [a,b,c]
    plane = vedo.Plane(pos, normal, s=(300,300)).c("green4")

    normal_ = normal / np.linalg.norm(normal)
    vectors = vedo_pcd.points - pos
    dot_products = np.dot(vectors, normal_)
    ids = np.where(dot_products < 0)[0]

    # mask = vedo_pcd.points[:,2] < pos[2]
    # ids = np.where(mask)[0]
    vedo_pcd.delete_cells_by_point_index(ids)

    return vedo_pcd


if __name__ == "__main__":
    main()