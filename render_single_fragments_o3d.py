import open3d as o3d
from visualization import VisOpen3D
import numpy as np
import matplotlib.pyplot as plt
import vedo as vd
import os, random, pdb
from utils import *

def main():
    w = 1000
    h = 1000
    group = 19
    rp_dataset_folder = '/home/luca/RePAIR/dataset/'
    #'/home/palma/Unive/RePAIR/Datasets/RePAIR_dataset'
    group_folder = os.path.join(rp_dataset_folder, f'group_{group}', 'top_surfaces')
    output_folder = os.path.join(rp_dataset_folder, f'group_{group}', 'rendered_o3d')
    if not os.path.exists(group_folder):
        print('Prepare top surfaces using segment_surfaces_for_rendering.py!')

    else:
        if not os.path.exists(output_folder):
            os.mkdir(output_folder)
        files = os.listdir(group_folder)
        frags = [file for file in files if file[-4:] == '.obj']
        print(f'in group {group} found {len(frags)} fragments (.obj)')

        #create window
        window_visible = False

        for frag in frags:
            print(frag)
            mesh = open3d.io.read_triangle_mesh(os.path.join(group_folder, frag), enable_post_processing=True)

            vis = VisOpen3D(width=w, height=h, visible=window_visible)
            vis.add_geometry(mesh)
            vis.load_view_point("single_fragment_vp.json")

            # save to file
            col_img = os.path.join(output_folder, f"{frag[:-4]}.png")
            vis.capture_screen_image(col_img)
            depth_img = os.path.join(output_folder,f"{frag[:-4]}_depth.png")
            vis.capture_depth_image(depth_img)
            print('saved', col_img)
            del vis


if __name__ == "__main__":
    main()
