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

def main():

    output_folder = '/home/lucap/code/RePair_3D_new/PUZZLES_2D_scale3/SOLVED/puzzle_0000001_RP_group_1/'
    # Code to evaluate the saved images and whether they are correct
    image_files = natsort.natsorted(glob.glob(output_folder + "*.png"))
    image_files = list(filter(lambda k: 'preview' not in k, image_files))

    with open(output_folder+'data.json') as f:
        data = json.load(f)

    tform = np.asarray(data["transform"])

    # Load saved images
    images = []
    images_transformed = []
    pnts = []
    sizex = sizey = None
    for i, image_file in vedo.progressbar(enumerate(image_files)):
        img = vedo.Image(image_file, channels=4)
        img_transformed = vedo.Image(image_file, channels=4).apply_transform(tform)
        # img_transformed = img.clone().apply_transform(tform)
        images.append(img)
        images_transformed.append(img_transformed)

        pnts.append(np.asarray(data["fragments"][i]["position"]))

        pixels = vedo.Points(np.asarray(pnts), c='green', r=10).apply_transform(np.linalg.inv(tform)).points
        breakpoint()

        if not sizex or not sizey:
            sizex, sizey = img.shape

    # check if the screenshots are correct
    plt = vedo.Plotter(size=(sizex, sizey))
    # plot images before and after transformation
    vedo.show(images, images_transformed, vedo.Points(np.asarray(pnts), c='red', r=6), vedo.Points(np.asarray(pnts), c='green', r=10).apply_transform(np.linalg.inv(tform)), axes=1, zoom="tightest", interactive=True).close()
    plt.close()
    return 0

if __name__ == "__main__":
    main()