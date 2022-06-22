import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv
import scipy.signal
import pdb
import sys


def main(num):

    dimg = plt.imread(
        f'/home/luca/RePAIR/dataset/group_19/rendered_o3d_full/RPf_{num:05d}_depth.png')
    cimg = plt.imread(
        f'/home/luca/RePAIR/dataset/group_19/rendered_o3d_full/RPf_{num:05d}.png')
    mask = create_mask(dimg)
    contour = create_contour(mask)
    #pdb.set_trace()
    plt.imsave(
        f'/home/luca/RePAIR/dataset/group_19/rendered_o3d_full/RPf_{num:05d}_mask.png', mask, cmap='gray')
    plt.imsave(
        f'/home/luca/RePAIR/dataset/group_19/rendered_o3d_full/RPf_{num:05d}_contour.png', contour, cmap='gray')
    plt.imsave(f'/home/luca/RePAIR/dataset/group_19/rendered_o3d_full/RPf_{num:05d}_masked.png',
               cimg * np.dstack((mask, mask, mask)))


def create_mask(depth_img):

    kernel = np.asarray([[1, 1, 1, 1, 1],
                         [1, 1, 1, 1, 1],
                         [1, 1, -24, 1, 1],
                         [1, 1, 1, 1, 1],
                         [1, 1, 1, 1, 1],
                         ])
    filt = scipy.signal.convolve2d(depth_img, kernel, mode='same')
    flat_surfaces = np.abs(filt) < 10e-4
    flat2 = np.isclose(filt, 0)
    background = np.isclose(depth_img, 0)
    foreground = depth_img > 0
    top_surface_attempt = flat_surfaces * foreground
    ccs_num, ccs = cv.connectedComponents(top_surface_attempt.astype(np.uint8))
    largest_cc_index = 0
    largest_cc_area = 0
    for j in range(1, ccs_num):
        #pdb.set_trace()
        cc_area = np.sum(np.isclose(ccs, j))
        #print(j, cc_area)
        if cc_area > largest_cc_area:
            largest_cc_index = j
            largest_cc_area = cc_area
    top_surface = np.isclose(ccs, largest_cc_index)
    return top_surface


def create_simple_mask(depth_img):
    foreground = depth_img > 0
    return foreground


def create_contour(mask):
    if np.max(mask) > 1:
        print("please provide binary mask")
        return 0

    kernel = np.asarray([[1, 1, 1],
                         [1, -8, 1],
                         [1, 1, 1]
                         ])

    filt = scipy.signal.convolve2d(mask, kernel, mode='same')
    contours = np.abs(filt) > 0
    return contours


if __name__ == '__main__':
    if len(sys.argv) > 1:
        num = int(sys.argv[1])
    else:
        num = 152
    main(num)
