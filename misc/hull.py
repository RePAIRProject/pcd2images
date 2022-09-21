import cv2
import numpy as np
import random as rng
import pdb
mask = cv2.imread('/home/luca/RePAIR/dataset/group_19/rendered_o3d_mask/RPf_00152_meshlab_mask.png', 0)
cont = cv2.imread('/home/luca/RePAIR/dataset/group_19/rendered_o3d_mask/RPf_00152_meshlab_contour.png')

ret, thrmask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
contours, hierarchy = cv2.findContours(thrmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
pdb.set_trace()
drawing = np.zeros((mask.shape[0], mask.shape[1], 3), np.uint8)
# for i in range(len(contours)):
#     color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
#     cv2.drawContours(drawing, contours, i, color)

cv2.drawContours(drawing, contours, -1, (255, 255, 255), 3)
cv2.imwrite('/home/luca/RePAIR/dataset/group_19/rendered_o3d_mask/RPf_00152_meshlab_contourcv.png', drawing)
