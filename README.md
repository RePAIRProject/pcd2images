# pcd2images

Code to render image of the top flat *textured* surface of a fresco fragment from its 3D pointcloud (having both .obj with high-res texture and ply for math operation and alignment).

## WARNING: work in progress, it may not always work as intended. At the moment is relying on having a large top flat surface, otherwise it may fails. 

The workflow is divided into three python scripts.

**Note:** remember to change the path, it is hard-coded.

## 1. `python align_face_up.py`
It reads both .ply and .obj and tries to align them on the z-axis (if you open them on meshlab, they should be perpendicular, so you see the top surface). 
Doable with vedo or open3d (slightly different methods). Does not segment anything, only aligns.
Helpful to actually visualize the meshes afterwards to check if it worked.

## 2. `python segment_plane.py`
It segments a plane and cut it from the rest. Done using segment plane method from open3d, not perfect. 

## 3. `python render_single_fragment_o3d.py`
It creates a virtual camera at a given position (specified in the `single_fragment_vp.json` file) and renders image and depth (and also creates mask contour if needed).


# Acknowledgements:
The rendering script is taken from the well-documented [render_depthmap_example](https://github.com/pablospe/render_depthmap_example) from [pablospe](https://github.com/pablospe), thanks.
