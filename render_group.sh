
# variables 
group=28
blend_scene='example_scene.blend'
output_root_folder='/media/lucap/big_data/datasets/repair/rendering/'

# align with the axis
python3 align_face_up.py --group $group

# background render with blender
blender -b $blend_scene -P blender_render.py -o $output_root_folder -- group $group
