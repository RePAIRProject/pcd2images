
# variables 
group=41
blend_scene='example_scene_raw.blend'
output_root_folder='/media/lucap/big_data/datasets/repair/rendering/4k'

# align with the axis
# python3 align_face_up.py --group $group

# background render with blender
blender -b $blend_scene -P blender_render.py -o $output_root_folder -- group $group
