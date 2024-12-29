
# variables 
group=15
blend_scene='example_scene_high_contrast_filmic_transparent.blend'
output_root_folder='output_f'

# align with the axis
# python3 align_face_up.py --group $group

# background render with blender
blender -b $blend_scene -P blender_render.py -- output $output_root_folder group $group
