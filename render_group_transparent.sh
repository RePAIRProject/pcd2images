
# variables 
group=3
blend_scene='example_scene_high_contrast_filmic_transparent.blend'
#output_root_folder='/media/lucap/big_data/datasets/repair/rendering/4k'

# align with the axis
# python3 align_face_up.py --group $group

# background render with blender
blender -b $blend_scene -P blender_render_consolidated.py -- group $group
