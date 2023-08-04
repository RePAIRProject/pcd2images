# Run as: blender -b <filename> -P <this_script> -- <image_path>
import bpy, sys, os
import pdb, json
import random 

with open("/media/lucap/big_data/datasets/repair/cds/db/Decor1_Decor2_singleView.json", 'r') as jf:
    source_fragments = json.load(jf)

noise_level = 'low'
if noise_level == 'without':
    location_noise = 0
    rotation_xy_noise = 0
    rotation_z_noise = 0
elif noise_level == 'low':
    location_noise = 0.05
    rotation_xy_noise = 0.08726646 # +/- 5 degrees in rad
    rotation_z_noise = 0.78539816 # +/- 45 degrees in rad
elif noise_level == 'high':
    location_noise = 0.1
    rotation_xy_noise = 0.52359878 # +/-30 degrees in rad
    rotation_z_noise = 1.57 # +/- pi / 2
output_dir = f"/media/lucap/big_data/datasets/repair/recognition/synthetic/query_{noise_level}_noise"
render_group_folder_classic = os.path.join(output_dir, "classic")
render_group_folder_filmic = os.path.join(output_dir, "filmic")
os.makedirs(render_group_folder_classic, exist_ok=True)
os.makedirs(render_group_folder_filmic, exist_ok=True)

for frag in source_fragments:
    
    bpy.ops.import_scene.obj(filepath=frag['aligned_path'])
    frag_name = frag['aligned_path'].split('/')[-1][:-4]
    bpy.data.objects[frag_name].rotation_euler = [random.uniform(-rotation_xy_noise, rotation_xy_noise), random.uniform(-rotation_xy_noise, rotation_xy_noise), random.uniform(-rotation_z_noise, rotation_z_noise)]
    bpy.data.objects[frag_name].location[2] = 0.25
    bpy.data.objects[frag_name].location[:2] = [random.uniform(-location_noise, location_noise), random.uniform(-location_noise, location_noise)]
    
    # render 'classic' view 
    bpy.context.scene.render.filepath = os.path.join(render_group_folder_classic, frag['img_name']+".png")
    bpy.ops.render.render(write_still=True)

    # render 'filmic' view
    bpy.context.scene.render.filepath = os.path.join(render_group_folder_filmic, frag['img_name']+".png")
    bpy.data.images[f"{frag_name}_0.png"].colorspace_settings.name = 'Filmic sRGB'
    bpy.ops.render.render(write_still=True)

    #pdb.set_trace()
    bpy.data.objects[frag_name].select_set(True)
    bpy.ops.object.delete()
    