# Run as: blender -b <filename> -P <this_script> -- <image_path>
import bpy, sys, os
import pdb, json
import random 

with open("/media/lucap/big_data/datasets/repair/cds/Decor1_Decor2_singleView.json", 'r') as jf:
    source_fragments = json.load(jf)

output_dir = "/media/lucap/big_data/datasets/repair/cds/query/Decor1_Decor2_singleView/"
render_group_folder_classic = os.path.join(output_dir, "classic")
render_group_folder_filmic = os.path.join(output_dir, "filmic")
os.makedirs(render_group_folder_classic, exist_ok=True)
os.makedirs(render_group_folder_filmic, exist_ok=True)

noise_range = 0.1

for frag in source_fragments:
    
    bpy.ops.import_scene.obj(filepath=frag['aligned_path'])
    frag_name = frag['aligned_path'].split('/')[-1][:-4]
    bpy.data.objects[frag_name].location[2] = 0.25
    bpy.data.objects[frag_name].location[:2] = [random.uniform(-noise_range, noise_range), random.uniform(-noise_range, noise_range)]
    bpy.data.objects[frag_name].rotation_euler = [random.uniform(-noise_range, noise_range), random.uniform(-noise_range, noise_range), random.uniform(-noise_range, noise_range)]

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
    