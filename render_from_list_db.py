# Run as: blender -b <filename> -P <this_script> -- <image_path>
import bpy, sys, os
import pdb, json
import random 

with open("/media/lucap/big_data/datasets/repair/recognition/synthetic/fragments_list.json", 'r') as jf:
    source_fragments = json.load(jf)

output_dir = "/media/lucap/big_data/datasets/repair/recognition/synthetic"
render_group_folder = os.path.join(output_dir, "db")
os.makedirs(render_group_folder, exist_ok=True)


for frag in source_fragments:
    
    bpy.ops.import_scene.obj(filepath=frag['aligned_path'])
    frag_name = frag['aligned_path'].split('/')[-1][:-4]
    bpy.data.objects[frag_name].rotation_euler = [0, 0, 0]
    bpy.data.objects[frag_name].location[2] = 0.25

    # render 'classic' view 
    bpy.context.scene.render.filepath = os.path.join(render_group_folder, frag['img_name']+".png")
    bpy.ops.render.render(write_still=True)

    #pdb.set_trace()
    bpy.data.objects[frag_name].select_set(True)
    bpy.ops.object.delete()
    