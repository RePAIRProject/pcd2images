#!/usr/bin/python
# -*- coding: utf-8 -*-
import bpy, sys, os
import pdb 
import json 

# Run as: blender -b <filename> -P <this_script> -- <image_path>
for i, arg in enumerate(sys.argv):
    if arg.endswith('.blend'):
        blend_file = arg

    if arg.startswith('group'):
        group_num = arg

# Encode the filename properly
encoded_path = os.fsencode(blend_file)
decoded_path = encoded_path.decode('utf-8', 'replace')

# Save current state and open new file
# bpy.ops.wm.save_mainfile()
bpy.ops.wm.open_mainfile(filepath=decoded_path)

output_folder = f'/media/lucap/big_data/datasets/repair/ground_truth/RENDERING2D/{group_num}'
os.makedirs(output_folder, exist_ok=True)


# camera = bpy.data.cameras.new("Camera")

bpy.ops.object.light_add(type='SUN')
bpy.ops.object.camera_add()

obj_scene = bpy.data.objects

# scene
scene = bpy.data.scenes[0]
# breakpoint()

# resolution
scene.render.resolution_x = 2000
scene.render.resolution_y = 2000
scene.render.film_transparent = True
scene.eevee.use_shadows = False
scene.view_settings.look = 'AgX - Medium High Contrast'

# get the camera
for obj in obj_scene:
    if 'Camera' in obj.name:
        camera = obj
    elif 'Sun' in obj.name:
        sun = obj
    else:
        obj.hide_render = True

# breakpoint()
# add the light 
scene.camera = camera

gt = {}

# move the camera on top of one fragment
for obj in obj_scene:  
    if 'RPf' in obj.name:
        cur_frag_name = obj.name
        gt[cur_frag_name] = [obj.location[0], obj.location[1], 0]
        obj.hide_render = False
        camera.location = [obj.location[0], obj.location[1], 500]
        sun.location = [obj.location[0], obj.location[1], 500]
        for other_obj in obj_scene:  
            if 'RPf' in other_obj.name and other_obj.name != cur_frag_name:
                other_obj.hide_render = True
        bpy.context.scene.render.filepath = os.path.join(output_folder, f"{obj.name}.png")
        bpy.ops.render.render(write_still=True)

with open(os.path.join(output_folder, f'gt_{group_num}.json'), 'w') as jgt:
    json.dump(gt, jgt, indent=3)


# camera.delete()
# sun.delete()

# breakpoint()

# for frag in objs:
#     bpy.context.scene.render.filepath = os.path.join(render_group_folder, frag[:-4]+".png")
#     f = os.path.join(fragments_folder, frag)
#     # print(f)    
#     if frag[-4:] == '.obj':
#         bpy.ops.wm.obj_import(filepath=f)
#     else:
#         bpy.ops.import_mesh.ply(filepath=f)
    
#     #pdb.set_trace()
    
#     bpy.data.objects[frag[:-4]].select_set(True)
#     bpy.ops.object.delete()
    