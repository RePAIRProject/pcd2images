# Run as: blender -b <filename> -P <this_script> -- <image_path>
import bpy, sys, os
import pdb 

fragments_folder = ''
files = os.listdir(fragments_folder)
objs = [file for file in files if file[-4:] == '.obj']
for frag in objs:
    f = os.path.join(fragments_folder, frag)
    bpy.ops.import_scene.obj(filepath=f)
    bpy.ops.render.render(write_still=True)
    pdb.set_trace()