# Run as: blender -b <filename> -P <this_script> -- <image_path>
import bpy, sys, os
import pdb 

# pdb.set_trace()
py_args = sys.argv[sys.argv.index("--") + 1:]
group_index = py_args.index('group')
if group_index > -1:
    group = py_args[group_index+1]
else:
    group = 1 #args.group
#pdb.set_trace()
fragments_folder = f'/media/lucap/big_data/datasets/repair/consolidated_fragments/group_{group}/facing_up'
files = os.listdir(fragments_folder)
objs = [file for file in files if file[-4:] == '.obj']
render_group_folder = f"/media/lucap/big_data/datasets/repair/rendering/retextured/group_{group}_test"
os.makedirs(render_group_folder, exist_ok=True)

for frag in objs:
    bpy.context.scene.render.filepath = os.path.join(render_group_folder, frag[:-4]+".png")
    f = os.path.join(fragments_folder, frag)
    # print(f)    
    if frag[-4:] == '.obj':
        bpy.ops.wm.obj_import(filepath=f)
    else:
        bpy.ops.import_mesh.ply(filepath=f)
    bpy.ops.render.render(write_still=True)
    #pdb.set_trace()
    
    bpy.data.objects[frag[:-4]].select_set(True)
    bpy.ops.object.delete()
    