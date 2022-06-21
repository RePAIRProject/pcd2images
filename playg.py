import open3d as o3d
import numpy as np
import pdb
import pymeshlab

mesh_path = '/home/luca/RePAIR/dataset/group_19/processed/RPf_00152.obj'
ms = pymeshlab.MeshSet()
ms.load_new_mesh(mesh_path)
m = ms.current_mesh()

# pytorch
obj2pt --> convert to pytorch data
tri = mesh.triangles # o3d
edge_index = torch.cat([tri[:2], tri[1:], tri[1::3]], dim=1)

segmentation_labels = segment....
np.savetxt(segmentation_labels..)
# back to meshlab with segmentation
m.add_vertex_custom_scalar_attribute(segmentation_labels, 'label')
ms.compute_selection_by_condition_per_vertex(condselect="(label == 1)")

# this should save only top surface
ms.save_current_mesh(mesh_path[:-4] + "_mlab.obj")


#o3d.io.write_triangle_mesh('/home/luca/RePAIR/dataset/group_19/processed/RPf_00152_removed.obj', mesh)
