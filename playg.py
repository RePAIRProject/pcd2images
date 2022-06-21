import open3d as o3d
import numpy as np
import pdb
import pymeshlab
import torch
from torch_geometric.data import Data

def main():
    mesh_path = '/home/luca/RePAIR/dataset/group_19/processed/RPf_00152.obj'
    ms = pymeshlab.MeshSet()
    rp_dataset_folder = '/home/luca/RePAIR/dataset/'
    #rp_dataset_folder = '/home/luca/Unive/RePAIR/Datasets/RePAIR_dataset'
    group_folder = os.path.join(rp_dataset_folder, f'group_{group}', 'top_surfaces')
    output_folder = os.path.join(rp_dataset_folder, f'group_{group}', 'rendered_o3d')
    if not os.path.exists(group_folder):
        print('Nothing found! Please prepare top surfaces using segment_surfaces_for_rendering.py!')

    else:
        if not os.path.exists(output_folder):
            os.mkdir(output_folder)
        files = os.listdir(group_folder)
        frags = [file for file in files if file[-4:] == '.obj']
        print(f'in group {group} found {len(frags)} fragments (.obj)')

        #create window
        window_visible = False

        for frag in frags:
    ms.load_new_mesh(mesh_path)
    m = ms.current_mesh()

    # pytorch
    v = m.vertex_matrix()
    f = m.face_matrix()
    e = m.edge_matrix()
    f_n = m.face_normal_matrix()
    f_n = m.vertex_normal_matrix()
    pdb.set_trace()
    edges = torch.cat([torch.tensor(f[:,:2]), torch.tensor(f[:,1:]), torch.tensor(f[:,0:3:2])], dim=0)
    x = torch.tensor(np.asarray(v), dtype=torch.float32)
    data = Data(vertices = x,
                vertices_normals = torch.tensor(v_n),
                edge_index=edges,
                faces = torch.tensor(f),
                faces_normals = torch.tensor(f_n)
                )
    pdb.set_trace()

    #segmentation_labels = segment....
    np.savetxt(segmentation_labels)
    # back to meshlab with segmentation
    m.add_vertex_custom_scalar_attribute(segmentation_labels, 'label')
    ms.compute_selection_by_condition_per_vertex(condselect="(label == 1)")

    # this should save only top surface
    ms.save_current_mesh(mesh_path[:-4] + "_mlab.obj")

if __name__ == "__main__":
    main()



#o3d.io.write_triangle_mesh('/home/luca/RePAIR/dataset/group_19/processed/RPf_00152_removed.obj', mesh)
