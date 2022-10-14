import open3d
from visualization import VisOpen3D
import numpy as np
import matplotlib.pyplot as plt
import vedo as vd
import os, random, pdb

def transform_mesh(m):
    cm = m.centerOfMass()
    m.shift(-cm)
    elli = vd.pcaEllipsoid(m, pvalue=0.5)

    ax1 = vd.versor(elli.axis1)
    ax2 = vd.versor(elli.axis2)
    ax3 = vd.versor(elli.axis3)

    # the transposed matrix is already the inverse
    T = np.array([ax1, ax2, ax3])

    return m.applyTransform(T, reset=True)  # <-- I had to enable reset


def get_R_x(n):
    R_x = np.zeros((3, 3))
    theta = np.arcsin(n[1] / np.linalg.norm(np.asarray([n[1], n[2]])))
    print(f"rotating {theta} on yz plane")
    R_x[0, 0] = 1
    R_x[1, 1] = np.cos(theta)
    R_x[1, 2] = - np.sin(theta)
    R_x[2, 2] = np.cos(theta)
    R_x[2, 1] = np.sin(theta)
    return R_x


def get_R_y(n):
    R_y = np.zeros((3, 3))
    theta = np.arcsin(n[0] / np.linalg.norm(np.asarray([n[0], n[2]])))
    print(f"rotating {theta} on xz plane")
    R_y[1, 1] = 1
    R_y[0, 0] = np.cos(theta)
    R_y[0, 2] = np.sin(theta)
    R_y[2, 2] = np.cos(theta)
    R_y[2, 0] = - 1 * np.sin(theta)
    return R_y


def align_face_up(mesh, n, mean_point):
    """
    We want the piece with the flat surface (from normal n)
    looking "up" in the z-axis and in the origin (0, 0, 0).
    So rotation in z is not needed (we don't have yet a solution)
    We just want on xz and yz plane so set it flat
    """
    # move to origin
    mesh.translate(-mean_point)

    # first on yz plane (around x-axis)
    Rx = get_R_x(n)
    mesh.transform(matrix_3x3_to_4x4(Rx))

    # then on xz plane (around y-axis)
    Ry = get_R_y(n)
    mesh.transform(matrix_3x3_to_4x4(Ry))

    # no return, the mesh is transformed
    #print(Rx, Ry, mean_point)

def matrix_3x3_to_4x4(m):

    T = np.zeros((4, 4))
    T[:3, :3] = m
    T[3, 3] = 1
    return T

def create_pos(x, y, noise=0.25):

    pos = np.zeros((3,1))
    pos[0] = x + random.uniform(-noise, noise)
    pos[1] = y + random.uniform(-noise, noise)
    pos[2] = 0 # we don't move on z axis
    return pos

def shake_it(frag):
    R = open3d.geometry.get_rotation_matrix_from_axis_angle(
        [0,
         0,
         np.random.uniform(0, 360)])
    frag.rotate(R=R, center=np.mean(np.array(frag.points), axis=0))
    frag.translate([np.random.uniform(-0.2, 0.2),
                    np.random.uniform(-0.2, 0.2),
                    np.random.uniform(-0.05, 0.05)])
