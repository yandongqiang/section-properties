#!/usr/bin/env python3
"""Verify J from the matching Rust/Python matrices on the same mesh.

Uses Python's sectionproperties to provide the ground-truth ixx/iyy and J,
then solves the Lagrange system with the (already verified identical) K/F/C
and checks the resulting J against Python's.
"""
import json
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu
from sectionproperties.pre import Material
from sectionproperties.pre.library import channel_section, i_section, angle_section
from sectionproperties.analysis import Section


def build_section(name):
    if name == "Channel_200x75":
        geom = channel_section(d=200.0, b=75.0, t_w=8.0, t_f=10.0, r=12.0, n_r=10)
        geom.create_mesh(mesh_sizes=[10])
    elif name == "I_section_300x150":
        geom = i_section(d=300.0, b=150.0, t_f=8.0, t_w=12.0, r=15.0, n_r=10)
        geom.create_mesh(mesh_sizes=[15])
    elif name == "Angle_100x100":
        geom = angle_section(d=100.0, b=100.0, t=8.0, r_r=10.0, r_t=5.0, n_r=8)
        geom.create_mesh(mesh_sizes=[5])
    elif name == "Thin_Channel_300x100x3":
        geom = channel_section(d=300.0, b=100.0, t_w=3.0, t_f=6.0, r=8.0, n_r=8)
        geom.create_mesh(mesh_sizes=[5])
    else:
        raise ValueError(name)
    return geom


def verify(name):
    geom = build_section(name)
    sec = Section(geometry=geom)
    sec.calculate_geometric_properties()
    sec.calculate_warping_properties()

    ixx = sec.section_props.ixx_c
    iyy = sec.section_props.iyy_c
    j_py = sec.get_j()

    py = json.load(open(f"python_global_{name}.json"))
    n = py["n_dof"]
    K = sp.csc_matrix((py["K"]["data"], (py["K"]["row"], py["K"]["col"])), shape=(n, n))
    C = np.array(py["C"])[:, None].astype(float)
    F = np.array(py["F"])
    A = sp.bmat([[K, C], [C.T, [[0]]]], format="csc")
    rhs = np.concatenate([F, [0.0]])
    omega = splu(A).solve(rhs)[:n]

    wtf = omega @ F
    j_computed = (ixx + iyy) - wtf

    # Python's own omega for reference
    om_py = np.asarray(sec.section_props.omega)
    omega_diff = np.max(np.abs(omega - om_py))

    print(f"{name}: n={n} ixx+iyy={ixx+iyy:.6e}")
    print(f"  J_python         = {j_py:.6e}")
    print(f"  J_computed(Rust K,F,C + Python solver) = {j_computed:.6e}")
    print(f"  omega_dot_f      = {wtf:.6e}")
    print(f"  max|omega - omega_py| = {omega_diff:.3e}")
    print()


if __name__ == "__main__":
    for s in ["Channel_200x75", "I_section_300x150", "Angle_100x100",
              "Thin_Channel_300x100x3"]:
        verify(s)
