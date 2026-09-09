#!/usr/bin/env python3
"""Check whether Python's own assemble_torsion + solve reproduces its stored omega."""
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu
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


for name in ["Channel_200x75", "I_section_300x150", "Angle_100x100", "Thin_Channel_300x100x3"]:
    geom = build_section(name)
    sec = Section(geometry=geom)
    sec.calculate_geometric_properties()
    sec.calculate_warping_properties()
    n = sec.num_nodes

    # Python's stored omega after full warping analysis
    om_py = np.asarray(sec.section_props.omega)

    # Rebuild k_lg, f from standalone assemble_torsion
    k_lg, f = sec.assemble_torsion()
    # NOTE: f has length n; append 0 for multiplier
    sol = splu(k_lg.tocsc()).solve(np.append(f, 0.0))
    om_re = sol[:n]

    print(f"{name}: max|omega_py - omega_reassemble| = "
          f"{np.max(np.abs(om_py - om_re)):.4e}")

    ixx = sec.section_props.ixx_c
    iyy = sec.section_props.iyy_c
    print(f"   J_py = {sec.get_j():.6e}")
    print(f"   J(om_py)      = {ixx+iyy - om_py@f:.6e}")
    print(f"   J(om_reassem) = {ixx+iyy - om_re@f:.6e}")
    print(f"   E of elem0 = {sec.elements[0].material.elastic_modulus}")
