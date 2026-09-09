#!/usr/bin/env python3
"""Verify negative J is caused by coordinate origin (centroid-shift of F_torsion)."""
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import splu
from sectionproperties.pre.library import channel_section
from sectionproperties.analysis import Section

geom = channel_section(d=200.0, b=75.0, t_w=8.0, t_f=10.0, r=12.0, n_r=10)
geom.create_mesh(mesh_sizes=[10])
sec = Section(geometry=geom)
sec.calculate_geometric_properties()
sec.calculate_warping_properties()
n = sec.num_nodes

cx, cy = sec.get_c()
ixx = sec.section_props.ixx_c
iyy = sec.section_props.iyy_c
print("centroid (cx,cy)=", cx, cy)

# Rebuild k_lg, f with centroid-shifted coordinates (mirroring Python's warping)
ws = Section(geometry=geom)
for el in ws.elements:
    el.coords[0, :] -= cx
    el.coords[1, :] -= cy
k_lg, f_shifted = ws.assemble_torsion()

om_shift = splu(k_lg.tocsc()).solve(np.append(f_shifted, 0.0))[:n]
j_shift = (ixx + iyy) - om_shift @ f_shifted
print("J with centroid-shifted F =", f"{j_shift:.6e}")
print("J_python                  =", f"{sec.get_j():.6e}")
print("max|om_shift - om_py|     =", np.max(np.abs(om_shift - sec.section_props.omega)))

# Compare with UN-shifted assembly (what Rust currently does)
k_lg2, f_un = sec.assemble_torsion()
om_un = splu(k_lg2.tocsc()).solve(np.append(f_un, 0.0))[:n]
j_un = (ixx + iyy) - om_un @ f_un
print("J with UN-shifted F       =", f"{j_un:.6e}")
