#!/usr/bin/env python3
"""Check Python's internal consistency: does its own solver reproduce stored omega?"""
import numpy as np
from sectionproperties.pre.library import channel_section
from sectionproperties.analysis import Section
from sectionproperties.analysis import solver

geom = channel_section(d=200.0, b=75.0, t_w=8.0, t_f=10.0, r=12.0, n_r=10)
geom.create_mesh(mesh_sizes=[10])
sec = Section(geometry=geom)
sec.calculate_geometric_properties()
sec.calculate_warping_properties()
n = sec.num_nodes

om_py = np.asarray(sec.section_props.omega)
k_lg, f = sec.assemble_torsion()
om_direct = solver.solve_direct_lagrange(k_lg=k_lg, f=f)

print("max|omega_py - omega_solve_direct_lagrange| =",
      np.max(np.abs(om_py - om_direct)))
print("om_py[:5]:   ", om_py[:5])
print("om_direct[:5]:", om_direct[:5])

# Also print some k_lg diagnostics and compare against python_global export
import json
py = json.load(open("python_global_Channel_200x75.json"))
K = np.asarray(py["K"]["data"])
print("exported K nnz:", len(K))
