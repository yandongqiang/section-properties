#!/usr/bin/env python3
"""
Export Python sectionproperties mesh for import into Rust.
"""

import json
import numpy as np
from sectionproperties.pre import Material
from sectionproperties.pre.library import channel_section, i_section, angle_section
from sectionproperties.analysis import Section

def export_python_mesh(section_name, geom):
    """Export mesh to JSON for Rust import."""
    mesh = geom.mesh
    
    data = {
        'section_name': section_name,
        'vertices': mesh['vertices'].tolist(),
        'triangles': mesh['triangles'].tolist(),
        'segments': mesh['segments'],
        'segment_markers': mesh['segment_markers'],
        'regions': mesh['regions']
    }
    
    with open(f'python_mesh_{section_name}.json', 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"Exported {section_name}: {mesh['vertices'].shape[0]} vertices, {mesh['triangles'].shape[0]} triangles")
    return mesh['vertices'].shape[0]

def main():
    sections = []
    
    # Channel 200x75
    channel = channel_section(d=200.0, b=75.0, t_w=8.0, t_f=10.0, r=12.0, n_r=10)
    channel.create_mesh(mesh_sizes=[10])
    sections.append(("Channel_200x75", channel))
    
    # I-section
    from sectionproperties.pre.library import i_section
    i_sec = i_section(d=300.0, b=150.0, t_f=8.0, t_w=12.0, r=15.0, n_r=10)
    i_sec.create_mesh(mesh_sizes=[15])
    sections.append(("I_section_300x150", i_sec))
    
    # Angle
    angle = angle_section(d=100.0, b=100.0, t=8.0, r_r=10.0, r_t=5.0, n_r=8)
    angle.create_mesh(mesh_sizes=[5])
    sections.append(("Angle_100x100", angle))
    
    # Thin Channel
    channel = channel_section(d=300.0, b=100.0, t_w=3.0, t_f=6.0, r=8.0, n_r=8)
    channel.create_mesh(mesh_sizes=[5])
    sections.append(("Thin_Channel_300x100x3", channel))
    
    for name, geom in sections:
        export_python_mesh(name, geom)

if __name__ == '__main__':
    from sectionproperties.pre import Material
    from sectionproperties.pre.library import channel_section, i_section, angle_section
    from sectionproperties.analysis import Section
    import numpy as np
    main()