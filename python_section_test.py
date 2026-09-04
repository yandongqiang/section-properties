#!/usr/bin/env python3
"""
Python sectionproperties warping analysis for cross-validation with Rust.

Tests various section types and checks for negative J issue.
"""

import numpy as np
from sectionproperties.pre import Material
from sectionproperties.pre.library import channel_section, i_section, angle_section
from sectionproperties.analysis import Section

def test_channel_section():
    """Test channel section - thin wall case where J is negative in Rust"""
    print("=" * 70)
    print("CHANNEL SECTION TEST (UPN200-like)")
    print("=" * 70)
    
    channel = channel_section(
        d=200.0, b=75.0, t_w=8.0, t_f=10.0, r=12.0, n_r=10
    )
    channel.create_mesh(mesh_sizes=[10])
    
    steel = Material(
        name="Steel",
        elastic_modulus=200_000,
        poissons_ratio=0.3,
        density=7850,
        yield_strength=250,
        color="grey"
    )
    
    sec = Section(geometry=channel)
    sec.calculate_geometric_properties()
    sec.calculate_warping_properties()
    
    print(f"J (FEM): {sec.get_j():.6e}")
    print(f"Shear center: ({sec.get_sc()[0]:.6f}, {sec.get_sc()[1]:.6f})")
    print(f"Omega max: {np.max(np.abs(sec.section_props.omega)):.6e}")
    
    return sec

def test_i_section():
    """Test I-section - should have positive J"""
    print("\n" + "=" * 70)
    print("I-SECTION TEST")
    print("=" * 70)
    
    from sectionproperties.pre.library import i_section
    
    i_sec = i_section(d=300.0, b=150.0, t_f=8.0, t_w=12.0, r=15.0, n_r=10)
    i_sec.create_mesh(mesh_sizes=[15])
    
    sec = Section(geometry=i_sec)
    sec.calculate_geometric_properties()
    sec.calculate_warping_properties()
    
    print(f"J (FEM): {sec.get_j():.6e}")
    print(f"Shear center: ({sec.get_sc()[0]:.6f}, {sec.get_sc()[1]:.6f})")
    print(f"Omega max: {np.max(np.abs(sec.section_props.omega)):.6e}")
    
    return sec

def test_angle_section():
    """Test angle section - thin wall"""
    print("\n" + "=" * 70)
    print("ANGLE SECTION TEST (100x100x8)")
    print("=" * 70)
    
    from sectionproperties.pre.library import angle_section
    
    angle = angle_section(d=100.0, b=100.0, t=8.0, r_r=10.0, r_t=5.0, n_r=8)
    angle.create_mesh(mesh_sizes=[5])
    
    sec = Section(geometry=angle)
    sec.calculate_geometric_properties()
    sec.calculate_warping_properties()
    
    print(f"J (FEM): {sec.get_j():.6e}")
    print(f"Shear center: ({sec.get_sc()[0]:.6f}, {sec.get_sc()[1]:.6f})")
    print(f"Omega max: {np.max(np.abs(sec.section_props.omega)):.6e}")
    
    return sec

def test_thin_channel():
    """Very thin channel - most likely to have issues"""
    print("\n" + "=" * 70)
    print("THIN CHANNEL TEST (300x100x3)")
    print("=" * 70)
    
    channel = channel_section(d=300.0, b=100.0, t_w=3.0, t_f=6.0, r=8.0, n_r=8)
    channel.create_mesh(mesh_sizes=[5])
    
    sec = Section(geometry=channel)
    sec.calculate_geometric_properties()
    sec.calculate_warping_properties()
    
    print(f"J (FEM): {sec.get_j():.6e}")
    print(f"Shear center: ({sec.get_sc()[0]:.6f}, {sec.get_sc()[1]:.6f})")
    print(f"Omega max: {np.max(np.abs(sec.section_props.omega)):.6e}")
    
    return sec

def main():
    print("=" * 70)
    print("PYTHON SECTIONPROPERTIES WARPING ANALYSIS")
    print("=" * 70)
    
    sec1 = test_channel_section()
    sec2 = test_i_section()
    sec3 = test_angle_section()
    sec4 = test_thin_channel()
    
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print("Channel:   J_FEM = {:.6e}".format(sec1.get_j()))
    print("I-section: J_FEM = {:.6e}".format(sec2.get_j()))
    print("Angle:     J_FEM = {:.6e}".format(sec3.get_j()))
    print("Thin Chan: J_FEM = {:.6e}".format(sec4.get_j()))
    print("\nIf J_FEM < 0, Python falls back to analytical J (like Rust)")

def test_i_section():
    """Test I-section - should have positive J"""
    print("\n" + "=" * 70)
    print("I-SECTION TEST")
    print("=" * 70)
    
    from sectionproperties.pre.library import i_section
    
    i_sec = i_section(d=300.0, b=150.0, t_f=8.0, t_w=12.0, r=15.0, n_r=10)
    i_sec.create_mesh(mesh_sizes=[15])
    
    sec = Section(geometry=i_sec)
    sec.calculate_geometric_properties()
    sec.calculate_warping_properties()
    
    print(f"J (FEM): {sec.get_j():.6e}")
    print(f"Shear center: ({sec.get_sc()[0]:.6f}, {sec.get_sc()[1]:.6f})")
    print(f"Omega max: {np.max(np.abs(sec.section_props.omega)):.6e}")
    
    return sec

def test_angle_section():
    """Test angle section - thin wall"""
    print("\n" + "=" * 70)
    print("ANGLE SECTION TEST (100x100x8)")
    print("=" * 70)
    
    from sectionproperties.pre.library import angle_section
    
    angle = angle_section(d=100.0, b=100.0, t=8.0, r_r=10.0, r_t=5.0, n_r=8)
    angle.create_mesh(mesh_sizes=[5])
    
    sec = Section(geometry=angle)
    sec.calculate_geometric_properties()
    sec.calculate_warping_properties()
    
    print(f"J (FEM): {sec.get_j():.6e}")
    print(f"Shear center: ({sec.get_sc()[0]:.6f}, {sec.get_sc()[1]:.6f})")
    print(f"Omega max: {np.max(np.abs(sec.section_props.omega)):.6e}")
    
    return sec

def test_thin_channel():
    """Very thin channel - most likely to have issues"""
    print("\n" + "=" * 70)
    print("THIN CHANNEL TEST (300x100x3)")
    print("=" * 70)
    
    channel = channel_section(d=300.0, b=100.0, t_w=3.0, t_f=6.0, r=8.0, n_r=8)
    channel.create_mesh(mesh_sizes=[5])
    
    sec = Section(geometry=channel)
    sec.calculate_geometric_properties()
    sec.calculate_warping_properties()
    
    print(f"J (FEM): {sec.get_j():.6e}")
    print(f"Shear center: ({sec.get_sc()[0]:.6f}, {sec.get_sc()[1]:.6f})")
    print(f"Omega max: {np.max(np.abs(sec.section_props.omega)):.6e}")
    
    return sec

def main():
    print("=" * 70)
    print("PYTHON SECTIONPROPERTIES WARPING ANALYSIS")
    print("=" * 70)
    
    sec1 = test_channel_section()
    sec2 = test_i_section()
    sec3 = test_angle_section()
    sec4 = test_thin_channel()
    
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print("Channel:   J_FEM = {:.6e}".format(sec1.get_j()))
    print("I-section: J_FEM = {:.6e}".format(sec2.get_j()))
    print("Angle:     J_FEM = {:.6e}".format(sec3.get_j()))
    print("Thin Chan: J_FEM = {:.6e}".format(sec4.get_j()))
    print("\nIf J_FEM < 0, Python falls back to analytical J (like Rust)")

if __name__ == '__main__':
    from sectionproperties.pre import Material
    from sectionproperties.pre.library import channel_section, i_section, angle_section
    from sectionproperties.analysis import Section
    import numpy as np
    main()