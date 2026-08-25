//! Mesh exporters: Nastran bulk data (BDF) and legacy VTK.
//!
//! Mirrors `sectionproperties.post.nastran` and the pyvista-based VTK export.

use crate::material::Material;
use crate::mesh::Mesh;

/// Export a triangular mesh as a Nastran bulk data file.
///
/// Produces `GRID`, `CTRIA3`, `PSHELL` and `MAT1` cards with 1-based
/// node/element numbering, matching the Python exporter's output format.
///
/// `material` supplies the MAT1 card; pass `None` to use default steel-like
/// values in SI units.
pub fn to_nastran(mesh: &Mesh, material: Option<&Material>, units: &str) -> String {
    let mut out = String::new();

    out.push_str("NASTRAN\n");
    out.push_str(&format!(
        "TITLE = sectionproperties mesh export ({units})\n"
    ));
    out.push_str("BEGIN BULK\n");

    // Nodes
    for (i, n) in mesh.nodes.iter().enumerate() {
        out.push_str(&format!(
            "GRID*   {:>16}{:>16.6}{:>16.6}\n",
            i + 1,
            n.x,
            n.y
        ));
    }

    // Material and property cards
    let (e, nu, rho) = match material {
        Some(m) => (m.youngs_modulus, m.poissons_ratio, m.density),
        None => (2.1e11, 0.3, 7850.0),
    };
    out.push_str(&format!(
        "MAT1*   {:>16}{:.4E}{:.4E}\n",
        "1", e, nu
    ));
    out.push_str(&format!("*       {:.4E}\n", rho));
    out.push_str("PSHELL* 1               1.0E-03\n");

    // Elements
    for (i, tri) in mesh.elements.iter().enumerate() {
        out.push_str(&format!(
            "CTRIA3  {:>16}{:>16}{:>16}{:>16}{:>16}\n",
            i + 1,
            "1",
            tri[0] + 1,
            tri[1] + 1,
            tri[2] + 1
        ));
    }

    out.push_str("ENDDATA\n");
    out
}

/// Export a triangular mesh as a legacy VTK (Visualization Toolkit) file.
///
/// Uses an UNSTRUCTURED_GRID dataset of 2D cells so that ParaView can render
/// the mesh directly, mirroring Python's `pyvista.UnstructuredGrid` export.
pub fn to_vtk(mesh: &Mesh) -> String {
    let mut out = String::new();
    out.push_str("# vtk DataFile Version 4.2\n");
    out.push_str("sectionproperties mesh export\n");
    out.push_str("ASCII\n");
    out.push_str("DATASET UNSTRUCTURED_GRID\n");

    out.push_str(&format!("POINTS {} double\n", mesh.nodes.len()));
    for n in &mesh.nodes {
        out.push_str(&format!("{:.10} {:.10} {:.10}\n", n.x, n.y, 0.0));
    }

    out.push_str(&format!("CELLS {} {}\n", mesh.elements.len(), mesh.elements.len() * 4));
    for tri in &mesh.elements {
        out.push_str(&format!("3 {} {} {}\n", tri[0], tri[1], tri[2]));
    }

    out.push_str(&format!("CELL_TYPES {}\n", mesh.elements.len()));
    for _ in &mesh.elements {
        out.push_str("5\n"); // VTK_TRIANGLE
    }

    out.push_str(&format!("CELL_DATA {}\n", mesh.elements.len()));
    if !mesh.element_materials.is_empty() {
        out.push_str("SCALARS material_id int 1\n");
        out.push_str("LOOKUP_TABLE default\n");
        for m in &mesh.element_materials {
            out.push_str(&format!("{m}\n"));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::{MeshParams, mesh_section};
    use crate::section_library::ParametricSection;
    use crate::section_library::primitive::RectangularSection;

    fn sample_mesh() -> Mesh {
        let sec = RectangularSection::new(0.2, 0.1).build();
        mesh_section(&sec, MeshParams::default())
    }

    #[test]
    fn nastran_export_basic() {
        let mesh = sample_mesh();
        assert!(!mesh.elements.is_empty());
        let bdf = to_nastran(&mesh, None, "SI");
        assert!(bdf.contains("BEGIN BULK"));
        assert!(bdf.contains("ENDDATA"));
        assert_eq!(bdf.matches("CTRIA3").count(), mesh.elements.len());
        assert!(bdf.contains("MAT1*"));
    }

    #[test]
    fn vtk_export_basic() {
        let mesh = sample_mesh();
        let vtk = to_vtk(&mesh);
        assert!(vtk.contains("DATASET UNSTRUCTURED_GRID"));
        assert!(vtk.contains(&format!("POINTS {}", mesh.nodes.len())));
        assert!(vtk.contains(&format!("CELL_TYPES {}", mesh.elements.len())));
    }
}
