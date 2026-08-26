//! Fibre-section export, mirroring Python `post/fibre.to_fibre_section`.
//!
//! A fibre section discretises the cross-section into uniaxial fibres
//! (one per mesh triangle), each carrying an area and a centroid. The result
//! feeds nonlinear beam-column analysis programs.

use crate::geometry::Point;
use crate::mesh::{MeshParams, Mesh, mesh_section};
use crate::section::Section;

/// A single uniaxial fibre.
#[derive(Debug, Clone, Copy)]
pub struct Fiber {
    /// Fibre area.
    pub area: f64,
    /// Fibre centroid (global coordinates).
    pub centroid: Point,
    /// Material group index (for multi-material sections).
    pub material: usize,
}

impl Fiber {
    /// Uniaxial stiffness contribution E·A given the modulus.
    pub fn ea(&self, e: f64) -> f64 {
        self.area * e
    }
}

/// Discretise the section into one fibre per mesh triangle.
///
/// Mirrors Python `to_fibre_section`, which converts the FE mesh into fibres
/// grouped by element.
pub fn to_fibre_section(
    section: &Section,
    params: MeshParams,
) -> Vec<Fiber> {
    let mesh: Mesh = mesh_section(section, params);
    to_fibre_from_mesh(&mesh)
}

/// Build fibres from an existing mesh (one fibre per element).
pub fn to_fibre_from_mesh(mesh: &Mesh) -> Vec<Fiber> {
    let mut fibres = Vec::with_capacity(mesh.n_elements());
    for i in 0..mesh.n_elements() {
        fibres.push(Fiber {
            area: mesh.element_area(i),
            centroid: mesh.element_centroid(i),
            material: mesh.element_materials.get(i).copied().unwrap_or(0),
        });
    }
    fibres
}

/// Total fibre area (should match the section area within mesh error).
pub fn total_area(fibres: &[Fiber]) -> f64 {
    fibres.iter().map(|f| f.area).sum()
}

/// Plastic axial force N_pl = fy * sum(A_fibre) for the whole fibre section.
pub fn plastic_axial(fibres: &[Fiber], fy: f64) -> f64 {
    total_area(fibres) * fy
}

/// Plastic bending moment about the x-axis:
/// M_pl = fy * sum(A_i * |y_i - y_na|) with the neutral axis at the plastic
/// centroid (equal-area split).
pub fn plastic_moment_x(fibres: &[Fiber], fy: f64) -> f64 {
    // Plastic neutral axis: horizontal line splitting total area in half;
    // find by sorting fibre centroids and accumulating area.
    let total: f64 = fibres.iter().map(|f| f.area).sum();
    let half = total / 2.0;

    let mut sorted: Vec<&Fiber> = fibres.iter().collect();
    sorted.sort_by(|a, b| a.centroid.y.partial_cmp(&b.centroid.y).unwrap());

    // Find y-na by walking cumulative area upward.
    let mut cum = 0.0;
    let mut y_na = sorted.first().map(|f| f.centroid.y).unwrap_or(0.0);
    for w in sorted.windows(2) {
        let (lo, hi) = (w[0].centroid.y, w[1].centroid.y);
        if cum + w[0].area >= half {
            y_na = lo;
            break;
        }
        cum += w[0].area;
        y_na = hi;
    }

    let _ = y_na;
    // Exact PNA position via linear interpolation inside the straddling band.
    let mut cum2 = 0.0;
    let mut na = sorted.last().map(|f| f.centroid.y).unwrap_or(0.0);
    for w in sorted.windows(2) {
        let (a_lo, y_lo) = (w[0].area, w[0].centroid.y);
        if cum2 + a_lo >= half {
            let frac = (half - cum2) / a_lo.max(1e-15);
            na = y_lo + frac * (w[1].centroid.y - y_lo);
            break;
        }
        cum2 += a_lo;
        na = w[1].centroid.y;
    }
    let _ = y_na;

    // Sum |contribution| about the found axis.
    let mut m = 0.0f64;
    for f in fibres {
        m += f.area * (f.centroid.y - na).abs();
    }
    m * fy
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Polygon;

    fn rect_section(w: f64, h: f64) -> Section {
        Section::new(
            Polygon::new(vec![
                Point::new(-w / 2.0, -h / 2.0),
                Point::new(w / 2.0, -h / 2.0),
                Point::new(w / 2.0, h / 2.0),
                Point::new(-w / 2.0, h / 2.0),
            ]),
            vec![],
        )
    }

    #[test]
    fn fibre_total_area_matches() {
        let sec = rect_section(0.1, 0.2);
        let fibres = to_fibre_section(&sec, MeshParams { target_size: 0.02, ..Default::default() });
        assert!(rel_err(total_area(&fibres), 0.02) < 1e-3);
    }

    #[test]
    fn fibre_plastic_moment_rectangle() {
        let sec = rect_section(0.1, 0.2);
        let fy = 355.0e6;
        let fibres = to_fibre_section(&sec, MeshParams { target_size: 0.01, ..Default::default() });
        let m_pl = plastic_moment_x(&fibres, fy);
        let expected = 0.1 * 0.04 / 4.0 * fy; // b*h^2/4 * fy
        assert!(
            (m_pl - expected).abs() / expected < 0.02,
            "Mpl={} expected={}",
            m_pl,
            expected
        );
    }

    fn rel_err(a: f64, b: f64) -> f64 {
        ((a - b) / b).abs()
    }
}
