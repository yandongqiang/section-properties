//! Mesh generation and FEM analysis core.
//!
//! Provides triangular mesh generation (ear clipping, Delaunay) and
//! linear elastic FEM solver for section analysis.

pub mod fem;
pub mod triangulation;

pub use fem::{
    AnalysisResults, ElementType, FemModel, FemSolver, LoadCase, MaterialProps, StressResult,
    analyze_section,
};
pub use triangulation::{Triangle, Triangulation, triangulate_polygon, triangulate_section};

use crate::geometry::{Point, Polygon};
use crate::section::Section;
use crate::section_library::CompositeSection;

/// 2D triangular mesh for a section.
#[derive(Debug, Clone)]
pub struct Mesh {
    /// Nodal coordinates
    pub nodes: Vec<Point>,
    /// Element connectivity (3 node indices per triangle, CCW)
    pub elements: Vec<[usize; 3]>,
    /// Material index per element
    pub element_materials: Vec<usize>,
    /// Boundary nodes (for applying BCs)
    pub boundary_nodes: Vec<usize>,
    /// Hole boundary nodes (for internal boundaries)
    pub hole_boundary_nodes: Vec<Vec<usize>>,
}

impl Mesh {
    /// Create an empty mesh.
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            elements: Vec::new(),
            element_materials: Vec::new(),
            boundary_nodes: Vec::new(),
            hole_boundary_nodes: Vec::new(),
        }
    }

    /// Number of nodes.
    pub fn n_nodes(&self) -> usize {
        self.nodes.len()
    }

    /// Number of elements.
    pub fn n_elements(&self) -> usize {
        self.elements.len()
    }

    /// Get node coordinates.
    pub fn node(&self, index: usize) -> Point {
        self.nodes[index]
    }

    /// Get element connectivity.
    pub fn element(&self, index: usize) -> [usize; 3] {
        self.elements[index]
    }

    /// Get element centroid.
    pub fn element_centroid(&self, index: usize) -> Point {
        let elem = self.elements[index];
        let p1 = self.nodes[elem[0]];
        let p2 = self.nodes[elem[1]];
        let p3 = self.nodes[elem[2]];
        Point::new((p1.x + p2.x + p3.x) / 3.0, (p1.y + p2.y + p3.y) / 3.0)
    }

    /// Get element area.
    pub fn element_area(&self, index: usize) -> f64 {
        let elem = self.elements[index];
        let p1 = self.nodes[elem[0]];
        let p2 = self.nodes[elem[1]];
        let p3 = self.nodes[elem[2]];
        0.5 * ((p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y)).abs()
    }

    /// Compute mesh quality metrics.
    pub fn quality_metrics(&self) -> MeshQuality {
        let mut min_angle = f64::MAX;
        let mut max_angle = f64::MIN;
        let mut min_area = f64::MAX;
        let mut max_area = f64::MIN;
        let mut aspect_ratios = Vec::new();

        for i in 0..self.n_elements() {
            let elem = self.elements[i];
            let p1 = self.nodes[elem[0]];
            let p2 = self.nodes[elem[1]];
            let p3 = self.nodes[elem[2]];

            // Edge lengths
            let a = ((p2.x - p3.x).powi(2) + (p2.y - p3.y).powi(2)).sqrt();
            let b = ((p3.x - p1.x).powi(2) + (p3.y - p1.y).powi(2)).sqrt();
            let c = ((p1.x - p2.x).powi(2) + (p1.y - p2.y).powi(2)).sqrt();

            // Angles using law of cosines
            let angles = [
                ((b * b + c * c - a * a) / (2.0 * b * c)).acos(),
                ((a * a + c * c - b * b) / (2.0 * a * c)).acos(),
                ((a * a + b * b - c * c) / (2.0 * a * b)).acos(),
            ];

            for &angle in &angles {
                min_angle = min_angle.min(angle);
                max_angle = max_angle.max(angle);
            }

            // Area
            let area = 0.5 * ((p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y)).abs();
            min_area = min_area.min(area);
            max_area = max_area.max(area);

            // Aspect ratio (longest edge / shortest altitude)
            let s = (a + b + c) / 2.0;
            let area_heron = (s * (s - a) * (s - b) * (s - c)).max(0.0).sqrt();
            let h_min = 2.0 * area_heron / a.max(b).max(c);
            let aspect = a.max(b).max(c) / h_min;
            aspect_ratios.push(aspect);
        }

        MeshQuality {
            min_angle: min_angle.to_degrees(),
            max_angle: max_angle.to_degrees(),
            min_area,
            max_area,
            avg_aspect_ratio: aspect_ratios.iter().sum::<f64>() / aspect_ratios.len() as f64,
            max_aspect_ratio: aspect_ratios.iter().fold(0.0, |a, &b| a.max(b)),
        }
    }
}

impl Default for Mesh {
    fn default() -> Self {
        Self::new()
    }
}

/// Mesh quality metrics.
#[derive(Debug, Clone, Copy)]
pub struct MeshQuality {
    pub min_angle: f64,
    pub max_angle: f64,
    pub min_area: f64,
    pub max_area: f64,
    pub avg_aspect_ratio: f64,
    pub max_aspect_ratio: f64,
}

/// Mesh generation parameters.
#[derive(Debug, Clone, Copy)]
pub struct MeshParams {
    /// Target element size (approximate edge length)
    pub target_size: f64,
    /// Maximum element size
    pub max_size: f64,
    /// Minimum element size
    pub min_size: f64,
    /// Quality threshold (0-1, higher = better quality)
    pub quality_threshold: f64,
    /// Whether to use Delaunay refinement
    pub use_delaunay: bool,
    /// Maximum refinement iterations
    pub max_iterations: usize,
}

impl Default for MeshParams {
    fn default() -> Self {
        Self {
            target_size: 0.01,
            max_size: 0.02,
            min_size: 0.001,
            quality_threshold: 0.3,
            use_delaunay: true,
            max_iterations: 10,
        }
    }
}

/// Generate mesh from a Section (outer boundary + holes).
pub fn mesh_section(section: &Section, params: MeshParams) -> Mesh {
    triangulate_section(section, params)
}

/// Generate mesh from a CompositeSection (multi-material).
pub fn mesh_composite_section(composite: &CompositeSection, params: MeshParams) -> Mesh {
    // For composite sections, we need to handle multiple materials
    // For now, use the outer boundary and assign materials based on material groups
    let mut mesh = triangulate_section(
        &Section::new(composite.outer.clone(), composite.holes.clone()),
        params,
    );

    // Assign material indices to elements based on centroid location
    // This is a simplified approach - full implementation would use proper material regions
    mesh.element_materials = vec![0; mesh.n_elements()];

    mesh
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon};
    use crate::section::Section;

    #[test]
    fn mesh_quality_metrics() {
        let mut mesh = Mesh::new();
        mesh.nodes = vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(0.0, 1.0),
            Point::new(1.0, 1.0),
        ];
        mesh.elements = vec![[0, 1, 2], [1, 3, 2]];

        let quality = mesh.quality_metrics();
        assert!(quality.min_angle > 0.0);
        assert!(quality.max_angle <= 180.0);
    }
}
