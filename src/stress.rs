//! Cross-section stress analysis.
//!
//! Mirrors Python `sectionproperties.post.stress_post.StressPost`:
//! computes stress distributions from design actions (N, Vx, Vy, Mxx, Myy, M11, M22, Mzz)
//! using analytical formulas (no FEM mesh required).

use crate::geometry::Point;
use crate::material::Material;
use crate::section::Section;
use crate::section_properties::SectionProperties;

/// Design actions applied to a cross-section.
#[derive(Debug, Clone, Copy)]
pub struct SectionLoads {
    /// Axial force [N] (positive = tension)
    pub n: f64,
    /// Shear force in x-direction [N]
    pub vx: f64,
    /// Shear force in y-direction [N]
    pub vy: f64,
    /// Bending moment about centroidal x-axis [N·m]
    pub mxx: f64,
    /// Bending moment about centroidal y-axis [N·m]
    pub myy: f64,
    /// Bending moment about principal 11-axis [N·m]
    pub m11: f64,
    /// Bending moment about principal 22-axis [N·m]
    pub m22: f64,
    /// Torsion moment about z-axis [N·m]
    pub mzz: f64,
}

impl SectionLoads {
    /// Create a zero load case.
    pub fn zero() -> Self {
        Self {
            n: 0.0,
            vx: 0.0,
            vy: 0.0,
            mxx: 0.0,
            myy: 0.0,
            m11: 0.0,
            m22: 0.0,
            mzz: 0.0,
        }
    }

    /// Pure axial load.
    pub fn axial(n: f64) -> Self {
        Self { n, ..Self::zero() }
    }

    /// Pure bending about x-axis.
    pub fn bending_x(mxx: f64) -> Self {
        Self {
            mxx,
            ..Self::zero()
        }
    }

    /// Pure bending about y-axis.
    pub fn bending_y(myy: f64) -> Self {
        Self {
            myy,
            ..Self::zero()
        }
    }

    /// Pure shear in y-direction.
    pub fn shear_y(vy: f64) -> Self {
        Self { vy, ..Self::zero() }
    }

    /// Pure torsion.
    pub fn torsion(mzz: f64) -> Self {
        Self {
            mzz,
            ..Self::zero()
        }
    }
}

impl Default for SectionLoads {
    fn default() -> Self {
        Self::zero()
    }
}

/// Stress state at a point in the cross-section, with breakdown by load component.
#[derive(Debug, Clone, Copy)]
pub struct StressAtPoint {
    /// x-coordinate [m]
    pub x: f64,
    /// y-coordinate [m]
    pub y: f64,
    /// Normal stress σ_z from axial force N [Pa]
    pub sig_zz_n: f64,
    /// Normal stress σ_z from Mxx [Pa]
    pub sig_zz_mxx: f64,
    /// Normal stress σ_z from Myy [Pa]
    pub sig_zz_myy: f64,
    /// Normal stress σ_z from M11 [Pa]
    pub sig_zz_m11: f64,
    /// Normal stress σ_z from M22 [Pa]
    pub sig_zz_m22: f64,
    /// Shear stress τ_xz from Vx [Pa]
    pub sig_zx_vx: f64,
    /// Shear stress τ_yz from Vx [Pa]
    pub sig_zy_vx: f64,
    /// Shear stress τ_xz from Vy [Pa]
    pub sig_zx_vy: f64,
    /// Shear stress τ_yz from Vy [Pa]
    pub sig_zy_vy: f64,
    /// Shear stress τ_xz from Mzz (torsion) [Pa]
    pub sig_zx_mzz: f64,
    /// Shear stress τ_yz from Mzz (torsion) [Pa]
    pub sig_zy_mzz: f64,
    /// Combined normal stress from all bending moments [Pa]
    pub sig_zz_m: f64,
    /// Resultant shear stress from torsion Mzz [Pa]
    pub sig_zxy_mzz: f64,
    /// Resultant shear stress from Vx [Pa]
    pub sig_zxy_vx: f64,
    /// Resultant shear stress from Vy [Pa]
    pub sig_zxy_vy: f64,
    /// Combined shear stress τ_xz from all shear forces [Pa]
    pub sig_zx_v: f64,
    /// Combined shear stress τ_yz from all shear forces [Pa]
    pub sig_zy_v: f64,
    /// Resultant shear stress from all shear forces [Pa]
    pub sig_zxy_v: f64,
    /// Combined normal stress σ_z [Pa]
    pub sigma_z: f64,
    /// Combined shear stress τ_xz [Pa]
    pub tau_xz: f64,
    /// Combined shear stress τ_yz [Pa]
    pub tau_yz: f64,
    /// Resultant shear stress τ_zxy [Pa]
    pub tau_zxy: f64,
    /// von Mises stress [Pa]
    pub von_mises: f64,
    /// Major principal stress [Pa]
    pub sigma_1: f64,
    /// Minor principal stress [Pa]
    pub sigma_2: f64,
}

/// Overall stress analysis results for a cross-section.
#[derive(Debug, Clone)]
pub struct StressAnalysisResult {
    /// Stress at each evaluated point (section boundary vertices).
    pub point_stresses: Vec<StressAtPoint>,
    /// Maximum normal stress σ_z [Pa]
    pub max_sigma_z: f64,
    /// Minimum normal stress σ_z [Pa]
    pub min_sigma_z: f64,
    /// Maximum von Mises stress [Pa]
    pub max_von_mises: f64,
    /// Maximum shear stress [Pa]
    pub max_tau: f64,
    /// Location of max von Mises (x, y)
    pub max_vm_location: Point,
    /// Applied loads
    pub loads: SectionLoads,
    /// Section area [m²]
    pub area: f64,
}

/// Analytical stress analysis for a cross-section.
///
/// Computes stress distributions from design actions using closed-form expressions:
/// - Normal stress: σ_z = N/A + Mxx·y/Ixx − Myy·x/Iyy + M11·y22/I11 − M22·x11/I22
/// - Shear stress: τ ≈ V/A_shear (simplified uniform distribution)
/// - Torsional stress: τ_t ≈ Mzz·t/J (thin-walled approximation)
pub struct StressAnalysis {
    section: Section,
    material: Material,
    props: SectionProperties,
}

impl StressAnalysis {
    /// Create a new stress analysis for a section with given material.
    pub fn new(section: Section, material: Material) -> Self {
        let props = SectionProperties::from_section(&section);
        Self {
            section,
            material,
            props,
        }
    }

    /// Calculate stress distribution for the given load case.
    ///
    /// Uses FEM Tri6 element stress analysis (mirroring Python `stress_post.py`).
    /// Falls back to analytical formulas if FEM mesh generation fails.
    pub fn calculate_stress(&self, loads: SectionLoads) -> StressAnalysisResult {
let area = self.props.area;

        // Try FEM Tri6 stress analysis first
        let fem_points = match crate::stress_fem::calculate_stress_fem(&self.section, &self.props, loads) {
            Ok(points) => Some(points),
            Err(_) => None,
        };

        if let Some(fem_points) = fem_points {
            let mut max_sigma_z = f64::NEG_INFINITY;
            let mut min_sigma_z = f64::INFINITY;
            let mut max_von_mises = 0.0;
            let mut max_tau = 0.0;
            let mut max_vm_loc = Point::new(0.0, 0.0);

            for p in &fem_points {
                if p.sigma_z > max_sigma_z {
                    max_sigma_z = p.sigma_z;
                }
                if p.sigma_z < min_sigma_z {
                    min_sigma_z = p.sigma_z;
                }
                if p.von_mises > max_von_mises {
                    max_von_mises = p.von_mises;
                    max_vm_loc = Point::new(p.x, p.y);
                }
                if p.tau_zxy > max_tau {
                    max_tau = p.tau_zxy;
                }
            }

            return StressAnalysisResult {
                point_stresses: fem_points,
                max_sigma_z,
                min_sigma_z,
                max_von_mises,
                max_tau,
                max_vm_location: max_vm_loc,
                loads,
                area,
            };
        }

        // Analytical fallback
        let cx = self.props.centroid.x;
        let cy = self.props.centroid.y;
        let ix = self.props.ix;
        let iy = self.props.iy;
        let ixy = self.props.ixy;

        let principal = self.props.principal_properties();
        let phi = principal.phi;
        let i11 = principal.i11;
        let i22 = principal.i22;

        let warping = crate::plastic::warping::WarpingProperties::from_section(&self.section, self.material.poissons_ratio);
        let ay = warping.ay.max(1e-12);
        let az = warping.az.max(1e-12);
        let j = warping.j.max(1e-12);

        let cos_phi = phi.cos();
        let sin_phi = phi.sin();

        let denom = ix * iy - ixy * ixy;

        let mut point_stresses: Vec<StressAtPoint> = Vec::new();
        let mut max_sigma_z = f64::NEG_INFINITY;
        let mut min_sigma_z = f64::INFINITY;
        let mut max_von_mises = 0.0;
        let mut max_tau = 0.0;
        let mut max_vm_loc = Point::new(0.0, 0.0);

        let vertices: Vec<Point> = self
            .section
            .outer
            .vertices
            .iter()
            .chain(self.section.holes.iter().flat_map(|h| h.vertices.iter()))
            .cloned()
            .collect();

        for v in &vertices {
            let dx = v.x - cx;
            let dy = v.y - cy;

            let x11 = dx * cos_phi + dy * sin_phi;
            let y22 = -dx * sin_phi + dy * cos_phi;

            // Stress breakdown by load component (mirrors Python StressResult)
            // σ_z = Mxx*(Iyy*y - Ixy*x)/denom + Myy*(Ixy*y - Ixx*x)/denom
            let sig_zz_n = loads.n / area;

            let (sig_zz_mxx, sig_zz_myy) = if denom.abs() > 1e-15 {
                (
                    loads.mxx * (iy * dy - ixy * dx) / denom,
                    loads.myy * (ixy * dy - ix * dx) / denom,
                )
            } else {
                (0.0, 0.0)
            };

            let sig_zz_m11 = if i11 > 1e-15 {
                loads.m11 * y22 / i11
            } else {
                0.0
            };
            let sig_zz_m22 = if i22 > 1e-15 {
                -loads.m22 * x11 / i22
            } else {
                0.0
            };

            // Shear stress breakdown
            let sig_zx_vx = loads.vx / ay;
            let sig_zy_vx = 0.0; // Vx y-component (simplified)
            let sig_zx_vy = 0.0; // Vy x-component (simplified)
            let sig_zy_vy = loads.vy / az;

            let t_max = self.estimate_max_thickness();
            let sig_zx_mzz = 0.0; // Torsion x-component (simplified)
            let sig_zy_mzz = loads.mzz * t_max / j;

            // Combined stresses (mirrors Python StressResult.calculate_combined_stresses)
            let sig_zz_m = sig_zz_mxx + sig_zz_myy + sig_zz_m11 + sig_zz_m22;
            let sig_zxy_mzz = (sig_zx_mzz * sig_zx_mzz + sig_zy_mzz * sig_zy_mzz).sqrt();
            let sig_zxy_vx = (sig_zx_vx * sig_zx_vx + sig_zy_vx * sig_zy_vx).sqrt();
            let sig_zxy_vy = (sig_zx_vy * sig_zx_vy + sig_zy_vy * sig_zy_vy).sqrt();
            let sig_zx_v = sig_zx_vx + sig_zx_vy;
            let sig_zy_v = sig_zy_vx + sig_zy_vy;
            let sig_zxy_v = (sig_zx_v * sig_zx_v + sig_zy_v * sig_zy_v).sqrt();

            let sigma_z = sig_zz_n + sig_zz_m;
            let tau_xz = sig_zx_mzz + sig_zx_v;
            let tau_yz = sig_zy_mzz + sig_zy_v;
            let tau_zxy = (tau_xz * tau_xz + tau_yz * tau_yz).sqrt();

            let von_mises = (sigma_z * sigma_z + 3.0 * tau_zxy * tau_zxy).sqrt();

            let half = sigma_z / 2.0;
            let disc = (half * half + tau_zxy * tau_zxy).sqrt();
            let sigma_1 = half + disc;
            let sigma_2 = half - disc;

            let sap = StressAtPoint {
                x: v.x,
                y: v.y,
                sig_zz_n,
                sig_zz_mxx,
                sig_zz_myy,
                sig_zz_m11,
                sig_zz_m22,
                sig_zx_vx,
                sig_zy_vx,
                sig_zx_vy,
                sig_zy_vy,
                sig_zx_mzz,
                sig_zy_mzz,
                sig_zz_m,
                sig_zxy_mzz,
                sig_zxy_vx,
                sig_zxy_vy,
                sig_zx_v,
                sig_zy_v,
                sig_zxy_v,
                sigma_z,
                tau_xz,
                tau_yz,
                tau_zxy,
                von_mises,
                sigma_1,
                sigma_2,
            };

            if sigma_z > max_sigma_z {
                max_sigma_z = sigma_z;
            }
            if sigma_z < min_sigma_z {
                min_sigma_z = sigma_z;
            }
            if von_mises > max_von_mises {
                max_von_mises = von_mises;
                max_vm_loc = *v;
            }
            if tau_zxy > max_tau {
                max_tau = tau_zxy;
            }

            point_stresses.push(sap);
        }

        if point_stresses.is_empty() {
            max_sigma_z = 0.0;
            min_sigma_z = 0.0;
        }

        StressAnalysisResult {
            point_stresses,
            max_sigma_z,
            min_sigma_z,
            max_von_mises,
            max_tau,
            max_vm_location: max_vm_loc,
            loads,
            area,
        }
    }

    /// Estimate maximum wall thickness for torsional stress calculation.
    fn estimate_max_thickness(&self) -> f64 {
        let bounds = self.section.bounds();
        let h = bounds.3 - bounds.2;
        let b = bounds.1 - bounds.0;
        let bbox_peri = 2.0 * (h + b);
        let actual_peri = self.section.perimeter();
        if actual_peri > 1e-12 {
            bbox_peri * (1.0 - actual_peri / (2.0 * (h + b))) / 4.0 + (h * b / (h + b))
        } else {
            h.min(b) / 2.0
        }
    }

    /// Get the section properties used by this analysis.
    pub fn properties(&self) -> &SectionProperties {
        &self.props
    }

    /// Get the material.
    pub fn material(&self) -> &Material {
        &self.material
    }

    /// Check if the section yields under the given loads.
    pub fn check_yield(&self, loads: SectionLoads) -> YieldCheckResult {
        let result = self.calculate_stress(loads);
        let fy = self.material.yield_strength;

        YieldCheckResult {
            max_stress: result.max_von_mises,
            yield_strength: fy,
            utilization: result.max_von_mises / fy,
            yielded: result.max_von_mises > fy,
        }
    }
}

/// Result of a yield check.
#[derive(Debug, Clone, Copy)]
pub struct YieldCheckResult {
    /// Maximum stress (von Mises) [Pa]
    pub max_stress: f64,
    /// Yield strength [Pa]
    pub yield_strength: f64,
    /// Utilization ratio (max_stress / yield_strength)
    pub utilization: f64,
    /// Whether the section has yielded
    pub yielded: bool,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon};
    use crate::material::presets::STEEL_S355;
    use crate::section_library::ParametricSection;

    #[test]
    fn stress_pure_axial() {
        let poly = Polygon::new(vec![
            Point::new(-0.05, -0.05),
            Point::new(0.05, -0.05),
            Point::new(0.05, 0.05),
            Point::new(-0.05, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let analysis = StressAnalysis::new(section, STEEL_S355);

        let n = 100e3;
        let result = analysis.calculate_stress(SectionLoads::axial(n));

        let area = 0.1 * 0.1;
        let expected_sigma = n / area;

        assert!((result.max_sigma_z - expected_sigma).abs() / expected_sigma < 0.01);
        assert!((result.min_sigma_z - expected_sigma).abs() / expected_sigma < 0.01);
    }

    #[test]
    fn stress_pure_bending() {
        let poly = Polygon::new(vec![
            Point::new(-0.05, -0.05),
            Point::new(0.05, -0.05),
            Point::new(0.05, 0.05),
            Point::new(-0.05, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let analysis = StressAnalysis::new(section, STEEL_S355);

        let mxx = 10e3;
        let result = analysis.calculate_stress(SectionLoads::bending_x(mxx));

        let ix = 0.1 * 0.1_f64.powi(3) / 12.0;
        let y_max = 0.05;
        let expected_sigma = mxx * y_max / ix;

        assert!((result.max_sigma_z - expected_sigma).abs() / expected_sigma < 0.01);
        assert!((result.min_sigma_z + expected_sigma).abs() / expected_sigma < 0.01);
    }

    #[test]
    fn stress_yield_check() {
        let poly = Polygon::new(vec![
            Point::new(-0.05, -0.05),
            Point::new(0.05, -0.05),
            Point::new(0.05, 0.05),
            Point::new(-0.05, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let analysis = StressAnalysis::new(section, STEEL_S355);

        let area = 0.1 * 0.1;
        let n_yield = STEEL_S355.yield_strength * area * 0.5;
        let check = analysis.check_yield(SectionLoads::axial(n_yield));

        assert!(!check.yielded);
        assert!(check.utilization < 1.0);
        assert!((check.utilization - 0.5).abs() < 0.05);
    }

    #[test]
    fn stress_i_section_bending() {
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();
        let analysis = StressAnalysis::new(section, STEEL_S355);

        let mxx = 50e3;
        let result = analysis.calculate_stress(SectionLoads::bending_x(mxx));

        assert!(result.max_sigma_z > 0.0);
        assert!(result.min_sigma_z < 0.0);
        assert!(result.max_von_mises > 0.0);
    }

    #[test]
    fn stress_combined_loads() {
        let poly = Polygon::new(vec![
            Point::new(-0.05, -0.05),
            Point::new(0.05, -0.05),
            Point::new(0.05, 0.05),
            Point::new(-0.05, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let analysis = StressAnalysis::new(section, STEEL_S355);

        let loads = SectionLoads {
            n: 50e3,
            vx: 5e3,
            vy: 10e3,
            mxx: 5e3,
            myy: 3e3,
            m11: 0.0,
            m22: 0.0,
            mzz: 1e3,
        };
        let result = analysis.calculate_stress(loads);

        assert!(result.max_von_mises > 0.0);
        assert!(result.max_tau > 0.0);
        assert!(result.point_stresses.len() >= 4);
    }

    #[test]
    fn stress_principal_bending() {
        let angle = crate::section_library::steel::AngleSection::new(0.1, 0.075, 0.008, 0.0, 0.0);
        let section = angle.build();
        let analysis = StressAnalysis::new(section, STEEL_S355);

        let m11 = 10e3;
        let loads = SectionLoads {
            m11,
            ..SectionLoads::zero()
        };
        let result = analysis.calculate_stress(loads);

        assert!(result.max_sigma_z > 0.0);
        assert!(result.min_sigma_z < 0.0);
    }

    #[test]
    fn stress_breakdown_axial() {
        // Pure axial: sig_zz_n should equal N/A, all other components zero
        let poly = Polygon::new(vec![
            Point::new(-0.05, -0.05),
            Point::new(0.05, -0.05),
            Point::new(0.05, 0.05),
            Point::new(-0.05, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let analysis = StressAnalysis::new(section, STEEL_S355);

        let n = 100e3;
        let result = analysis.calculate_stress(SectionLoads::axial(n));

        let area = 0.1 * 0.1;
        let expected = n / area;

        for s in &result.point_stresses {
            assert!((s.sig_zz_n - expected).abs() / expected < 1e-6);
            assert!(s.sig_zz_mxx.abs() < 1e-6);
            assert!(s.sig_zz_myy.abs() < 1e-6);
            assert!(s.sig_zz_m11.abs() < 1e-6);
            assert!(s.sig_zz_m22.abs() < 1e-6);
        }
    }

    #[test]
    fn stress_breakdown_bending() {
        // Pure Mxx: sig_zz_mxx should be non-zero, sig_zz_n zero
        let poly = Polygon::new(vec![
            Point::new(-0.05, -0.05),
            Point::new(0.05, -0.05),
            Point::new(0.05, 0.05),
            Point::new(-0.05, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let analysis = StressAnalysis::new(section, STEEL_S355);

        let mxx = 10e3;
        let result = analysis.calculate_stress(SectionLoads::bending_x(mxx));

        // FEM evaluates at mesh nodes, some near neutral axis where σ ≈ 0.
        // Check that the maximum bending stress is non-zero.
        let max_mxx = result
            .point_stresses
            .iter()
            .map(|s| s.sig_zz_mxx.abs())
            .fold(0.0_f64, f64::max);
        assert!(max_mxx > 1e-3);
        for s in &result.point_stresses {
            assert!(s.sig_zz_n.abs() < 1e-6);
        }
    }

    #[test]
    fn stress_breakdown_shear() {
        // Pure shear: sig_zx_vx and sig_zy_vy should be non-zero
        let poly = Polygon::new(vec![
            Point::new(-0.05, -0.05),
            Point::new(0.05, -0.05),
            Point::new(0.05, 0.05),
            Point::new(-0.05, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let analysis = StressAnalysis::new(section, STEEL_S355);

        let loads = SectionLoads {
            vx: 5e3,
            vy: 10e3,
            ..SectionLoads::zero()
        };
        let result = analysis.calculate_stress(loads);

        // Shear components must be non-zero somewhere (individual sample
        // points on symmetry lines can legitimately be zero).
        let max_zx_vx = result
            .point_stresses
            .iter()
            .map(|s| s.sig_zx_vx.abs())
            .fold(0.0_f64, f64::max);
        let max_zy_vy = result
            .point_stresses
            .iter()
            .map(|s| s.sig_zy_vy.abs())
            .fold(0.0_f64, f64::max);
        assert!(max_zx_vx > 0.0);
        assert!(max_zy_vy > 0.0);
        for s in &result.point_stresses {
            assert!(s.sig_zz_n.abs() < 1e-6);
        }
    }

    #[test]
    fn stress_breakdown_combination() {
        // Combined loads: sigma_z should equal sum of components
        let poly = Polygon::new(vec![
            Point::new(-0.05, -0.05),
            Point::new(0.05, -0.05),
            Point::new(0.05, 0.05),
            Point::new(-0.05, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let analysis = StressAnalysis::new(section, STEEL_S355);

        let loads = SectionLoads {
            n: 50e3,
            mxx: 5e3,
            myy: 3e3,
            ..SectionLoads::zero()
        };
        let result = analysis.calculate_stress(loads);

        for s in &result.point_stresses {
            let sum = s.sig_zz_n + s.sig_zz_mxx + s.sig_zz_myy + s.sig_zz_m11 + s.sig_zz_m22;
            assert!((sum - s.sigma_z).abs() < 1e-3);
        }
    }

    #[test]
    fn stress_combined_components_pure_axial() {
        let poly = Polygon::new(vec![
            Point::new(-0.05, -0.05),
            Point::new(0.05, -0.05),
            Point::new(0.05, 0.05),
            Point::new(-0.05, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let analysis = StressAnalysis::new(section, STEEL_S355);
        let result = analysis.calculate_stress(SectionLoads::axial(100e3));

        for s in &result.point_stresses {
            // Pure axial: no bending or shear combined components
            assert!(s.sig_zz_m.abs() < 1e-6);
            assert!(s.sig_zxy_mzz.abs() < 1e-6);
            assert!(s.sig_zxy_vx.abs() < 1e-6);
            assert!(s.sig_zxy_vy.abs() < 1e-6);
            assert!(s.sig_zx_v.abs() < 1e-6);
            assert!(s.sig_zy_v.abs() < 1e-6);
            assert!(s.sig_zxy_v.abs() < 1e-6);
        }
    }

    #[test]
    fn stress_combined_components_pure_bending() {
        let poly = Polygon::new(vec![
            Point::new(-0.05, -0.05),
            Point::new(0.05, -0.05),
            Point::new(0.05, 0.05),
            Point::new(-0.05, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let analysis = StressAnalysis::new(section, STEEL_S355);
        let result = analysis.calculate_stress(SectionLoads::bending_x(10e3));

        for s in &result.point_stresses {
            // sig_zz_m should equal sig_zz_mxx (only Mxx applied)
            assert!((s.sig_zz_m - s.sig_zz_mxx).abs() < 1e-3);
            // sigma_z should equal sig_zz_n + sig_zz_m = 0 + sig_zz_mxx
            assert!((s.sigma_z - s.sig_zz_m).abs() < 1e-3);
        }
    }

    #[test]
    fn stress_combined_components_pure_shear() {
        let poly = Polygon::new(vec![
            Point::new(-0.05, -0.05),
            Point::new(0.05, -0.05),
            Point::new(0.05, 0.05),
            Point::new(-0.05, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let analysis = StressAnalysis::new(section, STEEL_S355);
        let loads = SectionLoads {
            vx: 10e3,
            ..SectionLoads::zero()
        };
        let result = analysis.calculate_stress(loads);

        for s in &result.point_stresses {
            // FEM shear from Vx has both x and y components.
            // With Vy=0: sig_zx_v ≈ sig_zx_vx, sig_zy_v ≈ sig_zy_vx
            assert!(
                (s.sig_zx_v - s.sig_zx_vx).abs() < 1e-6,
                "sig_zx_v mismatch at ({}, {})",
                s.x,
                s.y
            );
            assert!(
                (s.sig_zy_v - s.sig_zy_vx).abs() < 1e-6,
                "sig_zy_v mismatch at ({}, {})",
                s.x,
                s.y
            );
            // No torsion: tau_xz ≈ sig_zx_v
            assert!(
                (s.tau_xz - s.sig_zx_v).abs() < 1e-6,
                "tau_xz mismatch at ({}, {})",
                s.x,
                s.y
            );
        }
    }

    #[test]
    fn stress_combined_components_pure_torsion() {
        let poly = Polygon::new(vec![
            Point::new(-0.05, -0.05),
            Point::new(0.05, -0.05),
            Point::new(0.05, 0.05),
            Point::new(-0.05, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let analysis = StressAnalysis::new(section, STEEL_S355);
        let loads = SectionLoads {
            mzz: 5e3,
            ..SectionLoads::zero()
        };
        let result = analysis.calculate_stress(loads);

        for s in &result.point_stresses {
            // FEM torsion has both x and y components.
            // No shear: tau_xz ≈ sig_zx_mzz, tau_yz ≈ sig_zy_mzz
            assert!(
                (s.tau_xz - s.sig_zx_mzz).abs() < 1e-3,
                "tau_xz mismatch at ({}, {})",
                s.x,
                s.y
            );
            assert!(
                (s.tau_yz - s.sig_zy_mzz).abs() < 1e-3,
                "tau_yz mismatch at ({}, {})",
                s.x,
                s.y
            );
        }
    }

    #[test]
    fn stress_unsymmetric_section_bending_formula() {
        // Angle section (ixy != 0): verify bending stress formula
        // σ_z = Mxx*(Iyy*y - Ixy*x)/denom + Myy*(Ixy*y - Ixx*x)/denom
        let angle = crate::section_library::steel::AngleSection::new(0.1, 0.075, 0.008, 0.0, 0.0);
        let section = angle.build();
        let props = crate::section_properties::SectionProperties::from_section(&section);
        let analysis = StressAnalysis::new(section, STEEL_S355);

        let mxx = 5e3;
        let result = analysis.calculate_stress(SectionLoads::bending_x(mxx));

        let ix = props.ix;
        let iy = props.iy;
        let ixy = props.ixy;
        let denom = ix * iy - ixy * ixy;
        let cx = props.centroid.x;
        let cy = props.centroid.y;

        // Verify at each point that sig_zz_mxx matches the formula
        for s in &result.point_stresses {
            let dx = s.x - cx;
            let dy = s.y - cy;
            let expected = mxx * (iy * dy - ixy * dx) / denom;
            assert!(
                (s.sig_zz_mxx - expected).abs() / expected.abs().max(1e-3) < 0.01,
                "sig_zz_mxx = {}, expected {} at ({}, {})",
                s.sig_zz_mxx,
                expected,
                s.x,
                s.y
            );
        }
    }
}
