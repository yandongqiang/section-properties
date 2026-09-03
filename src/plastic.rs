//! Plastic and advanced analysis module.
//!
//! Provides plastic section analysis, interaction diagrams,
//! warping torsion, section classification (EN 1993), and advanced post-processing.

pub mod classification;
pub mod interaction;
pub mod plastic_section;
pub mod warping;
pub mod warping_fem;

pub use classification::{
    ClassLimit, SectionClass, SectionClassification, StressDistribution, classify_section,
    effective,
};
pub use interaction::{
    CapacityCheck, InteractionDiagram, InteractionPoint, LoadCase3D, aisc360, en1993,
};
pub use plastic_section::{
    FullPlasticProperties, PlasticAxis, PlasticNeutralAxis, PlasticProperties, PlasticSection,
};
pub use warping::{TorsionAnalysis, WarpingProperties, trefftz};

use crate::material::Material;
use crate::section::Section;

/// High-level plastic analysis for a section.
#[derive(Debug, Clone)]
pub struct PlasticAnalysis {
    pub section: Section,
    pub material: Material,
    pub plastic_section: PlasticSection,
    pub interaction: InteractionDiagram,
    pub warping: WarpingProperties,
}

impl PlasticAnalysis {
    /// Create a complete plastic analysis for a section.
    pub fn new(section: Section, material: Material) -> Self {
        let plastic_section = PlasticSection::new(section.clone(), material);
        let interaction = InteractionDiagram::new(section.clone(), material);
        let warping = WarpingProperties::from_section(&section, material.poissons_ratio);

        Self {
            section,
            material,
            plastic_section,
            interaction,
            warping,
        }
    }

    /// Full plastic properties (both axes).
    pub fn full_properties(&self) -> crate::plastic::plastic_section::FullPlasticProperties {
        self.plastic_section.full_plastic_properties()
    }

    /// Section classification per EN 1993-1-1.
    pub fn classification(&self, stress: StressDistribution) -> SectionClassification {
        classify_section(&self.section, &self.material, stress)
    }

    /// Check capacity for a load case.
    pub fn check_capacity(&self, load: LoadCase3D, gamma_m0: f64) -> CapacityCheck {
        self.interaction.check_capacity(load, gamma_m0)
    }

    /// Check capacity using exact plastic surface.
    pub fn check_capacity_exact(&self, load: LoadCase3D, gamma_m0: f64) -> CapacityCheck {
        self.interaction.check_capacity_exact(load, gamma_m0)
    }

    /// EN 1993 combined bending and axial check.
    pub fn check_en1993(
        &self,
        n_ed: f64,
        m_y_ed: f64,
        m_z_ed: f64,
        gamma_m0: f64,
        gamma_m1: f64,
    ) -> CapacityCheck {
        en1993::check_combined_bending_axial(
            &self.section,
            &self.material,
            n_ed,
            m_y_ed,
            m_z_ed,
            gamma_m0,
            gamma_m1,
        )
    }

    /// AISC 360 combined forces check.
    pub fn check_aisc360(
        &self,
        p_u: f64,
        m_ux: f64,
        m_uy: f64,
        phi_c: f64,
        phi_b: f64,
    ) -> CapacityCheck {
        aisc360::check_h1(&self.section, &self.material, p_u, m_ux, m_uy, phi_c, phi_b)
    }

    /// Pure torsion analysis.
    pub fn torsion_analysis(&self, torque: f64) -> TorsionAnalysis {
        TorsionAnalysis::pure_torsion(&self.section, torque, &self.material)
    }

    /// Constrained torsion analysis (torsion + warping).
    pub fn constrained_torsion_analysis(&self, torque: f64, bimoment: f64) -> TorsionAnalysis {
        TorsionAnalysis::constrained_torsion(&self.section, torque, bimoment, &self.material)
    }

    /// Warping properties.
    pub fn warping_properties(&self) -> &WarpingProperties {
        &self.warping
    }

    /// Shape factors.
    pub fn shape_factors(&self) -> (f64, f64) {
        let full = self.full_properties();
        (full.shape_factor_x, full.shape_factor_y)
    }

    /// Plastic moment capacities.
    pub fn plastic_moments(&self) -> (f64, f64) {
        let full = self.full_properties();
        (
            full.x_axis.plastic_moment_capacity,
            full.y_axis.plastic_moment_capacity,
        )
    }

    /// Yield moments.
    pub fn yield_moments(&self) -> (f64, f64) {
        let full = self.full_properties();
        (full.x_axis.yield_moment, full.y_axis.yield_moment)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon};
    use crate::material::presets::STEEL_S355;
    use crate::section::Section;
    use crate::section_library::ParametricSection;

    #[test]
    fn plastic_analysis_i_section() {
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();
        let analysis = PlasticAnalysis::new(section, STEEL_S355);

        let full = analysis.full_properties();
        assert!(full.shape_factor_x > 1.0);
        assert!(full.shape_factor_y > 1.0);

        let (mpl_x, mpl_y) = analysis.plastic_moments();
        let (my_x, my_y) = analysis.yield_moments();

        assert!(mpl_x > my_x);
        assert!(mpl_y > my_y);
    }

    #[test]
    fn plastic_analysis_capacity() {
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();
        let analysis = PlasticAnalysis::new(section, STEEL_S355);

        let full = analysis.full_properties();
        let load = LoadCase3D::new(
            full.x_axis.plastic_moment_capacity * 0.1,
            full.x_axis.plastic_moment_capacity * 0.8,
            0.0,
        );

        let check = analysis.check_capacity(load, 1.0);
        assert!(check.utilization > 0.0);
    }

    #[test]
    fn plastic_analysis_torsion() {
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();
        let analysis = PlasticAnalysis::new(section, STEEL_S355);

        let torsion = analysis.torsion_analysis(10e3);
        assert!(torsion.tau_sv_max > 0.0);
        assert!(torsion.theta_prime > 0.0);

        let constrained = analysis.constrained_torsion_analysis(10e3, 5e3);
        assert!(constrained.sigma_w_max >= 0.0);
    }

    #[test]
    fn plastic_analysis_rectangular() {
        let poly = Polygon::new(vec![
            Point::new(-0.1, -0.05),
            Point::new(0.1, -0.05),
            Point::new(0.1, 0.05),
            Point::new(-0.1, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let analysis = PlasticAnalysis::new(section, STEEL_S355);

        let (sf_x, sf_y) = analysis.shape_factors();
        assert!((sf_x - 1.5).abs() < 0.1);
        assert!((sf_y - 1.5).abs() < 0.1);
    }

    #[test]
    fn section_classification() {
        let i = crate::section_library::steel::ISection::from_designation("IPE300").unwrap();
        let section = i.build();
        let analysis = PlasticAnalysis::new(section, STEEL_S355);

        let class = analysis.classification(StressDistribution::pure_bending());
        assert!(class.overall_class >= crate::plastic::SectionClass::Class2);
        assert!(class.can_plastic_moment());
    }
}
