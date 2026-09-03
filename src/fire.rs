//! Fire resistance analysis per EN 1993-1-2, EN 1994-1-2, EN 1992-1-2.
//!
//! Provides section factor (A_m/V), temperature analysis, and
//! load-bearing resistance in fire.

use crate::material::Material;
use crate::section::Section;
use crate::section_properties::SectionProperties;

/// Fire exposure curves.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FireExposure {
    /// Standard ISO 834 fire
    Standard,
    /// Hydrocarbon fire
    Hydrocarbon,
    /// External fire (EN 1991-1-2)
    External,
    /// Parametric fire
    Parametric { opening_factor: f64, fire_load: f64 },
}

impl FireExposure {
    /// Temperature at time t (minutes) for standard fire.
    pub fn temperature(&self, t_min: f64) -> f64 {
        match self {
            FireExposure::Standard => {
                // ISO 834: T = 20 + 345 * log10(8*t + 1)
                20.0 + 345.0 * (8.0 * t_min + 1.0).log10()
            }
            FireExposure::Hydrocarbon => {
                // Hydrocarbon: T = 20 + 1080 * (1 - 0.325*exp(-0.167*t) - 0.675*exp(-2.5*t))
                20.0 + 1080.0
                    * (1.0 - 0.325 * (-0.167 * t_min).exp() - 0.675 * (-2.5 * t_min).exp())
            }
            FireExposure::External => {
                // Simplified external fire
                20.0 + 660.0 * (1.0 - (-0.1 * t_min).exp())
            }
            FireExposure::Parametric {
                opening_factor,
                fire_load,
            } => {
                // Simplified parametric
                let t_eq = fire_load / (opening_factor * 1000.0); // equivalent time
                20.0 + 345.0 * (8.0 * t_eq + 1.0).log10()
            }
        }
    }
}

/// Section factor A_m/V (heated perimeter / cross-sectional area).
#[derive(Debug, Clone)]
pub struct SectionFactor {
    pub am_v: f64,            // [1/m] - section factor
    pub am: f64,              // [m²/m] - heated perimeter per unit length
    pub v: f64,               // [m²] - cross-sectional area
    pub shadow_factor: f64,   // Shadow effect factor (k_sh)
    pub box_protection: bool, // Box protection (k_sh = 1.0)
}

impl SectionFactor {
    /// Compute section factor for a section.
    pub fn from_section(section: &Section, fire_exposure: FireExposure) -> Self {
        let props = SectionProperties::from_section(section);
        let v = props.area;

        // Heated perimeter depends on fire exposure
        let am = match fire_exposure {
            FireExposure::Standard | FireExposure::Hydrocarbon => {
                // 3-sided or 4-sided exposure
                section.heated_perimeter_3sided()
            }
            FireExposure::External => section.heated_perimeter_4sided(),
            FireExposure::Parametric { .. } => section.heated_perimeter_4sided(),
        };

        let am_v = am / v;

        // Shadow factor for I-sections
        let shadow_factor = section.shadow_factor();

        Self {
            am_v,
            am,
            v,
            shadow_factor,
            box_protection: false,
        }
    }

    /// For box protection (uniform heating all around).
    pub fn with_box_protection(mut self) -> Self {
        self.box_protection = true;
        self.shadow_factor = 1.0;
        self
    }

    /// For fire protection (intumescent, board, spray).
    pub fn with_protection(
        mut self,
        thickness: f64,
        conductivity: f64,
        density: f64,
        specific_heat: f64,
    ) -> Self {
        self.am_v =
            self.am / self.v * (1.0 / (1.0 + thickness * conductivity / (density * specific_heat)));
        self
    }
}

/// Temperature distribution in section at given time.
#[derive(Debug, Clone)]
pub struct TemperatureProfile {
    pub time: f64, // Time [min]
    pub exposure: FireExposure,
    pub temps: Vec<f64>, // Temperature at each fiber/node [°C]
    pub max_temp: f64,
    pub avg_temp: f64,
}

impl TemperatureProfile {
    /// Uniform temperature (simplified).
    pub fn uniform(temp: f64, time: f64, exposure: FireExposure) -> Self {
        Self {
            time,
            exposure,
            temps: vec![temp],
            max_temp: temp,
            avg_temp: temp,
        }
    }

    /// Compute temperature profile using heat transfer analysis.
    pub fn from_heat_transfer(
        _section: &Section,
        _material: &Material,
        _section_factor: &SectionFactor,
        exposure: FireExposure,
        time: f64,
        _time_step: f64,
    ) -> Self {
        // Simplified lumped capacitance method
        // dT/dt = (h_net * A_m/V) * (T_g - T) / (rho * c_a)

        let t_g = match exposure {
            FireExposure::Standard => 345.0 * (8.0 * time + 1.0).log10(),
            _ => 20.0 + 660.0 * (1.0 - (-0.1 * time).exp()),
        };

        // Use average temperature
        let temp = t_g * (1.0 - (-time / 30.0).exp()) + 20.0;

        Self::uniform(temp, time, exposure)
    }
}

/// Material properties at elevated temperature.
#[derive(Debug, Clone)]
pub struct MaterialPropertiesAtTemp {
    pub temperature: f64,
    pub ky: f64, // Reduction factor for yield strength
    pub kp: f64, // Reduction factor for proportional limit
    pub ke: f64, // Reduction factor for Young's modulus
    pub thermal_elongation: f64,
}

impl MaterialPropertiesAtTemp {
    /// Carbon steel per EN 1993-1-2 Table 3.1.
    pub fn carbon_steel(temp: f64) -> Self {
        let (ky, kp, ke) = if temp <= 100.0 {
            (1.0, 1.0, 1.0)
        } else if temp <= 200.0 {
            (1.0, 0.81, 0.9)
        } else if temp <= 300.0 {
            (1.0, 0.61, 0.8)
        } else if temp <= 400.0 {
            (1.0, 0.42, 0.7)
        } else if temp <= 500.0 {
            (0.78, 0.31, 0.6)
        } else if temp <= 600.0 {
            (0.47, 0.13, 0.31)
        } else if temp <= 700.0 {
            (0.23, 0.09, 0.13)
        } else if temp <= 800.0 {
            (0.11, 0.07, 0.09)
        } else if temp <= 900.0 {
            (0.06, 0.05, 0.07)
        } else if temp <= 1000.0 {
            (0.04, 0.04, 0.05)
        } else {
            (0.02, 0.02, 0.03)
        };

        let thermal_elongation = if temp <= 750.0 {
            1.2e-5 * temp + 0.4e-8 * temp * temp - 2.416e-4
        } else {
            1.1e-2
        };

        Self {
            temperature: temp,
            ky,
            kp,
            ke,
            thermal_elongation,
        }
    }

    /// Stainless steel per EN 1993-1-2 Table C.1.
    pub fn stainless_steel(temp: f64) -> Self {
        // Simplified - similar to carbon but different curve
        let ky = if temp <= 100.0 {
            1.0
        } else if temp <= 500.0 {
            0.8
        } else if temp <= 700.0 {
            0.5
        } else {
            0.2
        };
        let kp = if temp <= 100.0 {
            1.0
        } else if temp <= 500.0 {
            0.6
        } else if temp <= 700.0 {
            0.3
        } else {
            0.1
        };
        let ke = if temp <= 100.0 {
            1.0
        } else if temp <= 500.0 {
            0.8
        } else if temp <= 700.0 {
            0.4
        } else {
            0.2
        };

        Self {
            temperature: temp,
            ky,
            kp,
            ke,
            thermal_elongation: 0.0,
        }
    }

    /// Concrete per EN 1992-1-2.
    pub fn concrete(temp: f64, _fc20: f64) -> Self {
        let kc = if temp <= 100.0 {
            1.0
        } else if temp <= 200.0 {
            0.95
        } else if temp <= 400.0 {
            0.75
        } else if temp <= 600.0 {
            0.45
        } else if temp <= 800.0 {
            0.25
        } else {
            0.1
        };

        Self {
            temperature: temp,
            ky: kc,
            kp: kc,
            ke: kc,
            thermal_elongation: 0.0,
        }
    }
}

/// Fire design method.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FireDesignMethod {
    /// Simple calculation model (EN 1993-1-2 4.2)
    Simple,
    /// Advanced calculation model (EN 1993-1-2 4.3)
    Advanced,
    /// Tabulated data (EN 1993-1-2 4.4)
    Tabulated,
}

/// Fire analysis for a section.
#[derive(Debug, Clone)]
pub struct FireAnalysis {
    pub section: Section,
    pub material: Material,
    pub exposure: FireExposure,
    pub method: FireDesignMethod,
    pub fire_protection: Option<FireProtection>,
    pub critical_temp: Option<f64>,
}

/// Fire protection system.
#[derive(Debug, Clone)]
pub struct FireProtection {
    pub protection_type: ProtectionType,
    pub thickness: f64,        // [m]
    pub conductivity: f64,     // [W/mK]
    pub density: f64,          // [kg/m³]
    pub specific_heat: f64,    // [J/kgK]
    pub moisture_content: f64, // [kg/m³] for gypsum/board
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ProtectionType {
    None,
    Intumescent,
    Board,
    Spray,
    ConcreteEncasement,
    HollowFilled, // Concrete-filled tube
}

impl FireAnalysis {
    pub fn new(section: Section, material: Material) -> Self {
        Self {
            section,
            material,
            exposure: FireExposure::Standard,
            method: FireDesignMethod::Simple,
            fire_protection: None,
            critical_temp: None,
        }
    }

    pub fn with_exposure(mut self, exposure: FireExposure) -> Self {
        self.exposure = exposure;
        self
    }

    pub fn with_method(mut self, method: FireDesignMethod) -> Self {
        self.method = method;
        self
    }

    pub fn with_protection(mut self, protection: FireProtection) -> Self {
        self.fire_protection = Some(protection);
        self
    }

    /// Section factor A_m/V.
    pub fn section_factor(&self) -> SectionFactor {
        SectionFactor::from_section(&self.section, self.exposure)
    }

    /// Critical temperature for given load ratio.
    pub fn critical_temperature(&self, load_ratio: f64) -> f64 {
        // EN 1993-1-2 Eq 4.25 for carbon steel
        if load_ratio <= 0.01 {
            return 20.0;
        }
        if load_ratio >= 1.0 {
            return 1200.0;
        }

        // Approximate: theta_cr = 39.19 * ln(1/(0.9674*mu0^3.833)) + 482
        // where mu0 = load_ratio
        let mu0 = load_ratio;
        39.19 * (1.0 / (0.9674 * mu0.powf(3.833))).ln() + 482.0
    }

    /// Time to reach critical temperature.
    pub fn fire_resistance_time(&self, load_ratio: f64) -> f64 {
        let theta_cr = self.critical_temperature(load_ratio);
        let section_factor = self.section_factor();

        // With protection
        if let Some(prot) = &self.fire_protection {
            return self.protected_time(theta_cr, section_factor, prot);
        }

        // Unprotected - use section factor
        self.unprotected_time(theta_cr, section_factor)
    }

    fn unprotected_time(&self, theta_cr: f64, sf: SectionFactor) -> f64 {
        // Numerical integration of temperature rise
        let dt = 1.0 / 60.0; // 1 second steps (in minutes)
        let mut t = 0.0;
        let mut theta_a = 20.0;

        while theta_a < theta_cr && t < 300.0 {
            // Max 5 hours
            let theta_g = self.exposure.temperature(t); // t is in minutes

            // Net heat flux
            let h_net = 25.0; // W/m²K (convection + radiation)

            // Steel properties
            let rho_a = 7850.0; // kg/m³
            let c_a = 600.0; // J/kgK (approx at 400°C)

            // Temperature rise
            let dtheta = h_net * sf.am_v / (rho_a * c_a) * (theta_g - theta_a) * dt * 60.0;
            theta_a += dtheta;
            t += dt;
        }

        t // Return in minutes
    }

    fn protected_time(&self, theta_cr: f64, sf: SectionFactor, prot: &FireProtection) -> f64 {
        // EN 1993-1-2 Annex G - protected members
        let dt = 1.0 / 60.0;
        let mut t = 0.0;
        let mut theta_a = 20.0;
        let mut theta_p = 20.0; // Protection temperature

        let rho_p = prot.density;
        let c_p = prot.specific_heat;
        let lambda_p = prot.conductivity;
        let d_p = prot.thickness;

        while theta_a < theta_cr && t < 300.0 {
            let theta_g = self.exposure.temperature(t / 60.0);

            // Heat flux through protection
            let h_net = 25.0;
            let phi = h_net * (theta_g - theta_p) / (1.0 + lambda_p / (h_net * d_p));

            // Protection temperature
            let dtheta_p = phi / (rho_p * c_p * d_p) * dt * 60.0;
            theta_p += dtheta_p;

            // Steel temperature
            let rho_a = 7850.0;
            let c_a = 600.0;
            let dtheta_a = phi / (rho_a * c_a * sf.v) * dt * 60.0;
            theta_a += dtheta_a;

            t += dt;
        }

        t * 60.0
    }

    /// Load-bearing resistance at time t (simplified).
    pub fn resistance_at_time(&self, t_min: f64, load_ratio: f64) -> FireResistanceResult {
        let theta_cr = self.critical_temperature(load_ratio);
        let theta_g = self.exposure.temperature(t_min);
        let section_factor = self.section_factor();

        // Section temperature
        let temp_profile = TemperatureProfile::from_heat_transfer(
            &self.section,
            &self.material,
            &section_factor,
            self.exposure,
            t_min,
            1.0,
        );

        let theta_a = temp_profile.avg_temp;
        let mat_props = MaterialPropertiesAtTemp::carbon_steel(theta_a);

        // Reduced yield strength
        let fy_theta = self.material.yield_strength * mat_props.ky;
        let e_theta = self.material.youngs_modulus * mat_props.ke;

        // Section properties at temperature (simplified - no degradation of geometry)
        let props = SectionProperties::from_section(&self.section);

        // Moment capacity at temperature
        let m_rd_theta = props.ix / props.max_fiber_distance_y() * fy_theta;

        // Axial capacity
        let n_rd_theta = props.area * fy_theta;

        // Utilization
        let utilization = if load_ratio > 0.0 {
            (theta_a - 20.0) / (theta_cr - 20.0)
        } else {
            0.0
        };

        FireResistanceResult {
            time: t_min,
            section_temp: theta_a,
            gas_temp: theta_g,
            fy_reduced: fy_theta,
            e_reduced: e_theta,
            moment_capacity: m_rd_theta,
            axial_capacity: n_rd_theta,
            utilization,
            passed: utilization <= 1.0,
        }
    }
}

/// Fire resistance result.
#[derive(Debug, Clone)]
pub struct FireResistanceResult {
    pub time: f64,            // Time [min]
    pub section_temp: f64,    // Average section temperature [°C]
    pub gas_temp: f64,        // Gas temperature [°C]
    pub fy_reduced: f64,      // Reduced yield strength [Pa]
    pub e_reduced: f64,       // Reduced Young's modulus [Pa]
    pub moment_capacity: f64, // Moment capacity at temperature [Nm]
    pub axial_capacity: f64,  // Axial capacity at temperature [N]
    pub utilization: f64,     // Utilization ratio
    pub passed: bool,         // Passed fire resistance?
}

/// Composite column fire resistance (EN 1994-1-2).
pub mod composite {
    use super::*;
    use crate::material::Material;
    use crate::section::Section;

    /// Concrete-filled tube fire analysis.
    pub struct CFTFireAnalysis {
        pub steel_section: Section,
        pub concrete_section: Section,
        pub steel_material: Material,
        pub concrete_material: Material,
        pub exposure: FireExposure,
        pub reinforcement: Option<Reinforcement>,
    }

    #[derive(Debug, Clone)]
    pub struct Reinforcement {
        pub area: f64,
        pub distance_from_center: f64,
        pub material: Material,
    }

    impl CFTFireAnalysis {
        pub fn new(
            steel: Section,
            concrete: Section,
            steel_mat: Material,
            conc_mat: Material,
        ) -> Self {
            Self {
                steel_section: steel,
                concrete_section: concrete,
                steel_material: steel_mat,
                concrete_material: conc_mat,
                exposure: FireExposure::Standard,
                reinforcement: None,
            }
        }

        /// Fire resistance per EN 1994-1-2.
        pub fn fire_resistance(&self, n_ed: f64, m_ed: f64) -> FireResistanceResult {
            // Simplified: use concrete core temperature
            // Concrete protects steel tube

            let section_factor = SectionFactor::from_section(&self.steel_section, self.exposure);
            let theta_cr = self.critical_temperature_composite(n_ed);

            // Time to reach critical temp
            let t_fi = self.unprotected_time(theta_cr, section_factor);

            FireResistanceResult {
                time: t_fi,
                section_temp: theta_cr,
                gas_temp: self.exposure.temperature(t_fi / 60.0),
                fy_reduced: self.steel_material.yield_strength * 0.5, // Approx
                e_reduced: self.steel_material.youngs_modulus * 0.5,
                moment_capacity: m_ed, // Placeholder
                axial_capacity: n_ed,
                utilization: 1.0,
                passed: t_fi >= 30.0, // R30 minimum
            }
        }

        fn critical_temperature_composite(&self, _n_ed: f64) -> f64 {
            // Concrete contribution delays steel heating
            // Approximate: 500-600°C for typical concrete-filled tubes
            550.0
        }

        fn unprotected_time(&self, theta_cr: f64, sf: SectionFactor) -> f64 {
            let dt = 1.0 / 60.0;
            let mut t = 0.0;
            let mut theta_a = 20.0;

            while theta_a < theta_cr && t < 300.0 {
                let theta_g = self.exposure.temperature(t / 60.0);
                let h_net = 25.0;
                let rho_a = 7850.0;
                let c_a = 600.0;
                let dtheta = h_net * sf.am_v / (rho_a * c_a) * (theta_g - theta_a) * dt * 60.0;
                theta_a += dtheta;
                t += dt;
            }
            t * 60.0
        }
    }
}

trait SectionFireProps {
    fn heated_perimeter_3sided(&self) -> f64;
    fn heated_perimeter_4sided(&self) -> f64;
    fn shadow_factor(&self) -> f64;
}

impl SectionFireProps for Section {
    fn heated_perimeter_3sided(&self) -> f64 {
        // Perimeter exposed on 3 sides (bottom not exposed)
        let bounds = self.bounds();
        let h = bounds.3 - bounds.2; // max_y - min_y = height

        let mut perimeter = 0.0;

        for i in 0..self.outer.vertices.len() {
            let v1 = self.outer.vertices[i];
            let v2 = self.outer.vertices[(i + 1) % self.outer.vertices.len()];

            let mid_y = (v1.y + v2.y) / 2.0;
            if mid_y > bounds.2 + 0.01 * h {
                let dx = v2.x - v1.x;
                let dy = v2.y - v1.y;
                perimeter += (dx * dx + dy * dy).sqrt();
            }
        }

        perimeter.max(0.01)
    }

    fn heated_perimeter_4sided(&self) -> f64 {
        let mut perimeter = 0.0;

        for i in 0..self.outer.vertices.len() {
            let v1 = self.outer.vertices[i];
            let v2 = self.outer.vertices[(i + 1) % self.outer.vertices.len()];
            let dx = v2.x - v1.x;
            let dy = v2.y - v1.y;
            perimeter += (dx * dx + dy * dy).sqrt();
        }

        perimeter
    }

    fn shadow_factor(&self) -> f64 {
        // k_sh for I-sections (shadow effect)
        // k_sh = 1 - (b / h) * (t_f / t_w) * 0.25 (simplified)
        let bounds = self.bounds();
        let h = bounds.3 - bounds.2;
        let b = bounds.1 - bounds.0;

        if h > b * 2.0 {
            // I-section like
            0.9
        } else {
            1.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::material::presets::STEEL_S355;

    use crate::section_library::ParametricSection;

    #[test]
    fn section_factor_i_section() {
        let i = crate::section_library::steel::ISection::from_designation("IPE300").unwrap();
        let section = i.build();

        let sf = SectionFactor::from_section(&section, FireExposure::Standard);

        assert!(sf.am_v > 0.0);
        assert!(sf.am > 0.0);
        assert!(sf.v > 0.0);
    }

    #[test]
    fn section_factor_hollow() {
        let rhs = crate::section_library::steel::RectangularHollowSection::new(
            0.2, 0.1, 0.005, 0.008, 0.003,
        );
        let section = rhs.build();

        let sf = SectionFactor::from_section(&section, FireExposure::Standard);

        // Hollow sections have lower A_m/V
        assert!(sf.am_v < 300.0);
    }

    #[test]
    fn critical_temperature() {
        let i = crate::section_library::steel::ISection::from_designation("IPE300").unwrap();
        let section = i.build();

        let analysis = FireAnalysis::new(section, STEEL_S355);

        // Low load ratio -> high critical temp
        let theta_cr_low = analysis.critical_temperature(0.1);
        let theta_cr_high = analysis.critical_temperature(0.7);

        assert!(theta_cr_low > theta_cr_high);
        assert!(theta_cr_low > 600.0);
        assert!(theta_cr_high < 600.0);
    }

    #[test]
    fn fire_resistance_time() {
        let i = crate::section_library::steel::ISection::from_designation("IPE300").unwrap();
        let section = i.build();

        let analysis = FireAnalysis::new(section, STEEL_S355);
        let t_fi = analysis.fire_resistance_time(0.5);

        // IPE300 unprotected ~15-20 min for 50% load
        assert!(t_fi > 10.0);
        assert!(t_fi < 60.0);
    }

    #[test]
    fn fire_protection() {
        let i = crate::section_library::steel::ISection::from_designation("IPE300").unwrap();
        let section = i.build();

        let protection = FireProtection {
            protection_type: ProtectionType::Board,
            thickness: 0.02, // 20mm
            conductivity: 0.2,
            density: 800.0,
            specific_heat: 1000.0,
            moisture_content: 0.0,
        };

        let analysis = FireAnalysis::new(section, STEEL_S355).with_protection(protection);
        let t_fi = analysis.fire_resistance_time(0.5);

        // With 20mm board protection -> 60+ minutes
        assert!(t_fi > 45.0);
    }

    #[test]
    fn material_props_at_temp() {
        let props_20 = MaterialPropertiesAtTemp::carbon_steel(20.0);
        assert_eq!(props_20.ky, 1.0);
        assert_eq!(props_20.ke, 1.0);

        let props_600 = MaterialPropertiesAtTemp::carbon_steel(600.0);
        assert!(props_600.ky < 0.5);
        assert!(props_600.ke < 0.5);
    }

    #[test]
    fn concrete_material_props() {
        let props = MaterialPropertiesAtTemp::concrete(400.0, 30e6);
        assert!(props.ky < 1.0 && props.ky > 0.0);
    }

    #[test]
    fn composite_column() {
        let steel = crate::section_library::steel::CircularHollowSectionLib::new(0.219, 0.008);
        let concrete = crate::section_library::concrete::CircularConcreteSection::new(0.203);

        let cft = composite::CFTFireAnalysis::new(
            steel.build(),
            concrete.build(),
            STEEL_S355,
            crate::material::presets::CONCRETE_C30_37,
        );

        let result = cft.fire_resistance(500e3, 10e3);
        assert!(result.time > 0.0);
    }

    #[test]
    fn shadow_factor_i_section_depth_gt_2x_width() {
        // Regression test: h and b were swapped in shadow_factor.
        // For I-section with depth=0.3, width=0.1: h > 2*b -> 0.3 > 0.2 -> true -> 0.9
        // With bug: h=0.1 (width), b=0.3 (depth) -> 0.1 > 0.6 -> false -> 1.0
        let i = crate::section_library::steel::ISection::new(0.3, 0.1, 0.008, 0.01, 0.012);
        let section = i.build();
        let sf = section.shadow_factor();
        assert!(
            (sf - 0.9).abs() < 1e-10,
            "shadow_factor should be 0.9 for I-section with depth > 2*width, got {}",
            sf
        );
    }
}
