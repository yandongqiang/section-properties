//! Isotropic linear-elastic material definition.
//!
//! All values in SI base units (Pa, kg/m³, /K).
use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Material {
    /// Young's modulus [Pa]
    pub youngs_modulus: f64,
    /// Shear modulus [Pa]
    pub shear_modulus: f64,
    /// Poisson's ratio [-]
    pub poissons_ratio: f64,
    /// Density [kg/m³]
    pub density: f64,
    /// Coefficient of thermal expansion [1/K]
    pub thermal_expansion: f64,
    /// Yield strength [Pa] (for plastic analysis)
    pub yield_strength: f64,
    /// Ultimate tensile strength [Pa]
    pub ultimate_strength: f64,
    /// Optional: identifier/name for display
    pub name: &'static str,
    /// Optional: color for visualization (RGB 0-255)
    pub color: Option<(u8, u8, u8)>,
}

impl Material {
    /// Create a new material from E and ν (computes G = E / (2(1+ν))).
    pub fn new(youngs_modulus: f64, poissons_ratio: f64, density: f64, name: &'static str) -> Self {
        let shear_modulus = youngs_modulus / (2.0 * (1.0 + poissons_ratio));
        Self {
            youngs_modulus,
            shear_modulus,
            poissons_ratio,
            density,
            thermal_expansion: 0.0,
            yield_strength: 0.0,
            ultimate_strength: 0.0,
            name,
            color: None,
        }
    }

    /// Create a new material with all properties specified.
    pub fn with_all(
        youngs_modulus: f64,
        shear_modulus: f64,
        poissons_ratio: f64,
        density: f64,
        thermal_expansion: f64,
        yield_strength: f64,
        ultimate_strength: f64,
        name: &'static str,
    ) -> Self {
        Self {
            youngs_modulus,
            shear_modulus,
            poissons_ratio,
            density,
            thermal_expansion,
            yield_strength,
            ultimate_strength,
            name,
            color: None,
        }
    }

    /// Set thermal expansion coefficient.
    pub fn with_thermal_expansion(mut self, alpha: f64) -> Self {
        self.thermal_expansion = alpha;
        self
    }

    /// Set yield strength.
    pub fn with_yield_strength(mut self, fy: f64) -> Self {
        self.yield_strength = fy;
        self
    }

    /// Set ultimate strength.
    pub fn with_ultimate_strength(mut self, fu: f64) -> Self {
        self.ultimate_strength = fu;
        self
    }

    /// Set visualization color (RGB).
    pub fn with_color(mut self, r: u8, g: u8, b: u8) -> Self {
        self.color = Some((r, g, b));
        self
    }

    /// Get modular ratio relative to another material (n = E_this / E_ref).
    pub fn modular_ratio(&self, ref_material: &Material) -> f64 {
        self.youngs_modulus / ref_material.youngs_modulus
    }

    /// Check if material properties are physically valid.
    pub fn is_valid(&self) -> bool {
        self.youngs_modulus > 0.0
            && self.shear_modulus > 0.0
            && self.poissons_ratio >= -1.0
            && self.poissons_ratio < 0.5
            && self.density >= 0.0
    }
}

impl Default for Material {
    /// Default to generic structural steel (S355).
    fn default() -> Self {
        presets::STEEL_S355
    }
}

impl fmt::Display for Material {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} (E={:.0} GPa, ν={:.2}, ρ={:.0} kg/m³)",
            self.name,
            self.youngs_modulus / 1e9,
            self.poissons_ratio,
            self.density
        )
    }
}

/// Common structural materials (SI units).
pub mod presets {
    use super::Material;

    // Steel
    /// Generic structural steel S235 (EN 10025)
    pub const STEEL_S235: Material = Material {
        youngs_modulus: 200e9,
        shear_modulus: 76.923e9,
        poissons_ratio: 0.3,
        density: 7850.0,
        thermal_expansion: 12e-6,
        yield_strength: 235e6,
        ultimate_strength: 360e6,
        name: "S235",
        color: Some((120, 120, 140)),
    };

    /// Generic structural steel S275 (EN 10025)
    pub const STEEL_S275: Material = Material {
        youngs_modulus: 200e9,
        shear_modulus: 76.923e9,
        poissons_ratio: 0.3,
        density: 7850.0,
        thermal_expansion: 12e-6,
        yield_strength: 275e6,
        ultimate_strength: 430e6,
        name: "S275",
        color: Some((130, 130, 150)),
    };

    /// Generic structural steel S355 (EN 10025) - most common
    pub const STEEL_S355: Material = Material {
        youngs_modulus: 200e9,
        shear_modulus: 76.923e9,
        poissons_ratio: 0.3,
        density: 7850.0,
        thermal_expansion: 12e-6,
        yield_strength: 355e6,
        ultimate_strength: 510e6,
        name: "S355",
        color: Some((140, 140, 160)),
    };

    /// Generic structural steel S460 (EN 10025)
    pub const STEEL_S460: Material = Material {
        youngs_modulus: 200e9,
        shear_modulus: 76.923e9,
        poissons_ratio: 0.3,
        density: 7850.0,
        thermal_expansion: 12e-6,
        yield_strength: 460e6,
        ultimate_strength: 620e6,
        name: "S460",
        color: Some((150, 150, 170)),
    };

    /// Stainless steel 304 (EN 1.4301)
    pub const STAINLESS_304: Material = Material {
        youngs_modulus: 193e9,
        shear_modulus: 74.23e9,
        poissons_ratio: 0.3,
        density: 7930.0,
        thermal_expansion: 17.2e-6,
        yield_strength: 215e6,
        ultimate_strength: 505e6,
        name: "304 Stainless",
        color: Some((180, 180, 190)),
    };

    /// Stainless steel 316 (EN 1.4401)
    pub const STAINLESS_316: Material = Material {
        youngs_modulus: 193e9,
        shear_modulus: 74.23e9,
        poissons_ratio: 0.3,
        density: 7980.0,
        thermal_expansion: 15.9e-6,
        yield_strength: 205e6,
        ultimate_strength: 515e6,
        name: "316 Stainless",
        color: Some((185, 185, 195)),
    };

    // Aluminum
    /// Aluminum 6061-T6
    pub const ALUMINUM_6061_T6: Material = Material {
        youngs_modulus: 68.9e9,
        shear_modulus: 26.0e9,
        poissons_ratio: 0.33,
        density: 2700.0,
        thermal_expansion: 23.6e-6,
        yield_strength: 276e6,
        ultimate_strength: 310e6,
        name: "6061-T6 Al",
        color: Some((200, 200, 210)),
    };

    /// Aluminum 6063-T5
    pub const ALUMINUM_6063_T5: Material = Material {
        youngs_modulus: 68.3e9,
        shear_modulus: 25.7e9,
        poissons_ratio: 0.33,
        density: 2700.0,
        thermal_expansion: 23.4e-6,
        yield_strength: 145e6,
        ultimate_strength: 186e6,
        name: "6063-T5 Al",
        color: Some((205, 205, 215)),
    };

    /// Aluminum 7075-T6
    pub const ALUMINUM_7075_T6: Material = Material {
        youngs_modulus: 71.7e9,
        shear_modulus: 26.9e9,
        poissons_ratio: 0.33,
        density: 2810.0,
        thermal_expansion: 23.6e-6,
        yield_strength: 503e6,
        ultimate_strength: 572e6,
        name: "7075-T6 Al",
        color: Some((210, 210, 220)),
    };

    // Concrete
    /// Concrete C25/30 (EN 1992)
    pub const CONCRETE_C25_30: Material = Material {
        youngs_modulus: 31e9,
        shear_modulus: 12.9e9,
        poissons_ratio: 0.2,
        density: 2400.0,
        thermal_expansion: 10e-6,
        yield_strength: 25e6, // f_ck
        ultimate_strength: 30e6,
        name: "C25/30 Concrete",
        color: Some((180, 170, 160)),
    };

    /// Concrete C30/37
    pub const CONCRETE_C30_37: Material = Material {
        youngs_modulus: 33e9,
        shear_modulus: 13.75e9,
        poissons_ratio: 0.2,
        density: 2400.0,
        thermal_expansion: 10e-6,
        yield_strength: 30e6,
        ultimate_strength: 37e6,
        name: "C30/37 Concrete",
        color: Some((175, 165, 155)),
    };

    /// Concrete C40/50
    pub const CONCRETE_C40_50: Material = Material {
        youngs_modulus: 35e9,
        shear_modulus: 14.58e9,
        poissons_ratio: 0.2,
        density: 2450.0,
        thermal_expansion: 10e-6,
        yield_strength: 40e6,
        ultimate_strength: 50e6,
        name: "C40/50 Concrete",
        color: Some((170, 160, 150)),
    };

    // Timber (GL24h)
    /// Glued laminated timber GL24h (EN 14080)
    pub const TIMBER_GL24H: Material = Material {
        youngs_modulus: 11.6e9,
        shear_modulus: 0.725e9,
        poissons_ratio: 0.35,
        density: 420.0,
        thermal_expansion: 5e-6,
        yield_strength: 24e6, // f_m,k
        ultimate_strength: 24e6,
        name: "GL24h Timber",
        color: Some((200, 170, 120)),
    };

    // Titanium
    /// Titanium Grade 5 (Ti-6Al-4V)
    pub const TITANIUM_GR5: Material = Material {
        youngs_modulus: 113.8e9,
        shear_modulus: 44.0e9,
        poissons_ratio: 0.342,
        density: 4430.0,
        thermal_expansion: 8.6e-6,
        yield_strength: 880e6,
        ultimate_strength: 950e6,
        name: "Ti-6Al-4V",
        color: Some((160, 160, 170)),
    };
}

/// Material group for composite sections (transformed section method).
///
/// Groups elements sharing the same material reference for stiffness assembly.
#[derive(Debug, Clone, PartialEq)]
pub struct MaterialGroup {
    pub material: Material,
    /// List of polygon indices belonging to this material group
    pub polygon_indices: Vec<usize>,
}

impl MaterialGroup {
    pub fn new(material: Material, polygon_indices: Vec<usize>) -> Self {
        Self {
            material,
            polygon_indices,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::presets::*;
    use super::*;

    #[test]
    fn steel_material_properties() {
        let mat = STEEL_S355;
        assert!((mat.youngs_modulus - 200e9).abs() < 1e6);
        assert!((mat.poissons_ratio - 0.3).abs() < 1e-9);
        assert!(mat.is_valid());
    }

    #[test]
    fn modular_ratio() {
        let steel = STEEL_S355;
        let concrete = CONCRETE_C30_37;
        let n = steel.modular_ratio(&concrete);
        assert!((n - 200.0 / 33.0).abs() < 0.1); // ≈ 6.06
    }

    #[test]
    fn material_builder() {
        let mat = Material::new(210e9, 0.3, 7850.0, "Custom Steel")
            .with_yield_strength(400e6)
            .with_color(255, 0, 0);
        assert_eq!(mat.yield_strength, 400e6);
        assert_eq!(mat.color, Some((255, 0, 0)));
    }
}
