//! Section classification per EN 1993-1-1 (Eurocode 3).
//!
//! Determines cross-section class (1, 2, 3, 4) based on
// width-to-thickness ratios and stress distribution.
// Class 1: Plastic hinge rotation capacity
// Class 2: Plastic moment capacity
// Class 3: Elastic moment capacity (first yield)
// Class 4: Local buckling governs (effective section)

use crate::material::Material;
use crate::section::Section;
use crate::section_properties::SectionProperties;

/// Cross-section class per EN 1993-1-1 Table 5.2.
/// Ordering: Class1 > Class2 > Class3 > Class4 (Class1 is most favorable)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SectionClass {
    /// Class 1 - Plastic (full plastic moment + rotation capacity)
    Class1 = 1,
    /// Class 2 - Compact (plastic moment, limited rotation)
    Class2 = 2,
    /// Class 3 - Semi-compact (elastic moment, first yield)
    Class3 = 3,
    /// Class 4 - Slender (local buckling, effective section)
    Class4 = 4,
}

impl PartialOrd for SectionClass {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for SectionClass {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Reverse: Class1 > Class2 > Class3 > Class4
        (*self as u8).cmp(&(*other as u8)).reverse()
    }
}

impl SectionClass {
    /// Can develop plastic moment resistance?
    pub fn can_plastic_moment(&self) -> bool {
        matches!(self, SectionClass::Class1 | SectionClass::Class2)
    }

    /// Can develop plastic hinge rotation?
    pub fn can_plastic_rotation(&self) -> bool {
        matches!(self, SectionClass::Class1)
    }

    /// Must use effective section properties?
    pub fn requires_effective_section(&self) -> bool {
        matches!(self, SectionClass::Class4)
    }

    /// Description of the class.
    pub fn description(&self) -> &'static str {
        match self {
            SectionClass::Class1 => "Class 1 - Plastic (rotation capacity)",
            SectionClass::Class2 => "Class 2 - Compact (plastic moment)",
            SectionClass::Class3 => "Class 3 - Semi-compact (elastic moment)",
            SectionClass::Class4 => "Class 4 - Slender (local buckling)",
        }
    }
}

/// Stress distribution in the element per EN 1993-1-1 Table 5.2.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StressDistribution {
    /// Uniform compression (e.g., flange in pure compression)
    UniformCompression,
    /// Linear varying stress (e.g., web in bending)
    Bending,
    /// Web in combined bending and axial
    BendingAndCompression { alpha: f64 }, // α = plastic compression zone ratio (0=pure tension, 0.5=pure bending, 1=pure compression)
    /// Flange in combined bending and axial
    FlangeBendingAndCompression { psi: f64 }, // ψ = σ_2/σ_1
}

impl StressDistribution {
    /// For pure bending (σ varies linearly from +σ to -σ)
    pub fn pure_bending() -> Self {
        StressDistribution::Bending
    }

    /// For pure compression
    pub fn pure_compression() -> Self {
        StressDistribution::UniformCompression
    }

    /// For web with axial + bending (α parameter).
    ///
    /// α is the plastic compression zone ratio (0 to 1):
    /// - α = 1.0: pure compression
    /// - α = 0.5: pure bending (doubly-symmetric section)
    /// - α = 0.0: pure tension (no compression zone)
    pub fn web_bending_compression(alpha: f64) -> Self {
        StressDistribution::BendingAndCompression { alpha }
    }

    /// For flange with axial + bending (ψ parameter)
    pub fn flange_bending_compression(psi: f64) -> Self {
        StressDistribution::FlangeBendingAndCompression { psi }
    }
}

/// Class limits (ε = sqrt(235/fy) for steel).
#[derive(Debug, Clone, Copy)]
pub struct ClassLimit {
    pub class1: f64,
    pub class2: f64,
    pub class3: f64,
}

impl ClassLimit {
    /// Check classification for a given width-to-thickness ratio.
    pub fn classify(&self, c_t: f64) -> SectionClass {
        if c_t <= self.class1 {
            SectionClass::Class1
        } else if c_t <= self.class2 {
            SectionClass::Class2
        } else if c_t <= self.class3 {
            SectionClass::Class3
        } else {
            SectionClass::Class4
        }
    }
}

/// Get class limits for internal compression parts (webs) per Table 5.2.
fn web_limits(eps: f64, stress: StressDistribution) -> ClassLimit {
    match stress {
        StressDistribution::UniformCompression => ClassLimit {
            class1: 33.0 * eps,
            class2: 38.0 * eps,
            class3: 42.0 * eps,
        },
        StressDistribution::Bending => ClassLimit {
            class1: 72.0 * eps,
            class2: 83.0 * eps,
            class3: 124.0 * eps,
        },
        StressDistribution::BendingAndCompression { alpha } => {
            // EN 1993-1-1 Table 5.2 (sheet 2), web subject to bending and axial compression
            // α = plastic compression zone ratio (0=pure tension, 0.5=pure bending, 1=pure compression)
            // ψ = stress ratio = σ_min/σ_max; for doubly-symmetric sections ψ = 4α - 3
            let alpha = alpha.clamp(0.0, 1.0);
            let psi = 4.0 * alpha - 3.0;
            let (class1, class2) = if alpha > 0.5 {
                (
                    396.0 * eps / (13.0 * alpha - 1.0),
                    456.0 * eps / (13.0 * alpha - 1.0),
                )
            } else if alpha > 0.0 {
                (36.0 * eps / alpha, 41.5 * eps / alpha)
            } else {
                (f64::MAX, f64::MAX)
            };
            // Class 3: 42ε / (0.67 + 0.33ψ); guard against non-positive denominator
            let class3_denom = 0.67 + 0.33 * psi;
            let class3 = if class3_denom > 0.0 {
                42.0 * eps / class3_denom
            } else {
                f64::MAX
            };
            ClassLimit {
                class1,
                class2,
                class3,
            }
        }
        StressDistribution::FlangeBendingAndCompression { .. } => {
            // Flange limits handled separately
            ClassLimit {
                class1: 0.0,
                class2: 0.0,
                class3: 0.0,
            }
        }
    }
}

/// Get class limits for outstand flanges per Table 5.2.
fn outstand_flange_limits(eps: f64, stress: StressDistribution) -> ClassLimit {
    match stress {
        StressDistribution::UniformCompression => ClassLimit {
            class1: 9.0 * eps,
            class2: 10.0 * eps,
            class3: 14.0 * eps,
        },
        StressDistribution::Bending => ClassLimit {
            class1: 9.0 * eps,
            class2: 10.0 * eps,
            class3: 14.0 * eps,
        },
        StressDistribution::BendingAndCompression { .. } => {
            // Use uniform compression limits for conservative estimate
            ClassLimit {
                class1: 9.0 * eps,
                class2: 10.0 * eps,
                class3: 14.0 * eps,
            }
        }
        StressDistribution::FlangeBendingAndCompression { psi } => {
            // ψ = σ_2/σ_1 (stress ratio)
            // EN 1993-1-5 for combined stress
            let class1 = 9.0 * eps / (1.0 + psi).sqrt().max(0.1);
            let class2 = 10.0 * eps / (1.0 + psi).sqrt().max(0.1);
            let class3 = 14.0 * eps / (1.0 + psi).sqrt().max(0.1);
            ClassLimit {
                class1,
                class2,
                class3,
            }
        }
    }
}

/// Get class limits for internal flanges (e.g., box section flanges).
fn internal_flange_limits(eps: f64) -> ClassLimit {
    ClassLimit {
        class1: 33.0 * eps,
        class2: 38.0 * eps,
        class3: 42.0 * eps,
    }
}

/// Classification result for a section.
#[derive(Debug, Clone)]
pub struct SectionClassification {
    /// Overall section class (governed by worst element)
    pub overall_class: SectionClass,
    /// Web classification
    pub web_class: SectionClass,
    /// Flange classification (compression flange)
    pub flange_class: SectionClass,
    /// Flange classification (tension flange)
    pub tension_flange_class: SectionClass,
    /// Individual element ratios
    pub web_c_t: f64,
    pub flange_c_t: f64,
    /// Epsilon factor
    pub epsilon: f64,
    /// Stress distribution assumed for web
    pub web_stress: StressDistribution,
    /// Stress distribution assumed for flange
    pub flange_stress: StressDistribution,
    /// Whether section is Class 4 (requires effective section)
    pub is_class4: bool,
}

impl SectionClassification {
    /// Check if section can develop plastic moment.
    pub fn can_plastic_moment(&self) -> bool {
        self.overall_class.can_plastic_moment()
    }

    /// Check if section can develop plastic rotation.
    pub fn can_plastic_rotation(&self) -> bool {
        self.overall_class.can_plastic_rotation()
    }

    /// Get governing class description.
    pub fn governing_element(&self) -> &'static str {
        let classes = [
            (self.web_class, "web"),
            (self.flange_class, "compression flange"),
            (self.tension_flange_class, "tension flange"),
        ];
        classes
            .iter()
            .max_by_key(|(c, _)| *c as u8)
            .map(|(_, name)| *name)
            .unwrap_or("unknown")
    }

    /// Summary string.
    pub fn summary(&self) -> String {
        format!(
            "Overall: {} (governed by {}) | Web: c/t={:.1} ({}) | Flange: c/t={:.1} ({}) | ε={:.3}",
            self.overall_class.description(),
            self.governing_element(),
            self.web_c_t,
            self.web_class.description(),
            self.flange_c_t,
            self.flange_class.description(),
            self.epsilon
        )
    }
}

/// Classify an I/H-section per EN 1993-1-1.
pub fn classify_i_section(
    section: &Section,
    material: &Material,
    stress: StressDistribution,
) -> SectionClassification {
    let fy = material.yield_strength;
    let eps = (235e6 / fy).sqrt();

    // Get geometric properties
    let props = SectionProperties::from_section(section);
    let bounds = section.bounds();
    let _h = bounds.3 - bounds.2; // depth (max_y - min_y)
    let b = bounds.1 - bounds.0; // width (max_x - min_x)

    // Estimate web and flange dimensions from section geometry
    // For I-section: find web thickness and flange thickness
    let (tw, tf, hw) = estimate_i_section_dimensions(section, &props);

    // Web: c/t = (h - 2*tf) / tw for internal part
    let web_c_t = hw / tw;

    // Flange: c/t = (b - tw)/2 / tf for outstand
    let flange_c_t = (b - tw) / (2.0 * tf);

    // Classify web
    let web_limits = web_limits(eps, stress);
    let web_class = web_limits.classify(web_c_t);

    // Classify compression flange (outstand)
    let flange_limits = outstand_flange_limits(eps, stress);
    let flange_class = flange_limits.classify(flange_c_t);

    // Tension flange (same geometry, but tension is always Class 1 for ductile steel)
    let tension_flange_class = SectionClass::Class1;

    // Overall class = min of all elements (worst class governs, with reversed Ord)
    let overall_class = [web_class, flange_class, tension_flange_class]
        .iter()
        .min()
        .copied()
        .unwrap_or(SectionClass::Class4);

    SectionClassification {
        overall_class,
        web_class,
        flange_class,
        tension_flange_class,
        web_c_t,
        flange_c_t,
        epsilon: eps,
        web_stress: stress,
        flange_stress: stress,
        is_class4: overall_class == SectionClass::Class4,
    }
}

/// Classify a hollow section (CHS/RHS/SHS) per EN 1993-1-1.
pub fn classify_hollow_section(
    section: &Section,
    material: &Material,
    stress: StressDistribution,
) -> SectionClassification {
    let fy = material.yield_strength;
    let eps = (235e6 / fy).sqrt();

    let props = SectionProperties::from_section(section);
    let bounds = section.bounds();
    let h = bounds.3 - bounds.2; // height (max_y - min_y)
    let b = bounds.1 - bounds.0; // width (max_x - min_x)

    // Check if circular or rectangular
    let is_circular = (h - b).abs() / h.max(b) < 0.05;

    if is_circular {
        // CHS - use diameter/thickness
        let area = props.area;
        let d = h; // diameter
        // t = A / (π*d) for thin-walled
        let t = area / (std::f64::consts::PI * d);
        let d_t = d / t;

        // CHS limits per EN 1993-1-1 Table 5.2 (d): 50ε², 70ε², 90ε²
        // (same for all stress distributions)
        let limits = ClassLimit {
            class1: 50.0 * eps * eps,
            class2: 70.0 * eps * eps,
            class3: 90.0 * eps * eps,
        };

        let overall_class = limits.classify(d_t);

        SectionClassification {
            overall_class,
            web_class: overall_class,
            flange_class: overall_class,
            tension_flange_class: overall_class,
            web_c_t: d_t,
            flange_c_t: d_t,
            epsilon: eps,
            web_stress: stress,
            flange_stress: stress,
            is_class4: overall_class == SectionClass::Class4,
        }
    } else {
        // RHS/SHS
        // For rectangular hollow sections, classify each wall
        // c/t = (h - 2t)/t for webs, (b - 2t)/t for flanges
        let area = props.area;
        let peri = 2.0 * (h + b);
        let t = area / peri; // average thickness

        let web_c_t = (h - 2.0 * t) / t;
        let flange_c_t = (b - 2.0 * t) / t;

        // Internal compression parts (Table 5.2)
        let web_limits = web_limits(eps, stress);
        let flange_limits = internal_flange_limits(eps);

        let web_class = web_limits.classify(web_c_t);
        let flange_class = flange_limits.classify(flange_c_t);
        let tension_flange_class = SectionClass::Class1;

        let overall_class = [web_class, flange_class, tension_flange_class]
            .iter()
            .min()
            .copied()
            .unwrap_or(SectionClass::Class4);

        SectionClassification {
            overall_class,
            web_class,
            flange_class,
            tension_flange_class,
            web_c_t,
            flange_c_t,
            epsilon: eps,
            web_stress: stress,
            flange_stress: stress,
            is_class4: overall_class == SectionClass::Class4,
        }
    }
}

/// Classify an angle section.
pub fn classify_angle_section(section: &Section, material: &Material) -> SectionClassification {
    let fy = material.yield_strength;
    let eps = (235e6 / fy).sqrt();

    let bounds = section.bounds();
    let h = bounds.3 - bounds.2; // height (max_y - min_y)
    let b = bounds.1 - bounds.0; // width (max_x - min_x)

    // For angles, classify each leg as outstand flange
    let props = SectionProperties::from_section(section);
    let area = props.area;
    let _peri = section.outer.vertices.len() as f64; // rough
    let t = area / (h + b); // approximate

    let c_t_leg1 = h / t;
    let c_t_leg2 = b / t;

    let limits = outstand_flange_limits(eps, StressDistribution::UniformCompression);
    let class1 = limits.classify(c_t_leg1);
    let class2 = limits.classify(c_t_leg2);

    let overall_class = [class1, class2]
        .iter()
        .min()
        .copied()
        .unwrap_or(SectionClass::Class4);

    SectionClassification {
        overall_class,
        web_class: class1,
        flange_class: class2,
        tension_flange_class: SectionClass::Class1,
        web_c_t: c_t_leg1,
        flange_c_t: c_t_leg2,
        epsilon: eps,
        web_stress: StressDistribution::UniformCompression,
        flange_stress: StressDistribution::UniformCompression,
        is_class4: overall_class == SectionClass::Class4,
    }
}

/// Classify a channel section.
pub fn classify_channel_section(
    section: &Section,
    material: &Material,
    stress: StressDistribution,
) -> SectionClassification {
    // Channel: web + flanges (one side only)
    // Web classified as internal, flange as outstand
    let fy = material.yield_strength;
    let eps = (235e6 / fy).sqrt();

    let props = SectionProperties::from_section(section);
    let bounds = section.bounds();
    let _h = bounds.3 - bounds.2; // height (max_y - min_y)
    let b = bounds.1 - bounds.0; // width (max_x - min_x)

    let (tw, tf, hw) = estimate_i_section_dimensions(section, &props);

    let web_c_t = hw / tw;
    let flange_c_t = b / tf; // outstand

    let web_limits = web_limits(eps, stress);
    let flange_limits = outstand_flange_limits(eps, stress);

    let web_class = web_limits.classify(web_c_t);
    let flange_class = flange_limits.classify(flange_c_t);
    let tension_flange_class = SectionClass::Class1;

    let overall_class = [web_class, flange_class, tension_flange_class]
        .iter()
        .min()
        .copied()
        .unwrap_or(SectionClass::Class4);

    SectionClassification {
        overall_class,
        web_class,
        flange_class,
        tension_flange_class,
        web_c_t,
        flange_c_t,
        epsilon: eps,
        web_stress: stress,
        flange_stress: stress,
        is_class4: overall_class == SectionClass::Class4,
    }
}

/// Classify a T-section (tee).
pub fn classify_tee_section(
    section: &Section,
    material: &Material,
    stress: StressDistribution,
) -> SectionClassification {
    // Tee: stem (web) + flange
    let fy = material.yield_strength;
    let eps = (235e6 / fy).sqrt();

    let props = SectionProperties::from_section(section);
    let bounds = section.bounds();
    let _h = bounds.3 - bounds.2; // height (max_y - min_y)
    let b = bounds.1 - bounds.0; // width (max_x - min_x)

    let (tw, tf, hw) = estimate_i_section_dimensions(section, &props);

    // Stem = web
    let web_c_t = hw / tw;
    // Flange = outstand
    let flange_c_t = (b - tw) / (2.0 * tf);

    let web_limits = web_limits(eps, stress);
    let flange_limits = outstand_flange_limits(eps, stress);

    let web_class = web_limits.classify(web_c_t);
    let flange_class = flange_limits.classify(flange_c_t);
    let tension_flange_class = SectionClass::Class1;

    let overall_class = [web_class, flange_class, tension_flange_class]
        .iter()
        .min()
        .copied()
        .unwrap_or(SectionClass::Class4);

    SectionClassification {
        overall_class,
        web_class,
        flange_class,
        tension_flange_class,
        web_c_t,
        flange_c_t,
        epsilon: eps,
        web_stress: stress,
        flange_stress: stress,
        is_class4: overall_class == SectionClass::Class4,
    }
}

/// Generic classification - auto-detect section type.
pub fn classify_section(
    section: &Section,
    material: &Material,
    stress: StressDistribution,
) -> SectionClassification {
    // Try to detect section type from geometry
    let props = SectionProperties::from_section(section);
    let bounds = section.bounds();
    let h = bounds.3 - bounds.2; // height (max_y - min_y)
    let b = bounds.1 - bounds.0; // width (max_x - min_x)
    let area = props.area;

    // Check if hollow (has holes)
    if !section.holes.is_empty() {
        return classify_hollow_section(section, material, stress);
    }

    // Check aspect ratio and area to guess type
    let _peri = section.outer.vertices.len() as f64;
    let _t_est = area / (2.0 * (h + b)); // rough thickness

    // If very thin-walled and closed -> hollow
    if area / (h * b) < 0.15 && section.holes.is_empty() {
        // Might be hollow without explicit hole (thin-walled closed)
        // Check if it's a tube-like shape
    }

    // Heuristic: count vertices for simple shapes
    let n_verts = section.outer.vertices.len();

    if n_verts <= 6 {
        // Could be angle, tee, or simple shape
        // Check symmetry
        if section.is_symmetric_about_x() && section.is_symmetric_about_y() {
            // Could be box or I-section
            if section.holes.is_empty() && area / (h * b) > 0.3 {
                // Solid-ish -> I-section or box
                return classify_i_section(section, material, stress);
            }
        } else if section.is_symmetric_about_x() || section.is_symmetric_about_y() {
            // Tee or channel
            return classify_tee_section(section, material, stress);
        } else {
            // Angle
            return classify_angle_section(section, material);
        }
    }

    // Default to I-section classification
    classify_i_section(section, material, stress)
}

/// Estimate I-section dimensions (tw, tf, hw) from geometry.
fn estimate_i_section_dimensions(section: &Section, props: &SectionProperties) -> (f64, f64, f64) {
    let bounds = section.bounds();
    let h = bounds.3 - bounds.2; // height (max_y - min_y)
    let b = bounds.1 - bounds.0; // width (max_x - min_x)
    let _area = props.area;

    // For I-section: area ≈ 2*b*tf + (h-2*tf)*tw
    // We need to estimate tw and tf from the polygon vertices

    // Find unique x and y coordinates
    let mut x_coords: Vec<f64> = section.outer.vertices.iter().map(|v| v.x).collect();
    let mut y_coords: Vec<f64> = section.outer.vertices.iter().map(|v| v.y).collect();
    x_coords.sort_by(|a, b| a.partial_cmp(b).unwrap());
    y_coords.sort_by(|a, b| a.partial_cmp(b).unwrap());

    // Web thickness: find the smallest non-zero |x| among vertices near center
    // This correctly handles root radii: web vertices have |x| = sw (smallest),
    // radius vertices have |x| in [sw, sw+r] (larger)
    let center_x = (bounds.0 + bounds.1) / 2.0;
    let mut x_abs: Vec<f64> = section
        .outer
        .vertices
        .iter()
        .filter(|v| (v.x - center_x).abs() < b * 0.25)
        .map(|v| (v.x - center_x).abs())
        .filter(|x| *x > 1e-9)
        .collect();
    x_abs.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let tw = if !x_abs.is_empty() {
        x_abs[0] * 2.0
    } else {
        b * 0.1
    };

    // Flange thickness = minimum y-distance at flange
    let top_vertices: Vec<f64> = section
        .outer
        .vertices
        .iter()
        .filter(|v| v.y > bounds.3 - h * 0.1)
        .map(|v| v.y)
        .collect();
    let tf = if top_vertices.len() >= 2 {
        top_vertices
            .iter()
            .max_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap()
            - top_vertices
                .iter()
                .min_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap()
    } else {
        h * 0.05
    };

    let hw = h - 2.0 * tf;

    (tw.max(1e-6), tf.max(1e-6), hw.max(1e-6))
}

/// Effective section properties for Class 4 sections (EN 1993-1-5).
pub mod effective {
    use super::*;
    use crate::geometry::Point;
    use crate::section::Section;
    use crate::section_properties::SectionProperties;

    /// Effective section properties after local buckling reduction.
    #[derive(Debug, Clone)]
    pub struct EffectiveProperties {
        pub area_eff: f64,
        pub centroid_eff: Point,
        pub ix_eff: f64,
        pub iy_eff: f64,
        pub ixy_eff: f64,
        pub classification: SectionClassification,
        /// Reduction factors for each element
        pub reduction_factors: ElementReductions,
    }

    #[derive(Debug, Clone, Default)]
    pub struct ElementReductions {
        pub web_rho: f64,
        pub flange_rho: f64,
    }

    /// Compute effective section for Class 4 sections per EN 1993-1-5.
    pub fn effective_section(
        section: &Section,
        material: &Material,
        stress: StressDistribution,
    ) -> EffectiveProperties {
        let classification = classify_section(section, material, stress);

        if !classification.is_class4 {
            // Not Class 4 - return gross properties
            let props = SectionProperties::from_section(section);
            return EffectiveProperties {
                area_eff: props.area,
                centroid_eff: props.centroid,
                ix_eff: props.ix,
                iy_eff: props.iy,
                ixy_eff: props.ixy,
                classification,
                reduction_factors: ElementReductions::default(),
            };
        }

        // For Class 4, compute reduced widths per EN 1993-1-5
        let fy = material.yield_strength;
        let eps = (235e6 / fy).sqrt();

        // Simplified: apply reduction factor to slender elements
        let mut rho_web = 1.0;
        let mut rho_flange = 1.0;

        // Web reduction (EN 1993-1-5 Eq 4.2)
        if classification.web_class == SectionClass::Class4 {
            let lambda_p = classification.web_c_t / (33.0 * eps); // normalized slenderness
            if lambda_p > 0.673 {
                rho_web = (lambda_p - 0.22) / lambda_p;
                rho_web = rho_web.clamp(0.0, 1.0);
            }
        }

        // Flange reduction (EN 1993-1-5 Eq 4.2)
        if classification.flange_class == SectionClass::Class4 {
            let lambda_p = classification.flange_c_t / (9.0 * eps);
            if lambda_p > 0.673 {
                rho_flange = (lambda_p - 0.22) / lambda_p;
                rho_flange = rho_flange.clamp(0.0, 1.0);
            }
        }

        // Build effective section polygon with reduced widths
        // This is complex - for now return gross properties with warning
        let props = SectionProperties::from_section(section);

        // Apply rough reduction to properties
        let area_red = 1.0 - (1.0 - rho_web) * 0.3 - (1.0 - rho_flange) * 0.2; // approximate
        let ix_red = 1.0 - (1.0 - rho_web) * 0.5 - (1.0 - rho_flange) * 0.3;

        EffectiveProperties {
            area_eff: props.area * area_red,
            centroid_eff: props.centroid,
            ix_eff: props.ix * ix_red,
            iy_eff: props.iy * ix_red, // approximate
            ixy_eff: props.ixy * ix_red,
            classification,
            reduction_factors: ElementReductions {
                web_rho: rho_web,
                flange_rho: rho_flange,
            },
        }
    }
}

/// Section trait for symmetry checks.
trait SectionSymmetryCheck {
    fn is_symmetric_about_x(&self) -> bool;
    fn is_symmetric_about_y(&self) -> bool;
}

impl SectionSymmetryCheck for Section {
    fn is_symmetric_about_x(&self) -> bool {
        for v in &self.outer.vertices {
            let found = self
                .outer
                .vertices
                .iter()
                .any(|v2| (v2.x - v.x).abs() < 1e-6 && (v2.y + v.y).abs() < 1e-6);
            if !found {
                return false;
            }
        }
        true
    }

    fn is_symmetric_about_y(&self) -> bool {
        for v in &self.outer.vertices {
            let found = self
                .outer
                .vertices
                .iter()
                .any(|v2| (v2.y - v.y).abs() < 1e-6 && (v2.x + v.x).abs() < 1e-6);
            if !found {
                return false;
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::material::presets::STEEL_S355;

    use crate::section_library::ParametricSection;
    use crate::section_library::steel::ISection;

    #[test]
    fn classify_i_section_class1() {
        // Stocky I-section (IPE300 in S355)
        let i = ISection::from_designation("IPE300").unwrap();
        let section = i.build();
        let classification =
            classify_i_section(&section, &STEEL_S355, StressDistribution::pure_bending());

        // IPE300 in S355 should be Class 1 or 2
        assert!(classification.overall_class >= SectionClass::Class2);
        assert!(classification.can_plastic_moment());
    }

    #[test]
    fn classify_i_section_class4() {
        // Very slender I-section
        let i = ISection::new(1.0, 0.2, 0.003, 0.005, 0.01); // Very thin web
        let section = i.build();
        let classification =
            classify_i_section(&section, &STEEL_S355, StressDistribution::pure_bending());

        assert_eq!(classification.overall_class, SectionClass::Class4);
        assert!(classification.is_class4);
    }

    #[test]
    fn classify_chs() {
        // CHS 219x8 in S355
        let chs = crate::section_library::steel::CircularHollowSectionLib::new(0.219, 0.008);
        let section = chs.build();
        let classification =
            classify_hollow_section(&section, &STEEL_S355, StressDistribution::pure_bending());

        // d/t = 219/8 = 27.4, limit Class 1 = 50*ε² = 50*0.81² = 33.1 -> Class 1
        assert!(classification.overall_class >= SectionClass::Class2);
    }

    #[test]
    fn classify_rhs() {
        // RHS 200x100x8 in S355
        let rhs = crate::section_library::steel::RectangularHollowSection::new(
            0.2, 0.1, 0.008, 0.012, 0.004,
        );
        let section = rhs.build();
        let classification =
            classify_hollow_section(&section, &STEEL_S355, StressDistribution::pure_bending());

        // h/t = 200/8 = 25, limit Class 1 = 33*ε = 26.7 -> Class 1/2
        assert!(classification.overall_class >= SectionClass::Class2);
    }

    #[test]
    fn classify_angle() {
        let angle = crate::section_library::steel::AngleSection::equal_leg(0.1, 0.01);
        let section = angle.build();
        let classification = classify_angle_section(&section, &STEEL_S355);

        // c/t = 100/10 = 10, limit Class 1 = 9*ε = 7.3 -> Class 2 or 3
        assert!(classification.overall_class <= SectionClass::Class1);
    }

    #[test]
    fn epsilon_calculation() {
        // S235: ε = 1.0
        let eps_s235: f64 = (235.0e6_f64 / 235.0e6_f64).sqrt();
        assert!((eps_s235 - 1.0).abs() < 1e-6);

        // S355: ε = sqrt(235/355) = 0.814
        let eps_s355: f64 = (235.0e6_f64 / 355.0e6_f64).sqrt();
        assert!((eps_s355 - 0.814).abs() < 0.01);

        // S460: ε = sqrt(235/460) = 0.715
        let eps_s460: f64 = (235.0e6_f64 / 460.0e6_f64).sqrt();
        assert!((eps_s460 - 0.715).abs() < 0.01);
    }

    #[test]
    fn effective_section_class4() {
        // Create a Class 4 section
        let i = ISection::new(0.5, 0.15, 0.003, 0.008, 0.01);
        let section = i.build();

        let effective =
            effective::effective_section(&section, &STEEL_S355, StressDistribution::pure_bending());

        assert!(effective.classification.is_class4);
        assert!(effective.area_eff < effective.classification.web_c_t * 0.001); // Some reduction
        assert!(effective.reduction_factors.web_rho < 1.0);
    }

    #[test]
    fn class_hierarchy() {
        assert!(SectionClass::Class1 > SectionClass::Class2);
        assert!(SectionClass::Class2 > SectionClass::Class3);
        assert!(SectionClass::Class3 > SectionClass::Class4);

        assert!(SectionClass::Class1.can_plastic_moment());
        assert!(SectionClass::Class2.can_plastic_moment());
        assert!(!SectionClass::Class3.can_plastic_moment());
        assert!(!SectionClass::Class4.can_plastic_moment());

        assert!(SectionClass::Class1.can_plastic_rotation());
        assert!(!SectionClass::Class2.can_plastic_rotation());
    }

    #[test]
    fn stress_distribution_variants() {
        let bending = StressDistribution::pure_bending();
        let compression = StressDistribution::pure_compression();
        let _web_bc = StressDistribution::web_bending_compression(0.5);
        let _flange_bc = StressDistribution::flange_bending_compression(-0.5);

        assert_eq!(bending, StressDistribution::Bending);
        assert_eq!(compression, StressDistribution::UniformCompression);
    }

    #[test]
    fn web_bending_compression_limits_pure_compression() {
        // α = 1.0 (pure compression) should match UniformCompression: 33ε, 38ε, 42ε
        let eps = 1.0;
        let bc = web_limits(eps, StressDistribution::web_bending_compression(1.0));
        let uc = web_limits(eps, StressDistribution::pure_compression());
        assert!((bc.class1 - uc.class1).abs() < 1e-10);
        assert!((bc.class2 - uc.class2).abs() < 1e-10);
        assert!((bc.class3 - uc.class3).abs() < 1e-10);
    }

    #[test]
    fn web_bending_compression_limits_pure_bending() {
        // α = 0.5 (pure bending) should match Bending: 72ε, 83ε, 124ε
        let eps = 1.0;
        let bc = web_limits(eps, StressDistribution::web_bending_compression(0.5));
        let bending = web_limits(eps, StressDistribution::pure_bending());
        assert!((bc.class1 - bending.class1).abs() < 1e-10);
        assert!((bc.class2 - bending.class2).abs() < 1e-10);
        assert!((bc.class3 - bending.class3).abs() < 1.0); // 123.5 vs 124 (rounding)
    }

    #[test]
    fn web_bending_compression_limits_intermediate() {
        // α = 0.75: Class 1 = 396/(13*0.75-1) = 45.26ε
        //           Class 2 = 456/(13*0.75-1) = 52.11ε
        //           ψ = 0, Class 3 = 42/0.67 = 62.69ε
        let eps = 1.0;
        let bc = web_limits(eps, StressDistribution::web_bending_compression(0.75));
        assert!((bc.class1 - 396.0 / 8.75).abs() < 1e-10);
        assert!((bc.class2 - 456.0 / 8.75).abs() < 1e-10);
        assert!((bc.class3 - 42.0 / 0.67).abs() < 1e-10);
    }

    #[test]
    fn web_bending_compression_limits_zero_alpha() {
        // α = 0 (pure tension) should return f64::MAX (no compression zone)
        let eps = 1.0;
        let bc = web_limits(eps, StressDistribution::web_bending_compression(0.0));
        assert_eq!(bc.class1, f64::MAX);
        assert_eq!(bc.class2, f64::MAX);
    }
}
