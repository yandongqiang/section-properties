//! Effective width method for cold-formed steel elements.
//!
//! Per EN 1993-1-3 (Eurocode 3 Part 1-3), AS/NZS 4600, AISI S100.

use crate::cold_formed_analysis::{BucklingCurve, EffectiveWidthParams, ElementReduction};

/// Compute effective width for a flat element.
///
/// # Arguments
/// * `width` - Flat width (clear distance between supports)
/// * `thickness` - Element thickness
/// * `fy` - Yield strength
/// * `e` - Young's modulus
/// * `stress` - Actual compressive stress in element
/// * `edge_support` - Edge support condition
/// * `params` - Calculation parameters
///
/// # Returns
/// Effective width and reduction factor
pub fn effective_width_flat(
    width: f64,
    thickness: f64,
    fy: f64,
    e: f64,
    stress: f64,
    edge_support: crate::cold_formed_analysis::EdgeSupport,
    params: &EffectiveWidthParams,
) -> (f64, f64) {
    if width <= 0.0 || thickness <= 0.0 || fy <= 0.0 {
        return (0.0, 0.0);
    }

// Normalized slenderness per EN 1993-1-3 Section 4.4(2):
    // λ̄ = (b/t) * sqrt(12(1-ν²) * fy / (k * E * π²))
    let nu_clamped = params.nu.clamp(0.0, 0.5); // keep (1-ν²) positive for metals
    let k = buckling_coefficient(edge_support);
    let lambda_bar = (width / thickness)
        * (12.0 * (1.0 - nu_clamped * nu_clamped) * fy / (k * e * std::f64::consts::PI.powi(2)))
            .sqrt();

    let rho = effective_width_reduction(lambda_bar, stress, fy);

    let b_eff = rho * width;
    (b_eff, rho)
}

/// Buckling coefficient k per EN 1993-1-3 Table 4.1 / AISI S100 Table B3.1.
fn buckling_coefficient(edge_support: crate::cold_formed_analysis::EdgeSupport) -> f64 {
    match edge_support {
        crate::cold_formed_analysis::EdgeSupport::DoubleSupported => 4.0, // Internal
        crate::cold_formed_analysis::EdgeSupport::Outstand => 0.43,       // Outstand
        crate::cold_formed_analysis::EdgeSupport::Stiffened => 4.0,       // Stiffened (conservative)
    }
}

/// Effective width reduction factor
/// per EN 1993-1-3 Section 4.4 / AISI S100 Section B3.
fn effective_width_reduction(lambda_bar: f64, stress: f64, fy: f64) -> f64 {
    const LAMBDA_BAR_LIMIT: f64 = 0.673;
    if lambda_bar <= LAMBDA_BAR_LIMIT {
        1.0
    } else {
        let psi = stress / fy; // Stress ratio
        let rho = (1.0 - 0.22 / lambda_bar) / lambda_bar;
        // For intermediate stress levels, use interpolation
        if psi < 1.0 {
            let rho_psi = 1.0 - (1.0 - rho) * psi;
            rho_psi.max(rho).min(1.0)
        } else {
            rho.min(1.0)
        }
    }
}

/// Compute effective width for a corner element.
pub fn effective_width_corner(
    width: f64,
    thickness: f64,
    fy: f64,
    e: f64,
    stress: f64,
    corner_radius: f64,
    params: &EffectiveWidthParams,
) -> (f64, f64) {
    // Corner elements have increased capacity due to cold work
    // Per EN 1993-1-3 Section 3.2.2

    if corner_radius > 0.0 && corner_radius <= 5.0 * thickness {
        // Effective corner properties
        let r_t = corner_radius / thickness;
        let k_c = 1.0 + 0.1 * (r_t - 1.0).min(4.0); // Corner enhancement factor
        let fy_corner = fy * k_c.min(1.5); // Limit per code

        let (b_eff, rho) = effective_width_flat(
            width,
            thickness,
            fy_corner,
            e,
            stress,
            crate::cold_formed_analysis::EdgeSupport::DoubleSupported,
            params,
        );
        (b_eff, rho)
    } else {
        effective_width_flat(
            width,
            thickness,
            fy,
            e,
            stress,
            crate::cold_formed_analysis::EdgeSupport::DoubleSupported,
            params,
        )
    }
}

/// Compute effective width for stiffened elements.
pub fn effective_width_stiffened(
    width: f64,
    thickness: f64,
    fy: f64,
    e: f64,
    stress: f64,
    stiffener: &crate::cold_formed_analysis::Stiffener,
    params: &EffectiveWidthParams,
) -> (f64, f64) {
    // Stiffened element effective width per EN 1993-1-3 Section 4.5
    // Requires checking stiffener rigidity

    let i_s = stiffener_moment_of_inertia(stiffener);
    let i_s_min = minimum_stiffener_inertia(width, thickness, fy, e, params);

    if i_s >= i_s_min {
        // Stiffener is fully effective - treat as two sub-elements
        let (b_eff1, rho1) = effective_width_flat(
            width / 2.0,
            thickness,
            fy,
            e,
            stress,
            crate::cold_formed_analysis::EdgeSupport::DoubleSupported,
            params,
        );
        let (b_eff2, rho2) = effective_width_flat(
            width / 2.0,
            thickness,
            fy,
            e,
            stress,
            crate::cold_formed_analysis::EdgeSupport::DoubleSupported,
            params,
        );
        (b_eff1 + b_eff2, (rho1 + rho2) / 2.0)
    } else {
        // Stiffener not fully effective - reduce stiffener contribution
        let reduction = (i_s / i_s_min).sqrt().min(1.0);
        let (b_eff, rho) = effective_width_flat(
            width,
            thickness,
            fy,
            e,
            stress,
            crate::cold_formed_analysis::EdgeSupport::DoubleSupported,
            params,
        );
        (b_eff, rho * reduction)
    }
}

/// Stiffener moment of inertia about its own centroid.
fn stiffener_moment_of_inertia(stiffener: &crate::cold_formed_analysis::Stiffener) -> f64 {
    match stiffener.type_ {
        crate::cold_formed_analysis::StiffenerType::Intermediate => {
            // Simple lip or intermediate stiffener
            stiffener.width * stiffener.thickness.powi(3) / 12.0
        }
        crate::cold_formed_analysis::StiffenerType::EdgeWithLip => {
            // Edge stiffener with lip - combined inertia
            let lip_len = stiffener.lip_length.unwrap_or(0.0);
            let a_lip = lip_len * stiffener.thickness;
            let a_stiff = stiffener.width * stiffener.thickness;
            let total_a = a_lip + a_stiff;
            let y_bar = (a_lip * (stiffener.width + lip_len / 2.0)
                + a_stiff * stiffener.width / 2.0)
                / total_a;
            let i_lip = a_lip * (stiffener.width + lip_len / 2.0 - y_bar).powi(2)
                + lip_len * stiffener.thickness.powi(3) / 12.0;
            let i_stiff = a_stiff * (stiffener.width / 2.0 - y_bar).powi(2)
                + stiffener.width * stiffener.thickness.powi(3) / 12.0;
            i_lip + i_stiff
        }
        crate::cold_formed_analysis::StiffenerType::EdgeWithoutLip => {
            stiffener.width * stiffener.thickness.powi(3) / 12.0
        }
    }
}

/// Minimum required stiffener inertia per EN 1993-1-3 Eq 4.13.
fn minimum_stiffener_inertia(
    width: f64,
    thickness: f64,
    fy: f64,
    e: f64,
    params: &EffectiveWidthParams,
) -> f64 {
    let k_sigma =
        buckling_coefficient(crate::cold_formed_analysis::EdgeSupport::DoubleSupported);
    let sigma_cr = k_sigma * e * std::f64::consts::PI.powi(2)
        / (12.0 * (1.0 - params.nu * params.nu))
        * (thickness / width).powi(2);

    // Simplified minimum inertia requirement
    0.5 * thickness.powi(4) * (width / thickness).sqrt() * (fy / sigma_cr).sqrt()
}

/// Compute reduced section properties for a cold-formed section.
pub fn reduced_section_properties(
    cfs: &crate::cold_formed_analysis::ColdFormedSection,
    f_c: f64,
) -> crate::cold_formed_analysis::EffectiveSectionProperties {
    let params = EffectiveWidthParams::default();
    let mut element_reductions = Vec::new();
    let mut total_area = 0.0;
    let _first_moment_x = 0.0;
    let _first_moment_y = 0.0;
    let _ix = 0.0;
    let _iy = 0.0;
    let _ixy = 0.0;

    // This is a simplified implementation
    // Full implementation would rebuild the section polygon with reduced widths
    let props = crate::section_properties::SectionProperties::from_section(&cfs.section);

    // Apply reductions to each element
    for (i, element) in cfs.elements.iter().enumerate() {
        let stress = f_c.min(element.yield_strength);

        let (b_eff, rho) = match element.element_type {
            crate::cold_formed_analysis::ElementType::Flat => effective_width_flat(
                element.width,
                element.thickness,
                element.yield_strength,
                params.e,
                stress,
                element.edge_support,
                &params,
            ),
            crate::cold_formed_analysis::ElementType::Corner => effective_width_corner(
                element.width,
                element.thickness,
                element.yield_strength,
                params.e,
                stress,
                element.corner_radius,
                &params,
            ),
            crate::cold_formed_analysis::ElementType::Stiffened => {
                if let Some(stiffener) = &element.stiffener {
                    effective_width_stiffened(
                        element.width,
                        element.thickness,
                        element.yield_strength,
                        params.e,
                        stress,
                        stiffener,
                        &params,
                    )
                } else {
                    (element.width, 1.0)
                }
            }
            _ => (element.width, 1.0),
        };

        element_reductions.push(ElementReduction {
            element_index: i,
            original_width: element.width,
            effective_width: b_eff,
            reduction_factor: rho,
            buckling_curve: match element.edge_support {
                crate::cold_formed_analysis::EdgeSupport::DoubleSupported => BucklingCurve::Internal,
                crate::cold_formed_analysis::EdgeSupport::Outstand => BucklingCurve::Outstand,
                crate::cold_formed_analysis::EdgeSupport::Stiffened => BucklingCurve::Stiffened,
            },
        });

        total_area += b_eff * element.thickness;
    }

    // Simplified: scale properties by area ratio
    let area_ratio = if props.area > 0.0 {
        total_area / props.area
    } else {
        1.0
    };

    crate::cold_formed_analysis::EffectiveSectionProperties {
        area_eff: total_area,
        centroid_eff: props.centroid,
        ix_eff: props.ix * area_ratio,
        iy_eff: props.iy * area_ratio,
        ixy_eff: props.ixy * area_ratio,
        element_reductions,
    }
}

/// Convenience function with default params.
pub fn effective_width_flat_simple(
    width: f64,
    thickness: f64,
    fy: f64,
    stress: f64,
    edge_support: crate::cold_formed_analysis::EdgeSupport,
) -> (f64, f64) {
    let params = EffectiveWidthParams {
        fy,
        e: 200e9,
        nu: 0.3,
        gamma_m0: 1.0,
        use_eurocode: true,
    };
    effective_width_flat(
        width,
        thickness,
        fy,
        params.e,
        stress,
        edge_support,
        &params,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cold_formed_analysis::EdgeSupport;

    #[test]
    fn effective_width_stocky() {
        // Stocky element (Class 1) - no reduction
        // b/t = 25, λ̄ ≈ 0.55 < 0.673 -> rho = 1.0
        let (b_eff, rho) =
            effective_width_flat_simple(50.0, 2.0, 350e6, 200e6, EdgeSupport::DoubleSupported);
        assert!((rho - 1.0).abs() < 1e-6);
        assert!((b_eff - 50.0).abs() < 1e-6);
    }

    #[test]
    fn effective_width_slender() {
        // Slender element - reduction expected
        let (b_eff, rho) =
            effective_width_flat_simple(500.0, 1.0, 350e6, 300e6, EdgeSupport::DoubleSupported);
        assert!(rho < 1.0);
        assert!(b_eff < 500.0);
    }

    #[test]
    fn effective_width_outstand() {
        // Outstand element
        let (b_eff, rho) =
            effective_width_flat_simple(100.0, 2.0, 350e6, 300e6, EdgeSupport::Outstand);
        assert!(rho <= 1.0);
    }

    #[test]
    fn effective_width_corner_enhancement() {
        // Corner with small radius gets enhancement
        let params = EffectiveWidthParams::default();
        let (b_eff1, rho1) = effective_width_flat(
            100.0,
            2.0,
            350e6,
            200e9,
            300e6,
            EdgeSupport::DoubleSupported,
            &params,
        );
        let (b_eff2, rho2) =
            effective_width_corner(100.0, 2.0, 350e6, 200e9, 300e6, 3.0 * 2.0, &params); // r = 3t
        assert!(rho2 >= rho1); // Corner should be better or equal
    }

    #[test]
    fn buckling_coefficients() {
        assert_eq!(
            buckling_coefficient(EdgeSupport::DoubleSupported),
            4.0
        );
        assert_eq!(buckling_coefficient(EdgeSupport::Outstand), 0.43);
    }
}

