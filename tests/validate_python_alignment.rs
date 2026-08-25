//! Numerical validation against analytical solutions used by the Python
//! library's own test suite. Each failing assertion indicates a functional
//! discrepancy to investigate.

use section_properties::plastic::{PlasticAnalysis, PlasticAxis};
use section_properties::section_library::ParametricSection;
use section_properties::section_library::primitive::{
    CircularHollowSection, CircularSection, RectangularSection,
};
use section_properties::section_library::steel::ISection;
use section_properties::stress::{SectionLoads, StressAnalysis};
use section_properties::material::presets::STEEL_S355;
use std::f64::consts::PI;

fn rel(a: f64, b: f64) -> f64 {
    ((a - b) / b.abs()).abs()
}

#[test]
fn report_i_section_warping_constant() {
    let ipe = ISection::from_designation("IPE300").unwrap();
    let sec = ipe.build();
    let fp = sec.frame_properties_full();

    // Analytical Iw for doubly-symmetric I: tf*bf^3/24 * hw^2
    let dims = ipe_dims();
    let (d, bf, tw, tf, _r) = dims;
    let hw = d - tf;
    let iw_analytic = tf * bf.powi(3) / 24.0 * hw.powi(2);
    println!("IPE300 (m): d={d} bf={bf} tw={tw} tf={tf}");
    println!("Iw computed = {:.6e}, analytic = {:.6e}, rel err = {:.3e}",
        fp.iw, iw_analytic, rel(fp.iw, iw_analytic));
    assert!(rel(fp.iw, iw_analytic) < 0.05, "Iw mismatch");
}

fn ipe_dims() -> (f64, f64, f64, f64, f64) {
    // IPE300 in metres
    (0.300, 0.150, 0.0071, 0.0107, 0.015)
}

#[test]
fn report_chs_torsion_constant_bredt() {
    let chs = CircularHollowSection::from_dimensions(0.2191, 0.0082);
    let sec = chs.build();
    let props = sec.frame_properties_full();

    // Bredt: J = 4*Am^2*t/s (thin-walled closed tube)
    let ro = 0.10955;
    let ri = ro - 0.0082;
    let rm = (ro + ri) / 2.0;
    let am = PI * rm * rm;
    let s = 2.0 * PI * rm;
    let j_bredt = 4.0 * am * am * 0.0082 / s;

    println!("CHS J computed = {:.6e}, Bredt = {:.6e}, rel err = {:.3e}",
        props.j, j_bredt, rel(props.j, j_bredt));
    assert!(rel(props.j, j_bredt) < 0.05, "J mismatch");
}

#[test]
fn report_rectangle_plastic_modulus() {
    let rect = RectangularSection::new(0.1, 0.2);
    let sec = rect.build();
    let mat = STEEL_S355;

    let pa = PlasticAnalysis::new(sec.clone(), mat);
    let pp = pa.plastic_section.plastic_properties(PlasticAxis::X);
    // Zpl,x = b*h^2/4
    let z_expected = 0.1 * 0.2_f64.powi(4).sqrt() * 0.2; // placeholder guard
    let z_true = 0.1 * 0.04 / 4.0; // b h^2/4 = 0.0002 m^3
    println!("Zpl computed = {:.6e}, expected = {:.6e}", pp.plastic_section_modulus, z_true);
    let _ = z_expected;
    assert!(rel(pp.plastic_section_modulus, z_true) < 0.01);
}

#[test]
fn report_stress_superposition() {
    // Rectangle under N + Mx: extreme fibre stress must equal N/A ± M/W.
    let rect = RectangularSection::new(0.1, 0.2);
    let sec = rect.build();
    let analysis = StressAnalysis::new(sec.clone(), STEEL_S355);

    let n = 100.0e3;
    let mx = 10.0e3; // N·m
    let result = analysis.calculate_stress(SectionLoads { n, vx: 0.0, vy: 0.0, mxx: mx, myy: 0.0, mzz: 0.0, m11: 0.0, m22: 0.0 });

    let area = 0.02;
    let zx = 0.1 * 0.2_f64.powi(2) / 6.0; // elastic modulus
    let s_max_top = n / area + mx / zx;
    let s_min_bot = n / area - mx / zx;

    println!("max sigma_zz = {:.1}, expected top = {:.1}", result.max_sigma_z, s_max_top);
    println!("min sigma_zz = {:.1}, expected bot = {:.1}", result.min_sigma_z, s_min_bot);
    assert!(rel(result.max_sigma_z, s_max_top) < 0.01, "max stress mismatch");
    assert!(rel(result.min_sigma_z.abs(), s_min_bot.abs()) < 0.01, "min stress mismatch");
}

#[test]
fn report_circular_section_moments() {
    let circ = CircularSection::new(0.1);
    let props = section_properties::SectionProperties::from_section(&circ.build());
    let ix_exact = PI * 0.1_f64.powi(4) / 4.0;
    println!("Ix = {:.6e}, exact = {:.6e}, rel err = {:.3e}", props.ix, ix_exact, rel(props.ix, ix_exact));
    assert!(rel(props.ix, ix_exact) < 0.01); // 64-gon approximation error (same in Python)
}


#[test]
fn report_solid_circle_j() {
    let circ = CircularSection::new(0.1);
    let sec = circ.build();
    let fp = sec.frame_properties_full();
    let ip = PI * 0.1_f64.powi(4) / 2.0;
    println!("solid circle J = {:.6e}, Ip exact = {:.6e}, rel err = {:.3e}", fp.j, ip, rel(fp.j, ip));
}

#[test]
fn debug_square_shear() {
    let poly = section_properties::Polygon::new(vec![
        section_properties::Point::new(-0.05, -0.05),
        section_properties::Point::new(0.05, -0.05),
        section_properties::Point::new(0.05, 0.05),
        section_properties::Point::new(-0.05, 0.05),
    ]);
    let sec = section_properties::Section::new(poly, vec![]);
    let wp = section_properties::plastic::WarpingProperties::from_section(&sec);
    println!("area={} ay={} az={} finite=({},{}), area*100={}", wp.area, wp.ay, wp.az, wp.ay.is_finite(), wp.az.is_finite(), wp.area * 100.0);
    println!("j={} iw={}", wp.j, wp.iw);

    let analysis = section_properties::stress::StressAnalysis::new(sec.clone(), STEEL_S355);
    let loads = section_properties::stress::SectionLoads { vx: 5e3, vy: 10e3, ..section_properties::stress::SectionLoads::zero() };
    let r = analysis.calculate_stress(loads);
    println!("n_points={} max_zx_vx={}", r.point_stresses.len(), r.point_stresses.iter().map(|s| s.sig_zx_vx.abs()).fold(0.0f64, f64::max));
}
