[1mdiff --git a/src/lib.rs b/src/lib.rs[m
[1mindex 70498ed..eef16de 100644[m
[1m--- a/src/lib.rs[m
[1m+++ b/src/lib.rs[m
[36m@@ -5,4 +5,4 @@[m [mpub mod section_properties;[m
 pub use crate::geometry::Point;[m
 pub use crate::geometry::Polygon;[m
 pub use crate::section::Section;[m
[31m-pub use crate::section_properties::SectionProperties;[m
[32m+[m[32mpub use crate::section_properties::{GyrationProperties, PrincipalProperties, SectionProperties};[m
[1mdiff --git a/src/section_properties.rs b/src/section_properties.rs[m
[1mindex 1575397..44bc194 100644[m
[1m--- a/src/section_properties.rs[m
[1m+++ b/src/section_properties.rs[m
[36m@@ -1,12 +1,5 @@[m
 use crate::geometry::Point;[m
 use crate::section::Section;[m
[31m-[m
[31m-#[derive(Debug, Clone, Copy)][m
[31m-pub struct GyrationProperties {[m
[31m-    pub rx: f64,[m
[31m-    pub ry: f64,[m
[31m-    pub polar: f64,[m
[31m-}[m
 #[derive(Debug, Clone, Copy)][m
 pub struct PrincipalProperties {[m
     /// Major principal moment of inertia.[m
[36m@@ -18,6 +11,19 @@[m [mpub struct PrincipalProperties {[m
     /// Principal axis angle in radians, measured CCW from the x-axis.[m
     pub angle: f64,[m
 }[m
[32m+[m
[32m+[m[32m#[derive(Debug, Clone, Copy)][m
[32m+[m[32mpub struct GyrationProperties {[m
[32m+[m[32m    /// Radius of gyration about the centroidal x-axis.[m
[32m+[m[32m    pub rx: f64,[m
[32m+[m
[32m+[m[32m    /// Radius of gyration about the centroidal y-axis.[m
[32m+[m[32m    pub ry: f64,[m
[32m+[m
[32m+[m[32m    /// Polar radius of gyration.[m
[32m+[m[32m    pub polar: f64,[m
[32m+[m[32m}[m
[32m+[m
 /// Mechanical properties of a section (area, centroid, moments of inertia, etc.).[m
 #[derive(Debug, Clone, Copy)][m
 pub struct SectionProperties {[m
[1mdiff --git a/tests/section_properties_test.rs b/tests/section_properties_test.rs[m
[1mindex 920d0a0..582dbaf 100644[m
[1m--- a/tests/section_properties_test.rs[m
[1m+++ b/tests/section_properties_test.rs[m
[36m@@ -120,3 +120,58 @@[m [mfn principal_moments_and_gyration() {[m
     assert!((ry - ry_exp).abs() < 1e-10);[m
     assert!((rho - rho_exp).abs() < 1e-10);[m
 }[m
[32m+[m[32m#[test][m
[32m+[m[32mfn principal_properties_invariants() {[m
[32m+[m[32m    let outer = Polygon::new(vec![[m
[32m+[m[32m        Point::new(0.0, 0.0),[m
[32m+[m[32m        Point::new(10.0, 0.0),[m
[32m+[m[32m        Point::new(8.0, 6.0),[m
[32m+[m[32m        Point::new(2.0, 5.0),[m
[32m+[m[32m    ]);[m
[32m+[m
[32m+[m[32m    let section = Section::new(outer, Vec::new());[m
[32m+[m[32m    let props = SectionProperties::from_section(&section);[m
[32m+[m
[32m+[m[32m    let principal = props.principal_properties();[m
[32m+[m
[32m+[m[32m    // Invariant 1:[m
[32m+[m[32m    // I1 + I2 = Ix + Iy[m
[32m+[m[32m    assert!((principal.i1 + principal.i2 - props.ix - props.iy).abs() < 1e-10);[m
[32m+[m
[32m+[m[32m    // Invariant 2:[m
[32m+[m[32m    // I1 * I2 = Ix * Iy - Ixy²[m
[32m+[m[32m    assert!([m
[32m+[m[32m        (principal.i1 * principal.i2 - (props.ix * props.iy - props.ixy.powi(2))).abs() < 1e-10[m
[32m+[m[32m    );[m
[32m+[m
[32m+[m[32m    // Principal moments are ordered.[m
[32m+[m[32m    assert!(principal.i1 >= principal.i2);[m
[32m+[m
[32m+[m[32m    // Principal moments must be non-negative for a valid area.[m
[32m+[m[32m    assert!(principal.i1 >= 0.0);[m
[32m+[m[32m    assert!(principal.i2 >= 0.0);[m
[32m+[m[32m}[m
[32m+[m[32m#[test][m
[32m+[m[32mfn gyration_properties() {[m
[32m+[m[32m    let outer = Polygon::new(vec![[m
[32m+[m[32m        Point::new(0.0, 0.0),[m
[32m+[m[32m        Point::new(10.0, 0.0),[m
[32m+[m[32m        Point::new(10.0, 5.0),[m
[32m+[m[32m        Point::new(0.0, 5.0),[m
[32m+[m[32m    ]);[m
[32m+[m
[32m+[m[32m    let section = Section::new(outer, Vec::new());[m
[32m+[m[32m    let props = SectionProperties::from_section(&section);[m
[32m+[m
[32m+[m[32m    let gyration = props.gyration_properties();[m
[32m+[m
[32m+[m[32m    assert!((gyration.rx - (props.ix / props.area).sqrt()).abs() < 1e-10);[m
[32m+[m
[32m+[m[32m    assert!((gyration.ry - (props.iy / props.area).sqrt()).abs() < 1e-10);[m
[32m+[m
[32m+[m[32m    assert!((gyration.polar - ((props.ix + props.iy) / props.area).sqrt()).abs() < 1e-10);[m
[32m+[m
[32m+[m[32m    // Polar radius identity:[m
[32m+[m[32m    // rp² = rx² + ry²[m
[32m+[m[32m    assert!((gyration.polar.powi(2) - gyration.rx.powi(2) - gyration.ry.powi(2)).abs() < 1e-10);[m
[32m+[m[32m}[m
