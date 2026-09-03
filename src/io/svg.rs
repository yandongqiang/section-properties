//! SVG export for section visualization.
//!
//! Generates scalable vector graphics for section geometry,
//! properties, and stress visualization.

use crate::geometry::{Point, Polygon};
use crate::mesh::StressResult;
use crate::section::Section;
use crate::section_properties::SectionProperties;

/// SVG export options.
#[derive(Debug, Clone)]
pub struct SvgExportOptions {
    /// Canvas width in pixels
    pub width: u32,
    /// Canvas height in pixels
    pub height: u32,
    /// Margin around section (pixels)
    pub margin: u32,
    /// Stroke width for outer boundary
    pub outer_stroke_width: f64,
    /// Stroke width for holes
    pub hole_stroke_width: f64,
    /// Fill color for section
    pub fill_color: String,
    /// Fill opacity (0-1)
    pub fill_opacity: f64,
    /// Stroke color for outer boundary
    pub outer_stroke_color: String,
    /// Stroke color for holes
    pub hole_stroke_color: String,
    /// Show centroid marker
    pub show_centroid: bool,
    /// Show principal axes
    pub show_principal_axes: bool,
    /// Show coordinate axes
    pub show_coordinate_axes: bool,
    /// Show dimension labels
    pub show_dimensions: bool,
    /// Font size for labels
    pub font_size: u32,
    /// Background color
    pub background_color: String,
    /// Title
    pub title: Option<String>,
    /// Scale factor (pixels per unit)
    pub scale: Option<f64>,
}

impl Default for SvgExportOptions {
    fn default() -> Self {
        Self {
            width: 800,
            height: 600,
            margin: 50,
            outer_stroke_width: 2.0,
            hole_stroke_width: 1.5,
            fill_color: "#e0e8f0".to_string(),
            fill_opacity: 0.7,
            outer_stroke_color: "#2c3e50".to_string(),
            hole_stroke_color: "#c0392b".to_string(),
            show_centroid: true,
            show_principal_axes: true,
            show_coordinate_axes: true,
            show_dimensions: false,
            font_size: 12,
            background_color: "#ffffff".to_string(),
            title: None,
            scale: None,
        }
    }
}

/// Export section to SVG string.
pub fn to_svg(section: &Section, options: SvgExportOptions) -> String {
    let props = SectionProperties::from_section(section);
    let bounds = section.bounds();

    // Calculate scale to fit section in canvas
    let section_width = bounds.1 - bounds.0;
    let section_height = bounds.3 - bounds.2;
    let max_dim = section_width.max(section_height);

    let scale = options.scale.unwrap_or_else(|| {
        let available_width = options.width as f64 - 2.0 * options.margin as f64;
        let available_height = options.height as f64 - 2.0 * options.margin as f64;
        (available_width / max_dim).min(available_height / max_dim)
    });

    // Center offset
    let offset_x = options.width as f64 / 2.0 - (bounds.0 + bounds.1) / 2.0 * scale;
    let offset_y = options.height as f64 / 2.0 + (bounds.2 + bounds.3) / 2.0 * scale; // Y flipped in SVG

    let mut svg = String::new();

    // SVG header
    svg.push_str(&format!(
        r#"<svg width="{}" height="{}" viewBox="0 0 {} {}" xmlns="http://www.w3.org/2000/svg">"#,
        options.width, options.height, options.width, options.height
    ));
    svg.push('\n');

    // Background
    svg.push_str(&format!(
        r#"<rect width="100%" height="100%" fill="{}"/>"#,
        options.background_color
    ));
    svg.push('\n');

    // Title
    if let Some(title) = &options.title {
        svg.push_str(&format!(
            r##"<text x="{}" y="{}" font-size="{}" text-anchor="middle" fill="#333">{}</text>"##,
            options.width / 2,
            options.margin / 2,
            options.font_size + 4,
            title
        ));
        svg.push('\n');
    }

    // Coordinate axes
    if options.show_coordinate_axes {
        svg.push_str(&coordinate_axes(
            &props, offset_x, offset_y, scale, &options,
        ));
    }

    // Section geometry
    svg.push_str(&section_polygons(
        section, offset_x, offset_y, scale, &options,
    ));

    // Centroid
    if options.show_centroid {
        svg.push_str(&centroid_marker(
            &props.centroid,
            offset_x,
            offset_y,
            scale,
            &options,
        ));
    }

    // Principal axes
    if options.show_principal_axes {
        svg.push_str(&principal_axes_svg(
            &props, offset_x, offset_y, scale, &options,
        ));
    }

    // Dimensions
    if options.show_dimensions {
        svg.push_str(&dimension_lines(
            section, offset_x, offset_y, scale, &options,
        ));
    }

    // Legend
    svg.push_str(&legend(&options));

    svg.push_str("</svg>\n");
    svg
}

fn section_polygons(
    section: &Section,
    ox: f64,
    oy: f64,
    scale: f64,
    options: &SvgExportOptions,
) -> String {
    let mut svg = String::new();

    // Outer boundary
    svg.push_str(&polygon_to_path(&section.outer, ox, oy, scale));
    svg.push_str(&format!(
        r#" fill="{}" fill-opacity="{}" stroke="{}" stroke-width="{}" stroke-linejoin="round" />"#,
        options.fill_color,
        options.fill_opacity,
        options.outer_stroke_color,
        options.outer_stroke_width
    ));
    svg.push('\n');

    // Holes (using fill-rule="evenodd" or separate paths with no fill)
    for hole in &section.holes {
        svg.push_str(&polygon_to_path(hole, ox, oy, scale));
        svg.push_str(&format!(
            r#" fill="none" stroke="{}" stroke-width="{}" stroke-dasharray="5,5" />"#,
            options.hole_stroke_color, options.hole_stroke_width
        ));
        svg.push('\n');
    }

    svg
}

fn polygon_to_path(poly: &Polygon, ox: f64, oy: f64, scale: f64) -> String {
    let mut path = String::new();
    path.push_str(r#"<path d=""#);

    let vertices = &poly.vertices;
    if vertices.is_empty() {
        return path;
    }

    // Move to first vertex
    let first = vertices[0];
    path.push_str(&format!(
        "M {:.2} {:.2} ",
        ox + first.x * scale,
        oy - first.y * scale
    ));

    // Line to each subsequent vertex
    for v in &vertices[1..] {
        path.push_str(&format!(
            "L {:.2} {:.2} ",
            ox + v.x * scale,
            oy - v.y * scale
        ));
    }

    // Close path
    path.push_str("Z");

    path.push_str(r#"" "#);
    path
}

fn centroid_marker(
    centroid: &Point,
    ox: f64,
    oy: f64,
    scale: f64,
    options: &SvgExportOptions,
) -> String {
    let x = ox + centroid.x * scale;
    let y = oy - centroid.y * scale;
    let size = 8.0;

    format!(
        r##"<g stroke="#e74c3c" stroke-width="2" fill="none">
  <line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}"/>
  <line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}"/>
  <circle cx="{:.1}" cy="{:.1}" r="3" fill="#e74c3c"/>
  <text x="{:.1}" y="{:.1}" font-size="{}" text-anchor="middle" fill="#e74c3c" dy="-10">C</text>
</g>"##,
        x - size,
        y,
        x + size,
        y,
        x,
        y - size,
        x,
        y + size,
        x,
        y,
        x,
        y,
        options.font_size
    )
}

/// Export a "centroids" plot: section outline with the geometric centroid,
/// the shear centre, and the principal axes marked.
///
/// Mirrors Python `Section.plot_centroids()`.
pub fn plot_centroids(section: &Section, options: SvgExportOptions) -> String {
    use crate::plastic::warping::WarpingProperties;

    let props = SectionProperties::from_section(section);
    let warping = WarpingProperties::from_section(section, 0.3);
    let bounds = section.bounds();

    let section_width = bounds.1 - bounds.0;
    let section_height = bounds.3 - bounds.2;
    let max_dim = section_width.max(section_height);

    let scale = options.scale.unwrap_or_else(|| {
        let available_width = options.width as f64 - 2.0 * options.margin as f64;
        let available_height = options.height as f64 - 2.0 * options.margin as f64;
        (available_width / max_dim).min(available_height / max_dim)
    });

    let offset_x = options.width as f64 / 2.0 - (bounds.0 + bounds.1) / 2.0 * scale;
    let offset_y = options.height as f64 / 2.0 + (bounds.2 + bounds.3) / 2.0 * scale;

    let mut svg = String::new();
    svg.push_str(&format!(
        r#"<svg width="{}" height="{}" viewBox="0 0 {} {}" xmlns="http://www.w3.org/2000/svg">"#,
        options.width, options.height, options.width, options.height
    ));
    svg.push('\n');
    svg.push_str(&format!(
        r#"<rect width="100%" height="100%" fill="{}"/>"#,
        options.background_color
    ));
    svg.push('\n');

    if let Some(title) = &options.title {
        svg.push_str(&format!(
            r##"<text x="{}" y="{}" font-size="{}" text-anchor="middle" fill="#333">{}</text>"##,
            options.width / 2,
            options.margin / 2,
            options.font_size + 4,
            title
        ));
        svg.push('\n');
    }

    // Section geometry
    svg.push_str(&section_polygons(
        section, offset_x, offset_y, scale, &options,
    ));

    // Geometric centroid
    svg.push_str(&centroid_marker(
        &props.centroid,
        offset_x,
        offset_y,
        scale,
        &options,
    ));

    // Shear centre
    svg.push_str(&shear_center_marker(
        &warping.shear_center,
        offset_x,
        offset_y,
        scale,
        &options,
    ));

    // Principal axes
    svg.push_str(&principal_axes_svg(
        &props, offset_x, offset_y, scale, &options,
    ));

    svg.push_str("</svg>\n");
    svg
}

/// Export an interactive standalone HTML viewer for a section.
///
/// Embeds the centroids plot in an HTML page with mouse-wheel zoom and
/// click-drag panning, providing the Rust-side equivalent of Python's
/// matplotlib-based interactive figures.
pub fn to_interactive_html(section: &Section, options: SvgExportOptions) -> String {
    // Render the base figure at double resolution for crisp zooming.
    let mut opts = options.clone();
    opts.width *= 2;
    opts.height *= 2;
    let svg = plot_centroids(section, opts);

    // Wrap all drawing elements in a root group we can transform.
    let open = svg.find('>').expect("svg tag") + 1;
    let mut body = String::with_capacity(svg.len() + 64);
    body.push_str(&svg[..open]);
    body.push_str(r#"<g id="root">"#);
    body.push_str(&svg[open..svg.rfind("</svg>").expect("closing svg")]);
    body.push_str("</g></svg>");

    format!(
        r#"<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>section-properties interactive viewer</title>
<style>
  html, body {{ margin: 0; height: 100%; background: {bg}; overflow: hidden; }}
  #viewport {{ height: 100vh; cursor: grab; }}
  #viewport.dragging {{ cursor: grabbing; }}
</style>
</head>
<body>
<div id="viewport">
{body}
</div>
<script>
(function () {{
  var vp = document.getElementById('viewport');
  var root = document.getElementById('root');
  var scale = 1, tx = 0, ty = 0;

  function apply() {{
    root.setAttribute('transform',
      'translate(' + tx + ',' + ty + ') scale(' + scale + ')');
  }}

  vp.addEventListener('wheel', function (e) {{
    e.preventDefault();
    var factor = e.deltaY < 0 ? 1.15 : 1 / 1.15;
    var rect = vp.getBoundingClientRect();
    var mx = e.clientX - rect.left - rect.width / 2;
    var my = e.clientY - rect.top - rect.height / 2;
    tx = mx - (mx - tx) * factor;
    ty = my - (my - ty) * factor;
    scale *= factor;
    apply();
  }}, {{ passive: false }});

  var drag = null;
  vp.addEventListener('mousedown', function (e) {{
    drag = {{ x: e.clientX, y: e.clientY, tx: tx, ty: ty }};
    vp.classList.add('dragging');
  }});
  window.addEventListener('mousemove', function (e) {{
    if (!drag) return;
    tx = drag.tx + e.clientX - drag.x;
    ty = drag.ty + e.clientY - drag.y;
    apply();
  }});
  window.addEventListener('mouseup', function () {{
    drag = null;
    vp.classList.remove('dragging');
  }});
  vp.addEventListener('dblclick', function () {{ scale = 1; tx = 0; ty = 0; apply(); }});
}})();
</script>
</body>
</html>
"#,
        bg = options.background_color,
        body = body
    )
}
fn shear_center_marker(
    sc: &Point,
    ox: f64,
    oy: f64,
    scale: f64,
    options: &SvgExportOptions,
) -> String {
    let x = ox + sc.x * scale;
    let y = oy - sc.y * scale;
    let size = 8.0;
    format!(
        r##"<g stroke="#2980b9" stroke-width="2" fill="none">
  <line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}"/>
  <line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}"/>
  <circle cx="{:.1}" cy="{:.1}" r="4"/>
  <text x="{:.1}" y="{:.1}" font-size="{}" text-anchor="middle" fill="#2980b9" dy="-10">S</text>
</g>"##,
        x - size,
        y,
        x + size,
        y,
        x,
        y - size,
        x,
        y + size,
        x,
        y,
        x,
        y,
        options.font_size
    )
}

fn principal_axes_svg(
    props: &SectionProperties,
    ox: f64,
    oy: f64,
    scale: f64,
    options: &SvgExportOptions,
) -> String {
    let (i1, i2, angle) = props.principal_moments();
    let cx = ox + props.centroid.x * scale;
    let cy = oy - props.centroid.y * scale;

    // Length based on section size
    let length = ((props.ix + props.iy).sqrt() * scale * 0.4).max(50.0);

    let dx1 = angle.cos() * length;
    let dy1 = angle.sin() * length;
    let dx2 = (angle + std::f64::consts::FRAC_PI_2).cos() * length;
    let dy2 = (angle + std::f64::consts::FRAC_PI_2).sin() * length;

    format!(
        r##"<g stroke="#27ae60" stroke-width="1.5" fill="none" stroke-dasharray="8,4">
  <line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}"/>
  <line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}"/>
  <text x="{:.1}" y="{:.1}" font-size="{}" fill="#27ae60" text-anchor="start">U (I₁={:.2e})</text>
  <text x="{:.1}" y="{:.1}" font-size="{}" fill="#27ae60" text-anchor="start">V (I₂={:.2e})</text>
</g>"##,
        cx - dx1,
        cy - dy1,
        cx + dx1,
        cy + dy1,
        cx - dx2,
        cy - dy2,
        cx + dx2,
        cy + dy2,
        cx + dx1 + 5.0,
        cy + dy1 - 5.0,
        options.font_size,
        i1,
        cx + dx2 + 5.0,
        cy + dy2 + 5.0,
        options.font_size,
        i2
    )
}

fn coordinate_axes(
    props: &SectionProperties,
    ox: f64,
    oy: f64,
    scale: f64,
    options: &SvgExportOptions,
) -> String {
    let cx = ox + props.centroid.x * scale;
    let cy = oy - props.centroid.y * scale;
    let length = ((props.ix + props.iy).sqrt() * scale * 0.5).max(60.0);

    format!(
        r##"<g stroke="#95a5a6" stroke-width="1" fill="none" stroke-dasharray="4,4">
  <line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}"/>
  <line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}"/>
  <text x="{:.1}" y="{:.1}" font-size="{}" fill="#95a5a6" text-anchor="end">X</text>
  <text x="{:.1}" y="{:.1}" font-size="{}" fill="#95a5a6" text-anchor="start">Y</text>
</g>"##,
        cx - length,
        cy,
        cx + length,
        cy,
        cx,
        cy - length,
        cx,
        cy + length,
        cx + length + 5.0,
        cy + 5.0,
        options.font_size,
        cx - 5.0,
        cy - length - 5.0,
        options.font_size
    )
}

fn dimension_lines(
    section: &Section,
    ox: f64,
    oy: f64,
    scale: f64,
    options: &SvgExportOptions,
) -> String {
    let bounds = section.bounds();
    let min_x = ox + bounds.0 * scale;
    let max_x = ox + bounds.1 * scale;
    let min_y = oy - bounds.3 * scale;
    let max_y = oy - bounds.2 * scale;
    let offset = 30.0;

    let width = bounds.1 - bounds.0;
    let height = bounds.3 - bounds.2;

    format!(
        r##"<g stroke="#7f8c8d" stroke-width="1" fill="none" font-size="{}" font-family="monospace">
  <!-- Width dimension -->
  <line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}"/>
  <line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}"/>
  <text x="{:.1}" y="{:.1}" text-anchor="middle" fill="#7f8c8d">{:.1}</text>
  <!-- Height dimension -->
  <line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}"/>
  <line x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}"/>
  <text x="{:.1}" y="{:.1}" text-anchor="middle" fill="#7f8c8d" transform="rotate(-90 {:.1} {:.1})">{:.1}</text>
</g>"##,
        options.font_size,
        min_x,
        max_y + offset,
        max_x,
        max_y + offset,
        min_x,
        max_y + offset - 5.0,
        min_x,
        max_y + offset + 5.0,
        (min_x + max_x) / 2.0,
        max_y + offset + 15.0,
        width * 1000.0,
        min_x - offset,
        min_y,
        min_x - offset,
        max_y,
        min_x - offset + 5.0,
        min_y,
        min_x - offset - 5.0,
        max_y,
        min_x - offset - 20.0,
        (min_y + max_y) / 2.0,
        min_x - offset - 20.0,
        (min_y + max_y) / 2.0,
        height * 1000.0
    )
}

fn legend(options: &SvgExportOptions) -> String {
    let y_start = options.height as f64 - 80.0;
    let x_start = 20.0;

    format!(
        r##"<g font-size="{}" font-family="sans-serif">
  <rect x="{}" y="{}" width="15" height="15" fill="{}" fill-opacity="{}" stroke="{}" stroke-width="{}"/>
  <text x="{}" y="{}" fill="#333">Section</text>
  <line x1="{}" y1="{}" x2="{}" y2="{}" stroke="{}" stroke-width="{}" stroke-dasharray="5,5"/>
  <text x="{}" y="{}" fill="#333">Holes</text>
  <line x1="{}" y1="{}" x2="{}" y2="{}" stroke="#e74c3c" stroke-width="2"/>
  <circle cx="{}" cy="{}" r="3" fill="#e74c3c"/>
  <text x="{}" y="{}" fill="#333">Centroid</text>
  <line x1="{}" y1="{}" x2="{}" y2="{}" stroke="#27ae60" stroke-width="1.5" stroke-dasharray="8,4"/>
  <text x="{}" y="{}" fill="#333">Principal Axes</text>
</g>"##,
        options.font_size,
        x_start,
        y_start,
        options.fill_color,
        options.fill_opacity,
        options.outer_stroke_color,
        options.outer_stroke_width,
        x_start + 25.0,
        y_start + 12.0,
        x_start,
        y_start + 30.0,
        x_start + 15.0,
        y_start + 30.0,
        options.hole_stroke_color,
        options.hole_stroke_width,
        x_start + 25.0,
        y_start + 37.0,
        x_start,
        y_start + 50.0,
        x_start + 15.0,
        y_start + 50.0,
        x_start + 10.0,
        y_start + 50.0,
        x_start + 25.0,
        y_start + 57.0,
        x_start,
        y_start + 70.0,
        x_start + 15.0,
        y_start + 70.0,
        x_start + 25.0,
        y_start + 77.0
    )
}

/// Generate SVG for stress visualization (color-mapped).
pub fn to_svg_stress(
    section: &Section,
    stresses: &[StressResult],
    options: SvgExportOptions,
) -> String {
    // Find min/max von Mises for color scaling
    let max_vm = stresses.iter().map(|s| s.von_mises).fold(0.0, f64::max);
    let min_vm = stresses
        .iter()
        .map(|s| s.von_mises)
        .fold(f64::INFINITY, f64::min);

    let mut svg = String::new();
    let _bounds = section.bounds();

    // ... similar to to_svg but with stress-colored elements
    // This would require mesh integration - simplified version
    svg.push_str(&to_svg(section, options.clone()));

    // Add stress legend
    svg.push_str(&stress_legend(min_vm, max_vm, &options));

    svg
}

fn stress_legend(min_vm: f64, max_vm: f64, options: &SvgExportOptions) -> String {
    let y = options.height as f64 - 150.0;
    let x = 20.0;

    format!(
        r##"<g font-size="{}" font-family="sans-serif">
  <text x="{}" y="{}" fill="#333">Von Mises Stress (Pa)</text>
  <text x="{}" y="{}" fill="#333">{:.2e}</text>
  <text x="{}" y="{}" fill="#333">{:.2e}</text>
  <defs>
    <linearGradient id="stressGradient" x1="0%" y1="0%" x2="100%" y2="0%">
      <stop offset="0%" stop-color="#3498db"/>
      <stop offset="50%" stop-color="#f1c40f"/>
      <stop offset="100%" stop-color="#e74c3c"/>
    </linearGradient>
  </defs>
  <rect x="{}" y="{}" width="200" height="15" fill="url(#stressGradient)"/>
</g>"##,
        options.font_size,
        x,
        y - 20.0,
        x,
        y + 35.0,
        min_vm,
        x + 190.0,
        y + 35.0,
        max_vm,
        x,
        y
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon};
    use crate::section::Section;
    use crate::section_library::ParametricSection;
    use crate::section_library::steel::ISection;

    #[test]
    fn svg_export_basic() {
        let i = ISection::from_designation("IPE300").unwrap();
        let section = i.build();

        let svg = to_svg(&section, SvgExportOptions::default());

        assert!(svg.contains("<svg"));
        assert!(svg.contains("width=\"800\""));
        assert!(svg.contains("height=\"600\""));
        assert!(svg.contains("<path"));
        assert!(svg.contains("fill="));
        assert!(svg.contains("</svg>"));
    }

    #[test]
    fn interactive_html_contains_viewer() {
        use crate::section_library::primitive::RectangularSection;
        let section = RectangularSection::new(0.2, 0.1).build();
        let html = to_interactive_html(&section, SvgExportOptions::default());
        assert!(html.contains("<!DOCTYPE html>"));
        assert!(html.contains(r#"id="root""#));
        assert!(html.contains("addEventListener('wheel'"));
        assert!(html.contains(">C</text>"));
        assert!(html.contains(">S</text>"));
    }
    #[test]
    fn plot_centroids_marks_shear_center() {
        use crate::section_library::primitive::RectangularSection;
        let section = RectangularSection::new(0.2, 0.1).build();
        let svg = plot_centroids(&section, SvgExportOptions::default());
        assert!(svg.contains(">C</text>"));
        assert!(svg.contains(">S</text>"));
        assert!(svg.contains("I₁"));
        assert!(svg.contains("</svg>"));
    }
    #[test]
    fn svg_with_title() {
        let i = ISection::from_designation("IPE300").unwrap();
        let section = i.build();

        let options = SvgExportOptions {
            title: Some("IPE300 Section".to_string()),
            ..Default::default()
        };

        let svg = to_svg(&section, options);

        assert!(svg.contains("IPE300 Section"));
    }

    #[test]
    fn svg_with_holes() {
        let outer = Polygon::new(vec![
            Point::new(-0.1, -0.1),
            Point::new(0.1, -0.1),
            Point::new(0.1, 0.1),
            Point::new(-0.1, 0.1),
        ]);
        let hole = Polygon::new(vec![
            Point::new(-0.02, -0.02),
            Point::new(-0.02, 0.02),
            Point::new(0.02, 0.02),
            Point::new(0.02, -0.02),
        ]);
        let section = Section::new(outer, vec![hole]);

        let svg = to_svg(&section, SvgExportOptions::default());

        // Should have 2 paths (outer + hole)
        let path_count = svg.matches("<path").count();
        assert_eq!(path_count, 2);
    }

    #[test]
    fn svg_scale_option() {
        let i = ISection::from_designation("IPE300").unwrap();
        let section = i.build();

        let options = SvgExportOptions {
            scale: Some(1000.0), // 1000 px/m = 1 px/mm
            ..Default::default()
        };

        let svg = to_svg(&section, options);
        assert!(svg.contains("<svg"));
    }
}




// ---------------------------------------------------------------------------
// Warping function contour plot (mirrors Python plot_warping_function)
// ---------------------------------------------------------------------------

/// Diverging colour scale: blue (negative) -> white (zero) -> red (positive).
fn warp_colour(t: f64) -> String {
    let t = t.clamp(-1.0, 1.0);
    let (r, g, b) = if t < 0.0 {
        let s = -t;
        (
            (255.0 * (1.0 - s) + 40.0 * s) as u8,
            (255.0 * (1.0 - s) + 80.0 * s) as u8,
            (255.0 * (1.0 - s) + 200.0 * s) as u8,
        )
    } else {
        (
            (255.0 * (1.0 - t) + 220.0 * t) as u8,
            (255.0 * (1.0 - t) + 60.0 * t) as u8,
            (255.0 * (1.0 - t) + 60.0 * t) as u8,
        )
    };
    format!("rgb({r},{g},{b})")
}

/// Render the warping function ω over the FE mesh as a filled-contour SVG.
///
/// Mirrors Python `Section.plot_warping_function()`: the mesh is drawn with
/// each element coloured by its mean ω, using a diverging blue-white-red
/// scale normalised by max|ω|, plus a colour legend.
pub fn plot_warping_svg(
    mesh: &crate::fea::Tri6Mesh,
    omega: &[f64],
    options: SvgExportOptions,
) -> String {
    use std::fmt::Write as _;

    let _n = mesh.nodes.len();
    let omega_max = omega.iter().fold(0.0f64, |m, &v| m.max(v.abs())).max(1e-30);

    // Bounds
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for p in &mesh.nodes {
        min_x = min_x.min(p.x);
        max_x = max_x.max(p.x);
        min_y = min_y.min(p.y);
        max_y = max_y.max(p.y);
    }
    let span_x = (max_x - min_x).max(1e-15);
    let span_y = (max_y - min_y).max(1e-15);

    let avail_w = options.width as f64 - 2.5 * options.margin as f64;
    let avail_h = options.height as f64 - 2.0 * options.margin as f64;
    let scale = (avail_w / span_x).min(avail_h / span_y);
    let ox = options.margin as f64;
    let oy = options.height as f64 - options.margin as f64;

    let px = |x: f64| ox + (x - min_x) * scale;
    let py = |y: f64| oy - (y - min_y) * scale;

    let mut svg = String::new();
    svg.push_str(&format!(
        r#"<svg width="{}" height="{}" viewBox="0 0 {} {}" xmlns="http://www.w3.org/2000/svg">"#,
        options.width, options.height, options.width, options.height
    ));
    svg.push('\n');
    svg.push_str(&format!(
        r#"<rect width="100%" height="100%" fill="{}"/>"#,
        options.background_color
    ));
    svg.push('\n');
    if let Some(title) = &options.title {
        svg.push_str(&format!(
            r##"<text x="{}" y="{}" font-size="{}" text-anchor="middle" fill="#333">{}</text>"##,
            options.width / 2,
            options.margin / 2,
            options.font_size + 4,
            title
        ));
        svg.push('\n');
    }

    // Elements coloured by mean nodal ω.
    for elem in &mesh.elements {
        let mean_omega: f64 =
            elem.iter().map(|&ni| omega[ni]).sum::<f64>() / elem.len() as f64;
        let t = (mean_omega / omega_max).clamp(-1.0, 1.0);
        let fill = warp_colour(t);

        svg.push_str(r#"<path d=""#);
        for (k, &ni) in elem.iter().enumerate() {
            let p = &mesh.nodes[ni];
            if k == 0 {
                write!(svg, "M {:.2} {:.2} ", px(p.x), py(p.y)).unwrap();
            } else {
                write!(svg, "L {:.2} {:.2} ", px(p.x), py(p.y)).unwrap();
            }
        }
        svg.push_str("Z");
        svg.push_str(&format!(
            r#" " fill="{fill}" stroke="{stroke}" stroke-width="0.3"/>"#,
            stroke = options.outer_stroke_color
        ));
        svg.push('\n');
    }

    // Legend: vertical gradient bar.
    let bar_x = options.width as i32 - options.margin as i32 / 2 - 10;
    let bar_top = options.margin;
    let bar_h = options.height - 2 * options.margin;
    svg.push_str("<defs><linearGradient id=\"wleg\" x1=\"0\" y1=\"1\" x2=\"0\" y2=\"0\">");
    for s in 0..=10 {
        let t = -1.0 + 0.2 * s as f64;
        let offset = (s as f64) / 10.0 * 100.0;
        let _write_ok = writeln!(
            svg,
            r#"<stop offset="{offset:.0}%" stop-color="{}"/>"#,
            warp_colour(t)
        );
    }
    svg.push_str("</linearGradient></defs>\n");
    svg.push_str(&format!(
        r##"<rect x="{}" y="{}" width="14" height="{}" fill="url(#wleg)" stroke="#333" stroke-width="0.5"/>"##,
        bar_x, bar_top, bar_h
    ));
    svg.push('\n');

    let _ = writeln!(
        svg,
        r##"<text x="{}" y="{}" font-size="{}" fill="#333">+w max = {:+.3e}</text>"##,
        bar_x - 4,
        bar_top as f64 + options.font_size as f64,
        options.font_size,
        omega_max
    );
    let _ = writeln!(
        svg,
        r##"<text x="{}" y="{}" font-size="{}" fill="#333">-w max = {:-.3e}</text>"##,
        bar_x - 4,
        bar_top + bar_h,
        options.font_size,
        omega_max
    );

    svg.push_str("</svg>\n");
    svg
}