//! DXF export for section geometry.
//!
//! Exports section outer boundary and holes as LWPOLYLINE entities
//! compatible with AutoCAD, BricsCAD, and other CAD software.

use crate::geometry::{Point, Polygon};
use crate::section::Section;
use crate::section_properties::SectionProperties;

/// DXF export options.
#[derive(Debug, Clone)]
pub struct DxfExportOptions {
    /// Layer name for outer boundary
    pub outer_layer: String,
    /// Layer name for holes
    pub hole_layer: String,
    /// Layer name for centroid marker
    pub centroid_layer: String,
    /// Layer name for principal axes
    pub axes_layer: String,
    /// Color for outer boundary (ACI 1-255)
    pub outer_color: DxfColor,
    /// Color for holes
    pub hole_color: DxfColor,
    /// Color for centroid
    pub centroid_color: DxfColor,
    /// Color for axes
    pub axes_color: DxfColor,
    /// Lineweight (0 = default, 1-211 = 0.01-2.11mm)
    pub lineweight: i16,
    /// Include centroid marker
    pub include_centroid: bool,
    /// Include principal axes
    pub include_axes: bool,
    /// Axis length factor (multiple of section height)
    pub axis_length_factor: f64,
    /// Unit: "m", "mm", "cm", "in"
    pub units: String,
    /// Precision for coordinates
    pub precision: usize,
}

impl Default for DxfExportOptions {
    fn default() -> Self {
        Self {
            outer_layer: "SECTION_OUTER".to_string(),
            hole_layer: "SECTION_HOLES".to_string(),
            centroid_layer: "SECTION_CENTROID".to_string(),
            axes_layer: "SECTION_AXES".to_string(),
            outer_color: DxfColor::ByLayer,
            hole_color: DxfColor::ByLayer,
            centroid_color: DxfColor::Red,
            axes_color: DxfColor::Green,
            lineweight: 0,
            include_centroid: true,
            include_axes: true,
            axis_length_factor: 0.5,
            units: "m".to_string(),
            precision: 6,
        }
    }
}

/// DXF color specification.
#[derive(Debug, Clone, Copy)]
pub enum DxfColor {
    ByLayer = 256,
    ByBlock = 0,
    Red = 1,
    Yellow = 2,
    Green = 3,
    Cyan = 4,
    Blue = 5,
    Magenta = 6,
    White = 7,
    Custom(u8),
}

impl DxfColor {
    fn to_aci(&self) -> i16 {
        match self {
            DxfColor::ByLayer => 256,
            DxfColor::ByBlock => 0,
            DxfColor::Red => 1,
            DxfColor::Yellow => 2,
            DxfColor::Green => 3,
            DxfColor::Cyan => 4,
            DxfColor::Blue => 5,
            DxfColor::Magenta => 6,
            DxfColor::White => 7,
            DxfColor::Custom(c) => *c as i16,
        }
    }
}

/// Export a section to DXF format.
pub fn to_dxf(section: &Section, options: DxfExportOptions) -> String {
    let mut dxf = String::new();

    // HEADER section
    dxf.push_str(&header_section(&options));

    // TABLES section (layers)
    dxf.push_str(&tables_section(&options));

    // ENTITIES section
    dxf.push_str("  0\nSECTION\n  2\nENTITIES\n");

    // Outer boundary
    dxf.push_str(&entity_polyline(
        &section.outer.vertices,
        &options.outer_layer,
        options.outer_color,
        options.lineweight,
        true,
        options.precision,
    ));

    // Holes
    for hole in &section.holes {
        dxf.push_str(&entity_polyline(
            &hole.vertices,
            &options.hole_layer,
            options.hole_color,
            options.lineweight,
            true,
            options.precision,
        ));
    }

    // Centroid marker
    if options.include_centroid {
        let props = SectionProperties::from_section(section);
        dxf.push_str(&entity_centroid_marker(&props.centroid, &options));
    }

    // Principal axes
    if options.include_axes {
        let props = SectionProperties::from_section(section);
        dxf.push_str(&entity_principal_axes(&props, &options));
    }

    dxf.push_str("  0\nENDSEC\n  0\nEOF\n");

    dxf
}

fn header_section(options: &DxfExportOptions) -> String {
    let mut s = String::new();
    s.push_str("  0\nSECTION\n  2\nHEADER\n");

    // Units
    let insunits = match options.units.as_str() {
        "mm" => 4,
        "cm" => 5,
        "m" => 6,
        "in" => 1,
        _ => 6,
    };
    s.push_str(&format!("  9\n$INSUNITS\n 70\n{}\n", insunits));

    // Precision
    s.push_str(&format!("  9\n$LUPREC\n 70\n{}\n", options.precision));

    s.push_str("  0\nENDSEC\n");
    s
}

fn tables_section(options: &DxfExportOptions) -> String {
    let mut s = String::new();
    s.push_str("  0\nSECTION\n  2\nTABLES\n");

    // LAYER table
    s.push_str("  0\nTABLE\n  2\nLAYER\n 70\n4\n"); // 4 layers

    for (name, color) in &[
        (&options.outer_layer, options.outer_color),
        (&options.hole_layer, options.hole_color),
        (&options.centroid_layer, options.centroid_color),
        (&options.axes_layer, options.axes_color),
    ] {
        s.push_str("  0\nLAYER\n");
        s.push_str(&format!("  2\n{}\n", name));
        s.push_str(" 70\n     0\n"); // Not frozen/locked
        s.push_str(&format!(" 62\n{}\n", color.to_aci()));
        s.push_str(&format!(" 370\n{}\n", options.lineweight.max(0)));
    }

    s.push_str("  0\nENDTAB\n");
    s.push_str("  0\nENDSEC\n");
    s
}

fn entity_polyline(
    vertices: &[Point],
    layer: &str,
    color: DxfColor,
    lineweight: i16,
    closed: bool,
    precision: usize,
) -> String {
    let mut s = String::new();

    s.push_str("  0\nLWPOLYLINE\n");
    s.push_str(&format!("  8\n{}\n", layer));
    s.push_str(&format!(" 62\n{}\n", color.to_aci()));
    s.push_str(&format!(" 370\n{}\n", lineweight.max(0)));
    s.push_str(" 90\n"); // Number of vertices
    s.push_str(&format!("{}\n", vertices.len()));
    s.push_str(" 70\n"); // Closed flag
    s.push_str(&format!("{}\n", if closed { 1 } else { 0 }));

    let fmt_str = format!("{{:.{}e}}", precision);
    for v in vertices {
        s.push_str(" 10\n");
        s.push_str(&format!("{}\n", fmt_str.format(&v.x)));
        s.push_str(" 20\n");
        s.push_str(&format!("{}\n", fmt_str.format(&v.y)));
    }

    s
}

fn entity_centroid_marker(centroid: &Point, options: &DxfExportOptions) -> String {
    let mut s = String::new();
    let size = 0.01; // 10mm marker

    // Crosshair lines
    let points = [
        (centroid.x - size, centroid.y, centroid.x + size, centroid.y),
        (centroid.x, centroid.y - size, centroid.x, centroid.y + size),
    ];

    for (x1, y1, x2, y2) in points {
        s.push_str("  0\nLINE\n");
        s.push_str(&format!("  8\n{}\n", options.centroid_layer));
        s.push_str(&format!(" 62\n{}\n", options.centroid_color.to_aci()));
        s.push_str(&format!(" 10\n{:.6}\n", x1));
        s.push_str(&format!(" 20\n{:.6}\n", y1));
        s.push_str(&format!(" 11\n{:.6}\n", x2));
        s.push_str(&format!(" 21\n{:.6}\n", y2));
    }

    s
}

fn entity_principal_axes(props: &SectionProperties, options: &DxfExportOptions) -> String {
    let mut s = String::new();

    let (i1, i2, angle) = props.principal_moments();
    let cx = props.centroid.x;
    let cy = props.centroid.y;

    // Length based on section size
    let length = (props.ix + props.iy).sqrt() * options.axis_length_factor;

    // Principal axis 1 (major)
    let dx1 = angle.cos() * length;
    let dy1 = angle.sin() * length;

    // Principal axis 2 (minor)
    let dx2 = (angle + std::f64::consts::FRAC_PI_2).cos() * length;
    let dy2 = (angle + std::f64::consts::FRAC_PI_2).sin() * length;

    // Axis 1 line
    s.push_str("  0\nLINE\n");
    s.push_str(&format!("  8\n{}\n", options.axes_layer));
    s.push_str(&format!(" 62\n{}\n", options.axes_color.to_aci()));
    s.push_str(" 48\n0.5\n"); // Dashed linetype scale
    s.push_str(&format!(" 10\n{:.6}\n", cx - dx1));
    s.push_str(&format!(" 20\n{:.6}\n", cy - dy1));
    s.push_str(&format!(" 11\n{:.6}\n", cx + dx1));
    s.push_str(&format!(" 21\n{:.6}\n", cy + dy1));

    // Axis 2 line
    s.push_str("  0\nLINE\n");
    s.push_str(&format!("  8\n{}\n", options.axes_layer));
    s.push_str(&format!(" 62\n{}\n", options.axes_color.to_aci()));
    s.push_str(&format!(" 10\n{:.6}\n", cx - dx2));
    s.push_str(&format!(" 20\n{:.6}\n", cy - dy2));
    s.push_str(&format!(" 11\n{:.6}\n", cx + dx2));
    s.push_str(&format!(" 21\n{:.6}\n", cy + dy2));

    // Labels (TEXT entities)
    s.push_str(&entity_text(
        cx + dx1 * 1.1,
        cy + dy1 * 1.1,
        "U",
        0.02,
        &options.axes_layer,
        options.axes_color,
    ));
    s.push_str(&entity_text(
        cx + dx2 * 1.1,
        cy + dy2 * 1.1,
        "V",
        0.02,
        &options.axes_layer,
        options.axes_color,
    ));

    s
}

fn entity_text(x: f64, y: f64, text: &str, height: f64, layer: &str, color: DxfColor) -> String {
    format!(
        "  0\nTEXT\n  8\n{}\n 62\n{}\n 10\n{:.6}\n 20\n{:.6}\n 40\n{:.6}\n  1\n{}\n",
        layer,
        color.to_aci(),
        x,
        y,
        height,
        text
    )
}

/// Import DXF and extract closed polylines as sections.
pub fn from_dxf(dxf_content: &str) -> Result<Vec<Section>, crate::io::DxfImportError> {
    // Simplified DXF parser for LWPOLYLINE entities
    let lines: Vec<&str> = dxf_content.lines().collect();
    let mut polylines = Vec::new();
    let mut i = 0;

    while i < lines.len() {
        let line = lines[i].trim();
        if line == "LWPOLYLINE" || line == "POLYLINE" {
            if let Some(poly) = parse_polyline(&lines, &mut i) {
                if poly.len() >= 3 {
                    polylines.push(poly);
                }
            }
        }
        i += 1;
    }

    if polylines.is_empty() {
        return Err(crate::io::DxfImportError::NoClosedPolylines);
    }

    // First polyline = outer, rest = holes (simplified)
    let outer = Polygon::new(polylines[0].clone());
    let holes = polylines[1..]
        .iter()
        .map(|v| Polygon::new(v.clone()))
        .collect();

    Ok(vec![Section::new(outer, holes)])
}

fn parse_polyline(lines: &[&str], index: &mut usize) -> Option<Vec<Point>> {
    let mut vertices = Vec::new();
    let mut x = None;
    let mut y = None;
    let mut _count = 0;

    while *index < lines.len() {
        let code = lines[*index].trim();
        *index += 1;

        if code == "0" {
            // Next entity
            *index -= 1;
            break;
        }

        if *index >= lines.len() {
            break;
        }

        let value = lines[*index].trim();
        *index += 1;

        match code {
            "90" => _count = value.parse().unwrap_or(0), // Vertex count
            "10" => x = value.parse().ok(),              // X coordinate
            "20" => y = value.parse().ok(),              // Y coordinate
            "70" => { /* flags */ }
            _ => {}
        }

        if let (Some(x_val), Some(y_val)) = (x, y) {
            vertices.push(crate::geometry::Point::new(x_val, y_val));
            x = None;
            y = None;
        }
    }

    if vertices.len() >= 3 {
        Some(vertices)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon};
    use crate::section::Section;
    use crate::section_library::steel::ISection;

    #[test]
    fn dxf_export_i_section() {
        let i = ISection::from_designation("IPE300").unwrap();
        let section = i.build();

        let dxf = to_dxf(&section, DxfExportOptions::default());

        assert!(dxf.contains("SECTION"));
        assert!(dxf.contains("HEADER"));
        assert!(dxf.contains("TABLES"));
        assert!(dxf.contains("ENTITIES"));
        assert!(dxf.contains("LWPOLYLINE"));
        assert!(dxf.contains("SECTION_OUTER"));
        assert!(dxf.contains("ENDSEC"));
        assert!(dxf.contains("EOF"));
    }

    #[test]
    fn dxf_with_holes() {
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

        let dxf = to_dxf(&section, DxfExportOptions::default());

        // Should have 2 polylines
        let polyline_count = dxf.matches("LWPOLYLINE").count();
        assert_eq!(polyline_count, 2);
    }

    #[test]
    fn dxf_layers() {
        let i = ISection::from_designation("IPE300").unwrap();
        let section = i.build();

        let options = DxfExportOptions {
            outer_layer: "MY_OUTER".to_string(),
            hole_layer: "MY_HOLES".to_string(),
            ..Default::default()
        };

        let dxf = to_dxf(&section, options);

        assert!(dxf.contains("MY_OUTER"));
        assert!(dxf.contains("MY_HOLES"));
    }
}
