//! CSV export for section properties and geometry.
//!
//! Exports section properties in tabular format for spreadsheets
//! and engineering reports.

use crate::geometry::{Point, Polygon};
use crate::section::Section;
use crate::section_properties::SectionProperties;

/// CSV export options.
#[derive(Debug, Clone)]
pub struct CsvExportOptions {
    /// Include header row
    pub include_header: bool,
    /// Decimal separator
    pub decimal_separator: char,
    /// Field delimiter
    pub delimiter: char,
    /// Include geometry vertices
    pub include_vertices: bool,
    /// Include principal properties
    pub include_principal: bool,
    /// Include gyration properties
    pub include_gyration: bool,
    /// Units for output (m, mm, cm, in)
    pub units: String,
    /// Precision for numeric values
    pub precision: usize,
}

impl Default for CsvExportOptions {
    fn default() -> Self {
        Self {
            include_header: true,
            decimal_separator: '.',
            delimiter: ',',
            include_vertices: true,
            include_principal: true,
            include_gyration: true,
            units: "m".to_string(),
            precision: 6,
        }
    }
}

/// Export section properties to CSV.
pub fn to_csv(section: &Section, options: CsvExportOptions) -> String {
    let props = SectionProperties::from_section(section);
    let mut csv = String::new();
    let fmt_val = |v: f64| format!("{:.p$e}", v, p = options.precision);

    // Unit conversion factor
    let unit_factor: f64 = match options.units.as_str() {
        "mm" => 1000.0,
        "cm" => 100.0,
        "in" => 39.3701,
        _ => 1.0,
    };
    let area_factor = unit_factor * unit_factor;
    let inertia_factor = unit_factor.powi(4);

    // Header
    if options.include_header {
        csv.push_str("Property,Value,Unit\n");
    }

    // Basic properties
    csv.push_str(&format!(
        "Area,{},{}^2\n",
        fmt_val(props.area * area_factor),
        options.units
    ));
    csv.push_str(&format!(
        "CentroidX,{},{}\n",
        fmt_val(props.centroid.x * unit_factor),
        options.units
    ));
    csv.push_str(&format!(
        "CentroidY,{},{}\n",
        fmt_val(props.centroid.y * unit_factor),
        options.units
    ));
    csv.push_str(&format!(
        "Ix,{},{}^4\n",
        fmt_val(props.ix * inertia_factor),
        options.units
    ));
    csv.push_str(&format!(
        "Iy,{},{}^4\n",
        fmt_val(props.iy * inertia_factor),
        options.units
    ));
    csv.push_str(&format!(
        "Ixy,{},{}^4\n",
        fmt_val(props.ixy * inertia_factor),
        options.units
    ));

    // Principal properties
    if options.include_principal {
        let (i1, i2, angle) = props.principal_moments();
        csv.push_str(&format!(
            "I1 (Major),,{},{}^4\n",
            fmt_val(i1 * inertia_factor),
            options.units
        ));
        csv.push_str(&format!(
            "I2 (Minor),,{},{}^4\n",
            fmt_val(i2 * inertia_factor),
            options.units
        ));
        csv.push_str(&format!("PrincipalAngle,{},rad\n", fmt_val(angle)));
        csv.push_str(&format!(
            "PrincipalAngleDeg,{},deg\n",
            fmt_val(angle.to_degrees())
        ));
    }

    // Gyration properties
    if options.include_gyration {
        let (rx, ry, rp) = props.radius_of_gyration();
        csv.push_str(&format!(
            "rx,{},{}\n",
            fmt_val(rx * unit_factor),
            options.units
        ));
        csv.push_str(&format!(
            "ry,{},{}\n",
            fmt_val(ry * unit_factor),
            options.units
        ));
        csv.push_str(&format!(
            "rp (polar),,{},{}\n",
            fmt_val(rp * unit_factor),
            options.units
        ));
    }

    // Geometry vertices
    if options.include_vertices {
        csv.push_str("\n# Geometry Vertices\n");
        if options.include_header {
            csv.push_str("Type,Index,X,Y\n");
        }

        for (i, v) in section.outer.vertices.iter().enumerate() {
            csv.push_str(&format!(
                "Outer,{},{},{}\n",
                i,
                fmt_val(v.x * unit_factor),
                fmt_val(v.y * unit_factor)
            ));
        }

        for (h_idx, hole) in section.holes.iter().enumerate() {
            for (_i, v) in hole.vertices.iter().enumerate() {
                csv.push_str(&format!(
                    "Hole{},,{},{}\n",
                    h_idx,
                    fmt_val(v.x * unit_factor),
                    fmt_val(v.y * unit_factor)
                ));
            }
        }
    }

    // Replace delimiter if not comma (must do before decimal separator
    // to avoid replacing decimal commas with the delimiter)
    if options.delimiter != ',' {
        csv = csv.replace(',', &options.delimiter.to_string());
    }

    // Replace decimal separator if needed
    if options.decimal_separator != '.' {
        csv = csv.replace('.', &options.decimal_separator.to_string());
    }

    csv
}

/// Export multiple sections to CSV (one row per section).
pub fn to_csv_catalog(sections: &[(String, Section)], options: CsvExportOptions) -> String {
    let mut csv = String::new();
    let fmt_val = |v: f64| format!("{:.p$e}", v, p = options.precision);
    let unit_factor: f64 = match options.units.as_str() {
        "mm" => 1000.0,
        "cm" => 100.0,
        "in" => 39.3701,
        _ => 1.0,
    };
    let area_factor = unit_factor * unit_factor;
    let inertia_factor = unit_factor.powi(4);

    if options.include_header {
        csv.push_str("Designation,Area,Ix,Iy,Ixy,rx,ry,Angle,AngleDeg\n");
    }

    for (name, section) in sections {
        let props = SectionProperties::from_section(section);
        let (_i1, _i2, angle) = props.principal_moments();
        let (rx, ry, _) = props.radius_of_gyration();

        csv.push_str(&format!(
            "{},{},{},{},{},{},{},{},{}\n",
            name,
            fmt_val(props.area * area_factor),
            fmt_val(props.ix * inertia_factor),
            fmt_val(props.iy * inertia_factor),
            fmt_val(props.ixy * inertia_factor),
            fmt_val(rx * unit_factor),
            fmt_val(ry * unit_factor),
            fmt_val(angle),
            fmt_val(angle.to_degrees()),
        ));
    }

    if options.decimal_separator != '.' {
        csv = csv.replace('.', &options.decimal_separator.to_string());
    }
    if options.delimiter != ',' {
        csv = csv.replace(',', &options.delimiter.to_string());
    }

    csv
}

/// Import section from CSV (geometry only).
pub fn from_csv(csv_str: &str) -> Result<Section, Box<dyn std::error::Error>> {
    let mut rdr = csv::ReaderBuilder::new()
        .has_headers(true)
        .flexible(true)
        .from_reader(csv_str.as_bytes());

    let mut outer_vertices = Vec::new();
    let mut holes: Vec<Vec<Point>> = Vec::new();

    for result in rdr.records() {
        let record = result?;

        // Expected columns: Type,Index,X,Y
        let type_ = record.get(0).unwrap_or("");
        let _idx = record.get(1).unwrap_or("");
        let x_str = record.get(2).unwrap_or("");
        let y_str = record.get(3).unwrap_or("");

        let x = x_str.parse::<f64>().unwrap_or(0.0);
        let y = y_str.parse::<f64>().unwrap_or(0.0);

        match type_ {
            "Outer" => outer_vertices.push(Point::new(x, y)),
            t if t.starts_with("Hole") => {
                let hole_idx = t
                    .strip_prefix("Hole")
                    .unwrap_or("0")
                    .parse::<usize>()
                    .unwrap_or(0);
                while holes.len() <= hole_idx {
                    holes.push(Vec::new());
                }
                holes[hole_idx].push(Point::new(x, y));
            }
            _ => {}
        }
    }

    let outer = Polygon::new(outer_vertices);
    let hole_polys: Vec<Polygon> = holes.into_iter().map(Polygon::new).collect();

    Ok(Section::new(outer, hole_polys))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon};
    use crate::section::Section;
    use crate::section_library::ParametricSection;
    use crate::section_library::steel::ISection;

    #[test]
    fn csv_export_properties() {
        let i = ISection::from_designation("IPE300").unwrap();
        let section = i.build();

        let csv = to_csv(&section, CsvExportOptions::default());

        assert!(csv.contains("Area"));
        assert!(csv.contains("Ix"));
        assert!(csv.contains("Iy"));
        assert!(csv.contains("PrincipalAngle"));
        assert!(csv.contains("rx"));
    }

    #[test]
    fn csv_catalog() {
        let i1 = ISection::from_designation("IPE300").unwrap();
        let i2 = ISection::from_designation("HEB200").unwrap();

        let sections = vec![
            ("IPE300".to_string(), i1.build()),
            ("HEB200".to_string(), i2.build()),
        ];

        let csv = to_csv_catalog(&sections, CsvExportOptions::default());

        assert!(csv.contains("IPE300"));
        assert!(csv.contains("HEB200"));
        assert!(csv.contains("Designation"));
    }

    #[test]
    fn csv_custom_options() {
        let i = ISection::from_designation("IPE300").unwrap();
        let section = i.build();

        let options = CsvExportOptions {
            delimiter: ';',
            decimal_separator: ',',
            units: "mm".to_string(),
            precision: 3,
            ..Default::default()
        };

        let csv = to_csv(&section, options);

        assert!(csv.contains(";"));
        assert!(csv.contains(",")); // decimal comma
        assert!(csv.contains("mm"));
    }

    #[test]
    fn csv_import_geometry() {
        let outer = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(1.0, 1.0),
            Point::new(0.0, 1.0),
        ]);
        let hole = Polygon::new(vec![
            Point::new(0.3, 0.3),
            Point::new(0.3, 0.7),
            Point::new(0.7, 0.7),
            Point::new(0.7, 0.3),
        ]);
        let section = Section::new(outer, vec![hole]);

        let csv = to_csv(&section, CsvExportOptions::default());
        let imported = from_csv(&csv).unwrap();

        assert_eq!(imported.outer.vertices.len(), 4);
        assert_eq!(imported.holes.len(), 1);
        assert_eq!(imported.holes[0].vertices.len(), 4);
    }
}
