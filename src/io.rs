//! Input/Output module for section data exchange.
//!
//! Supports DXF, JSON, CSV, and other formats for interoperability
//! with CAD software and other engineering tools.

pub mod csv;
pub mod dxf;
pub mod json;
pub mod svg;

pub use csv::{CsvExportOptions, from_csv, to_csv};
pub use dxf::{DxfColor, DxfExportOptions, to_dxf};
pub use json::{JsonMaterial, JsonSection, from_json, to_json};
pub use svg::{SvgExportOptions, to_svg};

use crate::material::Material;
use crate::section::Section;
use crate::section_library::{CompositeSection, ParametricSection};
use crate::section_properties::SectionProperties;

/// Export a section to multiple formats.
pub struct SectionExporter {
    pub section: Section,
    pub properties: Option<SectionProperties>,
    pub material: Option<Material>,
    pub designation: Option<String>,
}

impl SectionExporter {
    pub fn new(section: Section) -> Self {
        Self {
            section,
            properties: None,
            material: None,
            designation: None,
        }
    }

    pub fn with_properties(mut self, props: SectionProperties) -> Self {
        self.properties = Some(props);
        self
    }

    pub fn with_material(mut self, material: Material) -> Self {
        self.material = Some(material);
        self
    }

    pub fn with_designation(mut self, designation: String) -> Self {
        self.designation = Some(designation);
        self
    }

    /// Export to DXF string.
    pub fn to_dxf(&self, options: DxfExportOptions) -> String {
        dxf::to_dxf(&self.section, options)
    }

    /// Export to JSON string.
    pub fn to_json(&self, pretty: bool) -> String {
        json::to_json(
            &self.section,
            self.properties.as_ref(),
            self.material.as_ref(),
            self.designation.as_deref(),
            pretty,
        )
    }

    /// Export to CSV string.
    pub fn to_csv(&self, options: CsvExportOptions) -> String {
        csv::to_csv(&self.section, options)
    }

    /// Export to SVG string.
    pub fn to_svg(&self, options: SvgExportOptions) -> String {
        svg::to_svg(&self.section, options)
    }

    /// Save to file (auto-detect format from extension).
    pub fn save<P: AsRef<std::path::Path>>(&self, path: P) -> Result<(), std::io::Error> {
        let path = path.as_ref();
        let ext = path
            .extension()
            .and_then(|s| s.to_str())
            .unwrap_or("")
            .to_lowercase();

        let content = match ext.as_str() {
            "dxf" => self.to_dxf(DxfExportOptions::default()),
            "json" => self.to_json(true),
            "csv" => self.to_csv(CsvExportOptions::default()),
            "svg" => self.to_svg(SvgExportOptions::default()),
            _ => {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    "Unsupported format",
                ));
            }
        };

        std::fs::write(path, content)
    }
}

/// Import a section from various formats.
pub struct SectionImporter;

impl SectionImporter {
    /// Import from JSON.
    pub fn from_json(json_str: &str) -> Result<JsonSection, serde_json::Error> {
        json::from_json(json_str)
    }

    /// Import from CSV.
    pub fn from_csv(csv_str: &str) -> Result<Section, Box<dyn std::error::Error>> {
        csv::from_csv(csv_str)
    }

    /// Import from DXF (simplified - reads POLYLINE/LWPOLYLINE).
    pub fn from_dxf(dxf_str: &str) -> Result<Vec<Section>, DxfImportError> {
        dxf::from_dxf(dxf_str)
    }
}

/// DXF import errors.
#[derive(Debug, Clone)]
pub enum DxfImportError {
    ParseError(String),
    NoClosedPolylines,
    InvalidGeometry(String),
}

impl std::fmt::Display for DxfImportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DxfImportError::ParseError(e) => write!(f, "DXF parse error: {}", e),
            DxfImportError::NoClosedPolylines => write!(f, "No closed polylines found"),
            DxfImportError::InvalidGeometry(e) => write!(f, "Invalid geometry: {}", e),
        }
    }
}

impl std::error::Error for DxfImportError {}

/// Batch export multiple sections.
pub fn export_section_library<P: AsRef<std::path::Path>>(
    sections: &[(String, Section)],
    directory: P,
    format: ExportFormat,
) -> Result<(), std::io::Error> {
    let dir = directory.as_ref();
    std::fs::create_dir_all(dir)?;

    for (name, section) in sections {
        let safe_name = name.replace('/', "_").replace('\\', "_").replace(':', "_");
        let filename = format!("{}.{}", safe_name, format.extension());
        let path = dir.join(filename);

        let exporter = SectionExporter::new(section.clone());
        exporter.save(&path)?;
    }

    Ok(())
}

/// Export format options.
#[derive(Debug, Clone, Copy)]
pub enum ExportFormat {
    Dxf,
    Json,
    Csv,
    Svg,
}

impl ExportFormat {
    pub fn extension(&self) -> &'static str {
        match self {
            ExportFormat::Dxf => "dxf",
            ExportFormat::Json => "json",
            ExportFormat::Csv => "csv",
            ExportFormat::Svg => "svg",
        }
    }
}

/// Create a section from a parametric section for export.
pub fn section_from_parametric<S: ParametricSection>(parametric: &S) -> SectionExporter {
    SectionExporter::new(parametric.build()).with_designation(parametric.designation())
}

/// Create a section from composite section for export.
pub fn section_from_composite(composite: &CompositeSection) -> SectionExporter {
    let section = crate::section::Section::new(composite.outer.clone(), composite.holes.clone());
    SectionExporter::new(section)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon};
    use crate::material::presets::STEEL_S355;
    use crate::section::Section;
    use crate::section_library::steel::ISection;

    #[test]
    fn exporter_basic() {
        let i = ISection::from_designation("IPE300").unwrap();
        let section = i.build();

        let exporter = SectionExporter::new(section)
            .with_material(STEEL_S355)
            .with_designation("IPE300".to_string());

        let dxf = exporter.to_dxf(DxfExportOptions::default());
        assert!(dxf.contains("SECTION"));
        assert!(dxf.contains("POLYLINE"));

        let json = exporter.to_json(true);
        assert!(json.contains("IPE300"));
        assert!(json.contains("vertices"));

        let svg = exporter.to_svg(SvgExportOptions::default());
        assert!(svg.contains("<svg"));
        assert!(svg.contains("<path"));
    }

    #[test]
    fn json_roundtrip() {
        let i = ISection::from_designation("IPE300").unwrap();
        let section = i.build();

        let exporter = SectionExporter::new(section.clone()).with_designation("IPE300".to_string());

        let json = exporter.to_json(false);
        let imported = SectionImporter::from_json(&json).unwrap();

        assert_eq!(imported.designation, Some("IPE300".to_string()));
        assert_eq!(imported.outer_vertices.len(), section.outer.vertices.len());
    }
}
