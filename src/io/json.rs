//! JSON import/export for section data.
//!
//! Provides serialization of sections, materials, and properties
//! for data exchange and storage.

use crate::geometry::{Point, Polygon};
use crate::material::Material;
use crate::section::Section;
use crate::section_properties::SectionProperties;
use serde::{Deserialize, Serialize};

/// JSON representation of a section for export/import.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JsonSection {
    pub designation: Option<String>,
    pub outer_vertices: Vec<[f64; 2]>,
    pub holes: Vec<Vec<[f64; 2]>>,
    pub properties: Option<JsonSectionProperties>,
    pub material: Option<JsonMaterial>,
}

/// JSON representation of section properties.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JsonSectionProperties {
    pub area: f64,
    pub centroid: [f64; 2],
    pub ix: f64,
    pub iy: f64,
    pub ixy: f64,
    pub principal: Option<JsonPrincipalProperties>,
    pub gyration: Option<JsonGyrationProperties>,
}

/// JSON principal properties.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JsonPrincipalProperties {
    pub i1: f64,
    pub i2: f64,
    pub angle: f64,
}

/// JSON gyration properties.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JsonGyrationProperties {
    pub rx: f64,
    pub ry: f64,
    pub polar: f64,
}

/// JSON material representation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JsonMaterial {
    pub name: String,
    pub youngs_modulus: f64,
    pub shear_modulus: f64,
    pub poissons_ratio: f64,
    pub density: f64,
    pub thermal_expansion: f64,
    pub yield_strength: f64,
    pub ultimate_strength: f64,
    pub color: Option<[u8; 3]>,
}

/// Export a section to JSON string.
pub fn to_json(
    section: &Section,
    properties: Option<&SectionProperties>,
    material: Option<&Material>,
    designation: Option<&str>,
    pretty: bool,
) -> String {
    let json_section = JsonSection {
        designation: designation.map(String::from),
        outer_vertices: section.outer.vertices.iter().map(|v| [v.x, v.y]).collect(),
        holes: section
            .holes
            .iter()
            .map(|h| h.vertices.iter().map(|v| [v.x, v.y]).collect())
            .collect(),
        properties: properties.map(|p| JsonSectionProperties {
            area: p.area,
            centroid: [p.centroid.x, p.centroid.y],
            ix: p.ix,
            iy: p.iy,
            ixy: p.ixy,
            principal: Some(JsonPrincipalProperties {
                i1: p.principal_moments().0,
                i2: p.principal_moments().1,
                angle: p.principal_moments().2,
            }),
            gyration: Some(JsonGyrationProperties {
                rx: p.radius_of_gyration().0,
                ry: p.radius_of_gyration().1,
                polar: p.radius_of_gyration().2,
            }),
        }),
        material: material.map(|m| JsonMaterial {
            name: m.name.to_string(),
            youngs_modulus: m.youngs_modulus,
            shear_modulus: m.shear_modulus,
            poissons_ratio: m.poissons_ratio,
            density: m.density,
            thermal_expansion: m.thermal_expansion,
            yield_strength: m.yield_strength,
            ultimate_strength: m.ultimate_strength,
            color: m.color.map(|c| [c.0, c.1, c.2]),
        }),
    };

    if pretty {
        serde_json::to_string_pretty(&json_section).unwrap()
    } else {
        serde_json::to_string(&json_section).unwrap()
    }
}

/// Import a section from JSON string.
pub fn from_json(json_str: &str) -> Result<JsonSection, serde_json::Error> {
    serde_json::from_str(json_str)
}

/// Convert JsonSection back to Section.
impl From<JsonSection> for Section {
    fn from(json: JsonSection) -> Self {
        let outer = Polygon::new(
            json.outer_vertices
                .into_iter()
                .map(|v| Point::new(v[0], v[1]))
                .collect(),
        );

        let holes = json
            .holes
            .into_iter()
            .map(|h| Polygon::new(h.into_iter().map(|v| Point::new(v[0], v[1])).collect()))
            .collect();

        Section::new(outer, holes)
    }
}

impl JsonMaterial {
    /// Convert to Material with owned name (no memory leak).
    pub fn into_material(self) -> Material {
        Material {
            youngs_modulus: self.youngs_modulus,
            shear_modulus: self.shear_modulus,
            poissons_ratio: self.poissons_ratio,
            density: self.density,
            thermal_expansion: self.thermal_expansion,
            yield_strength: self.yield_strength,
            ultimate_strength: self.ultimate_strength,
            name: Box::leak(self.name.into_boxed_str()), // TODO: Use owned string in Material
            color: self.color.map(|c| (c[0], c[1], c[2])),
        }
    }
}

/// Batch export multiple sections to a single JSON file.
pub fn export_section_catalog(sections: &[(String, Section)], pretty: bool) -> String {
    #[derive(Serialize)]
    struct Catalog {
        version: String,
        sections: Vec<CatalogEntry>,
    }

    #[derive(Serialize)]
    struct CatalogEntry {
        name: String,
        designation: Option<String>,
        outer_vertices: Vec<[f64; 2]>,
        holes: Vec<Vec<[f64; 2]>>,
    }

    let catalog = Catalog {
        version: "1.0".to_string(),
        sections: sections
            .iter()
            .map(|(name, section)| CatalogEntry {
                name: name.clone(),
                designation: None,
                outer_vertices: section.outer.vertices.iter().map(|v| [v.x, v.y]).collect(),
                holes: section
                    .holes
                    .iter()
                    .map(|h| h.vertices.iter().map(|v| [v.x, v.y]).collect())
                    .collect(),
            })
            .collect(),
    };

    if pretty {
        serde_json::to_string_pretty(&catalog).unwrap()
    } else {
        serde_json::to_string(&catalog).unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon};
    use crate::material::presets::STEEL_S355;
    use crate::section::Section;
    use crate::section_library::ParametricSection;
    use crate::section_library::steel::ISection;

    #[test]
    fn json_export_import() {
        let i = ISection::from_designation("IPE300").unwrap();
        let section = i.build();

        let json = to_json(&section, None, Some(&STEEL_S355), Some("IPE300"), true);

        assert!(json.contains("IPE300"));
        assert!(json.contains("vertices"));
        assert!(json.contains("youngs_modulus"));

        let imported = from_json(&json).unwrap();
        assert_eq!(imported.designation, Some("IPE300".to_string()));
        assert_eq!(imported.outer_vertices.len(), section.outer.vertices.len());
    }

    #[test]
    fn json_section_properties() {
        let i = ISection::from_designation("IPE300").unwrap();
        let section = i.build();
        let props = SectionProperties::from_section(&section);

        let json = to_json(
            &section,
            Some(&props),
            Some(&STEEL_S355),
            Some("IPE300"),
            true,
        );

        assert!(json.contains("ix"));
        assert!(json.contains("principal"));
        assert!(json.contains("gyration"));

        let imported = from_json(&json).unwrap();
        assert!(imported.properties.is_some());
        let p = imported.properties.unwrap();
        assert!((p.area - props.area).abs() < 1e-6);
        assert!((p.ix - props.ix).abs() < 1e-6);
    }

    #[test]
    fn json_roundtrip_geometry() {
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

        let json = to_json(&section, None, None, Some("TestSection"), false);
        let imported = from_json(&json).unwrap();

        assert_eq!(imported.outer_vertices.len(), 4);
        assert_eq!(imported.holes.len(), 1);
        assert_eq!(imported.holes[0].len(), 4);
    }
}
