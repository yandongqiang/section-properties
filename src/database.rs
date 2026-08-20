//! Section database for managing and querying section libraries.
//!
//! Provides in-memory database with indexing, filtering, and search capabilities
//! similar to Python's sectionproperties.database.

use crate::material::Material;
use crate::section::Section;
use crate::section_library::{CompositeSection, ParametricSection};
use crate::section_properties::SectionProperties;

/// High-level section entry with metadata.
#[derive(Debug, Clone)]
pub struct SectionEntry {
    pub name: String,
    pub designation: String,
    pub section: Section,
    pub parametric: Option<Box<dyn ParametricSection>>,
    pub material: Option<Material>,
    pub properties: Option<SectionProperties>,
    pub tags: Vec<String>,
    pub source: String,
    pub metadata: std::collections::HashMap<String, String>,
}

impl SectionEntry {
    pub fn new<S: ParametricSection + 'static>(parametric: S, material: Option<Material>) -> Self {
        let designation = parametric.designation();
        let section = parametric.build();
        let properties = SectionProperties::from_section(&section);

        Self {
            name: designation.clone(),
            designation,
            section,
            parametric: Some(Box::new(parametric)),
            material,
            properties: Some(properties),
            tags: Vec::new(),
            source: "parametric".to_string(),
            metadata: std::collections::HashMap::new(),
        }
    }

    pub fn from_section(section: Section, name: String, source: String) -> Self {
        let properties = SectionProperties::from_section(&section);
        Self {
            name: name.clone(),
            designation: name,
            section,
            parametric: None,
            material: None,
            properties: Some(properties),
            tags: Vec::new(),
            source,
            metadata: std::collections::HashMap::new(),
        }
    }

    pub fn with_tags(mut self, tags: Vec<String>) -> Self {
        self.tags = tags;
        self
    }

    pub fn with_metadata(mut self, key: String, value: String) -> Self {
        self.metadata.insert(key, value);
        self
    }

    pub fn area(&self) -> f64 {
        self.properties.as_ref().map(|p| p.area).unwrap_or(0.0)
    }

    pub fn ix(&self) -> f64 {
        self.properties.as_ref().map(|p| p.ix).unwrap_or(0.0)
    }

    pub fn iy(&self) -> f64 {
        self.properties.as_ref().map(|p| p.iy).unwrap_or(0.0)
    }

    pub fn weight_per_meter(&self, density: f64) -> f64 {
        self.area() * density
    }
}

/// Search filter for database queries.
#[derive(Debug, Clone, Default)]
pub struct SearchFilter {
    pub name_contains: Option<String>,
    pub designation_contains: Option<String>,
    pub min_area: Option<f64>,
    pub max_area: Option<f64>,
    pub min_ix: Option<f64>,
    pub max_ix: Option<f64>,
    pub min_iy: Option<f64>,
    pub max_iy: Option<f64>,
    pub min_depth: Option<f64>,
    pub max_depth: Option<f64>,
    pub min_width: Option<f64>,
    pub max_width: Option<f64>,
    pub section_type: Option<String>,
    pub material: Option<String>,
    pub tags: Vec<String>,
    // pub class: Option<crate::plastic::SectionClass>,
    pub shape_factor_min: Option<f64>,
    pub shape_factor_max: Option<f64>,
}

/// Search result with score.
#[derive(Debug, Clone)]
pub struct SearchResult {
    pub entry: SectionEntry,
    pub score: f64,
    pub matched_fields: Vec<String>,
}

/// In-memory section database.
#[derive(Debug, Default)]
pub struct SectionDatabase {
    entries: Vec<SectionEntry>,
    name_index: std::collections::HashMap<String, usize>,
    designation_index: std::collections::HashMap<String, usize>,
    tag_index: std::collections::HashMap<String, Vec<usize>>,
}

impl SectionDatabase {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a section entry to the database.
    pub fn add(&mut self, entry: SectionEntry) {
        let idx = self.entries.len();
        let name = entry.name.clone();
        let designation = entry.designation.clone();
        let tags = entry.tags.clone();

        self.name_index.insert(name, idx);
        self.designation_index.insert(designation, idx);

        for tag in tags {
            self.tag_index.entry(tag).or_default().push(idx);
        }

        self.entries.push(entry);
    }

    /// Add multiple entries.
    pub fn add_many(&mut self, entries: Vec<SectionEntry>) {
        for entry in entries {
            self.add(entry);
        }
    }

    /// Get entry by name.
    pub fn get_by_name(&self, name: &str) -> Option<&SectionEntry> {
        self.name_index.get(name).map(|&i| &self.entries[i])
    }

    /// Get entry by designation.
    pub fn get_by_designation(&self, designation: &str) -> Option<&SectionEntry> {
        self.designation_index
            .get(designation)
            .map(|&i| &self.entries[i])
    }

    /// Get entries by tag.
    pub fn get_by_tag(&self, tag: &str) -> Vec<&SectionEntry> {
        self.tag_index
            .get(tag)
            .map(|indices| indices.iter().map(|&i| &self.entries[i]).collect())
            .unwrap_or_default()
    }

    /// Search with filter.
    pub fn search(&self, filter: &SearchFilter) -> Vec<SearchResult> {
        let mut results = Vec::new();

        for entry in &self.entries {
            let mut score = 0.0;
            let mut matched = Vec::new();

            // Name/designation matching
            if let Some(ref pattern) = filter.name_contains {
                if entry.name.to_lowercase().contains(&pattern.to_lowercase()) {
                    score += 10.0;
                    matched.push("name".to_string());
                }
            }
            if let Some(ref pattern) = filter.designation_contains {
                if entry
                    .designation
                    .to_lowercase()
                    .contains(&pattern.to_lowercase())
                {
                    score += 10.0;
                    matched.push("designation".to_string());
                }
            }

            // Property ranges
            if let Some(props) = &entry.properties {
                if let Some(min) = filter.min_area {
                    if props.area >= min {
                        score += 1.0;
                        matched.push("area_min".to_string());
                    }
                }
                if let Some(max) = filter.max_area {
                    if props.area <= max {
                        score += 1.0;
                        matched.push("area_max".to_string());
                    }
                }
                if let Some(min) = filter.min_ix {
                    if props.ix >= min {
                        score += 1.0;
                        matched.push("ix_min".to_string());
                    }
                }
                if let Some(max) = filter.max_ix {
                    if props.ix <= max {
                        score += 1.0;
                        matched.push("ix_max".to_string());
                    }
                }
                if let Some(min) = filter.min_iy {
                    if props.iy >= min {
                        score += 1.0;
                        matched.push("iy_min".to_string());
                    }
                }
                if let Some(max) = filter.max_iy {
                    if props.iy <= max {
                        score += 1.0;
                        matched.push("iy_max".to_string());
                    }
                }
            }

            // Section type
            if let Some(ref typ) = filter.section_type {
                if entry.tags.iter().any(|t| t.contains(typ)) {
                    score += 5.0;
                    matched.push("type".to_string());
                }
            }

            // Material
            if let Some(ref mat) = filter.material {
                if entry
                    .material
                    .as_ref()
                    .map(|m| m.name.contains(mat))
                    .unwrap_or(false)
                {
                    score += 5.0;
                    matched.push("material".to_string());
                }
            }

            // Tags
            for tag in &filter.tags {
                if entry.tags.contains(tag) {
                    score += 2.0;
                    matched.push(format!("tag:{}", tag));
                }
            }

            // Bounds check
            let bounds = entry.section.bounds();
            let depth = bounds.1 - bounds.0;
            let width = bounds.3 - bounds.2;

            if let Some(min) = filter.min_depth {
                if depth >= min {
                    score += 1.0;
                }
            }
            if let Some(max) = filter.max_depth {
                if depth <= max {
                    score += 1.0;
                }
            }
            if let Some(min) = filter.min_width {
                if width >= min {
                    score += 1.0;
                }
            }
            if let Some(max) = filter.max_width {
                if width <= max {
                    score += 1.0;
                }
            }

            if score > 0.0 {
                results.push(SearchResult {
                    entry: entry.clone(),
                    score,
                    matched_fields: matched,
                });
            }
        }

        // Sort by score descending
        results.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());
        results
    }

    /// Find sections similar to a reference section.
    pub fn find_similar(&self, reference: &Section, tolerance: f64) -> Vec<SearchResult> {
        let ref_props = crate::section_properties::SectionProperties::from_section(reference);
        let ref_area = ref_props.area;
        let ref_ix = ref_props.ix;
        let ref_iy = ref_props.iy;

        let mut filter = SearchFilter::default();
        filter.min_area = Some(ref_area * (1.0 - tolerance));
        filter.max_area = Some(ref_area * (1.0 + tolerance));
        filter.min_ix = Some(ref_ix * (1.0 - tolerance));
        filter.max_ix = Some(ref_ix * (1.0 + tolerance));
        filter.min_iy = Some(ref_iy * (1.0 - tolerance));
        filter.max_iy = Some(ref_iy * (1.0 + tolerance));

        self.search(&filter)
    }

    /// Get all entries.
    pub fn all(&self) -> &[SectionEntry] {
        &self.entries
    }

    /// Get count.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Check if empty.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Clear database.
    pub fn clear(&mut self) {
        self.entries.clear();
        self.name_index.clear();
        self.designation_index.clear();
        self.tag_index.clear();
    }

    /// Export to JSON.
    pub fn to_json(&self, pretty: bool) -> String {
        #[derive(serde::Serialize)]
        struct DbExport {
            version: String,
            count: usize,
            entries: Vec<DbEntry>,
        }

        #[derive(serde::Serialize)]
        struct DbEntry {
            name: String,
            designation: String,
            area: f64,
            ix: f64,
            iy: f64,
            tags: Vec<String>,
            source: String,
        }

        let export = DbExport {
            version: "1.0".to_string(),
            count: self.entries.len(),
            entries: self
                .entries
                .iter()
                .map(|e| DbEntry {
                    name: e.name.clone(),
                    designation: e.designation.clone(),
                    area: e.area(),
                    ix: e.ix(),
                    iy: e.iy(),
                    tags: e.tags.clone(),
                    source: e.source.clone(),
                })
                .collect(),
        };

        if pretty {
            serde_json::to_string_pretty(&export).unwrap()
        } else {
            serde_json::to_string(&export).unwrap()
        }
    }
}

/// Build a standard database with common sections.
pub fn build_standard_database() -> SectionDatabase {
    let mut db = SectionDatabase::new();

    // Add standard steel sections
    use crate::material::presets::STEEL_S355;
    use crate::section_library::steel::*;

    // I-sections
    for des in [
        "IPE160", "IPE200", "IPE240", "IPE300", "IPE360", "IPE400", "IPE450", "IPE500", "IPE600",
    ] {
        if let Some(i) = ISection::from_designation(des) {
            let entry = SectionEntry::new(i, Some(STEEL_S355)).with_tags(vec![
                "steel".to_string(),
                "i-section".to_string(),
                "hot-rolled".to_string(),
            ]);
            db.add(entry);
        }
    }

    // HEA/HEB
    for des in ["HEA200", "HEA300", "HEB200", "HEB300", "HEB400"] {
        if let Some(i) = ISection::from_designation(des) {
            let entry = SectionEntry::new(i, Some(STEEL_S355)).with_tags(vec![
                "steel".to_string(),
                "h-section".to_string(),
                "hot-rolled".to_string(),
            ]);
            db.add(entry);
        }
    }

    // Channels
    for des in ["UPN100", "UPN160", "UPN200", "UPN240", "UPN300"] {
        if let Some(c) = crate::section_library::steel::ChannelSection::from_designation(des) {
            let entry = SectionEntry::new(c, Some(STEEL_S355)).with_tags(vec![
                "steel".to_string(),
                "channel".to_string(),
                "hot-rolled".to_string(),
            ]);
            db.add(entry);
        }
    }

    // Angles
    for des in ["L50X50X5", "L75X75X8", "L100X100X10", "L150X150X12"] {
        if let Some(a) = crate::section_library::steel::AngleSection::from_designation(des) {
            let entry = SectionEntry::new(a, Some(STEEL_S355)).with_tags(vec![
                "steel".to_string(),
                "angle".to_string(),
                "hot-rolled".to_string(),
            ]);
            db.add(entry);
        }
    }

    // Hollow sections
    for des in [
        "SHS100X5",
        "SHS150X8",
        "SHS200X8",
        "SHS250X10",
        "RHS200X100X8",
        "RHS300X200X10",
    ] {
        if let Some(h) =
            crate::section_library::steel::RectangularHollowSection::from_designation(des)
        {
            let entry = SectionEntry::new(h, Some(STEEL_S355)).with_tags(vec![
                "steel".to_string(),
                "hollow".to_string(),
                "cold-formed".to_string(),
            ]);
            db.add(entry);
        }
    }

    for des in ["CHS60X3", "CHS89X4", "CHS114X5", "CHS168X6", "CHS219X8"] {
        if let Some(h) =
            crate::section_library::steel::CircularHollowSectionLib::from_designation(des)
        {
            let entry = SectionEntry::new(h, Some(STEEL_S355)).with_tags(vec![
                "steel".to_string(),
                "chs".to_string(),
                "cold-formed".to_string(),
            ]);
            db.add(entry);
        }
    }

    db
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon};
    use crate::material::presets::STEEL_S355;
    use crate::section::Section;
    use crate::section_library::steel::ISection;

    #[test]
    fn database_basic() {
        let mut db = SectionDatabase::new();

        let i = ISection::from_designation("IPE300").unwrap();
        let entry = SectionEntry::new(i, Some(STEEL_S355))
            .with_tags(vec!["steel".to_string(), "i-section".to_string()]);

        db.add(entry);

        assert_eq!(db.len(), 1);
        assert!(db.get_by_name("IPE300").is_some());
        assert!(db.get_by_designation("IPE300").is_some());
    }

    #[test]
    fn database_search() {
        let mut db = SectionDatabase::new();

        let i1 = ISection::from_designation("IPE300").unwrap();
        let i2 = ISection::from_designation("HEB200").unwrap();

        db.add(SectionEntry::new(i1, Some(STEEL_S355)).with_tags(vec!["steel".to_string()]));
        db.add(SectionEntry::new(i2, Some(STEEL_S355)).with_tags(vec!["steel".to_string()]));

        let mut filter = SearchFilter::default();
        filter.name_contains = Some("IPE".to_string());

        let results = db.search(&filter);
        assert_eq!(results.len(), 1);
        assert!(results[0].entry.name.contains("IPE"));
    }

    #[test]
    fn database_property_filter() {
        let mut db = build_standard_database();

        let mut filter = SearchFilter::default();
        filter.min_area = Some(5e-3); // 5000 mm²
        filter.max_area = Some(15e-3); // 15000 mm²

        let results = db.search(&filter);
        assert!(!results.is_empty());

        for r in &results {
            assert!(r.entry.area() >= 5e-3);
            assert!(r.entry.area() <= 15e-3);
        }
    }

    #[test]
    fn database_find_similar() {
        let db = build_standard_database();

        let i = ISection::from_designation("IPE300").unwrap();
        let section = i.build();

        let similar = db.find_similar(&section, 0.2); // ±20%

        assert!(!similar.is_empty());
        // Should find IPE300 itself and similar sizes
    }

    #[test]
    fn standard_database() {
        let db = build_standard_database();
        assert!(db.len() > 20);

        // Check variety
        let tags: std::collections::HashSet<_> =
            db.all().flat_map(|e| e.tags.iter().cloned()).collect();

        assert!(tags.contains("i-section"));
        assert!(tags.contains("channel"));
        assert!(tags.contains("angle"));
        assert!(tags.contains("hollow"));
    }
}
