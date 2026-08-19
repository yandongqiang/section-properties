//! Composite sections and transformed section method.
//!
//! Supports multi-material sections (steel-concrete, timber-concrete, etc.)

use crate::geometry::{Point, Polygon};
use crate::material::{Material, MaterialGroup};
use crate::section::Section;
use crate::section_library::{ParametricSection, rectangle_polygon};

/// Composite section with multiple materials using transformed section method.
#[derive(Debug, Clone)]
pub struct CompositeSection {
    pub outer: Polygon,
    pub holes: Vec<Polygon>,
    pub material_groups: Vec<MaterialGroup>,
    /// Reference material for transformation (usually first group)
    pub reference_material: Material,
}

impl CompositeSection {
    /// Create a composite section from a single-material Section.
    pub fn single_material(section: Section, material: Material) -> Self {
        let reference_material = material;
        Self {
            outer: section.outer,
            holes: section.holes,
            material_groups: vec![MaterialGroup::new(material, vec![0])],
            reference_material,
        }
    }

    /// Create from outer polygon, holes, and material groups.
    pub fn new(outer: Polygon, holes: Vec<Polygon>, material_groups: Vec<MaterialGroup>) -> Self {
        let reference_material = material_groups
            .first()
            .map(|g| g.material)
            .unwrap_or_else(|| Material::default());
        Self {
            outer,
            holes,
            material_groups,
            reference_material,
        }
    }

    /// Create from a parametric section with a single material.
    pub fn from_parametric<S: ParametricSection>(section: &S, material: Material) -> Self {
        Self::single_material(section.build(), material)
    }

    /// Create a composite section from multiple parametric sections with different materials.
    pub fn from_components(components: Vec<(Box<dyn ParametricSection>, Point, Material)>) -> Self {
        let mut all_polygons = Vec::new();
        let mut material_groups = Vec::new();
        let mut current_polygon_index = 0;

        for (section, offset, material) in components {
            let built = section.build();
            // Apply offset to outer
            let mut outer = built.outer;
            for v in &mut outer.vertices {
                v.x += offset.x;
                v.y += offset.y;
            }
            all_polygons.push(outer);

            // Apply offset to holes
            for mut hole in built.holes {
                for v in &mut hole.vertices {
                    v.x += offset.x;
                    v.y += offset.y;
                }
                all_polygons.push(hole);
            }

            let n_polygons = 1 + built.holes.len();
            material_groups.push(MaterialGroup::new(
                material,
                (current_polygon_index..current_polygon_index + n_polygons).collect(),
            ));
            current_polygon_index += n_polygons;
        }

        // For the composite section, we use the first polygon as outer
        // and the rest as holes (simplified - real impl would do boolean union)
        let outer = all_polygons[0].clone();
        let holes = all_polygons[1..].to_vec();

        Self::new(outer, holes, material_groups)
    }

    /// Get the reference material.
    pub fn reference_material(&self) -> &Material {
        &self.reference_material
    }

    /// Get modular ratios for all groups relative to reference material.
    pub fn modular_ratios(&self) -> Vec<f64> {
        self.material_groups
            .iter()
            .map(|g| g.material.modular_ratio(&self.reference_material))
            .collect()
    }

    /// Compute transformed section properties using the reference material.
    ///
    /// Uses the transformed section method:
    /// - Each material's area is multiplied by its modular ratio n = E_material / E_ref
    /// - Moments of inertia are computed about the transformed centroid
    pub fn transformed_properties(&self) -> crate::section_properties::SectionProperties {
        // For full transformed section analysis, we need to:
        // 1. Transform all areas by modular ratio
        // 2. Find transformed centroid
        // 3. Compute transformed moments of inertia

        // Simplified implementation: compute properties for each group separately
        // and combine using parallel axis theorem with transformed areas

        let mut total_area = 0.0;
        let mut first_moment_x = 0.0;
        let mut first_moment_y = 0.0;
        let mut ix = 0.0;
        let mut iy = 0.0;
        let mut ixy = 0.0;

        // Process outer polygon (index 0)
        if let Some(group) = self
            .material_groups
            .iter()
            .find(|g| g.polygon_indices.contains(&0))
        {
            let n = group.material.modular_ratio(&self.reference_material);
            let poly = &self.outer;
            let signed_area = poly.signed_area() * n;
            let centroid = poly.centroid();
            let i_x = poly.moment_of_inertia_x() * n;
            let i_y = poly.moment_of_inertia_y() * n;
            let i_xy = poly.product_of_inertia_xy() * n;

            total_area += signed_area;
            first_moment_x += signed_area * centroid.x;
            first_moment_y += signed_area * centroid.y;
            ix += i_x;
            iy += i_y;
            ixy += i_xy;
        }

        // Process holes
        for (i, hole) in self.holes.iter().enumerate() {
            let poly_index = i + 1;
            if let Some(group) = self
                .material_groups
                .iter()
                .find(|g| g.polygon_indices.contains(&poly_index))
            {
                let n = group.material.modular_ratio(&self.reference_material);
                // Holes have negative signed area (CW)
                let signed_area = hole.signed_area() * n; // already negative
                let centroid = hole.centroid();
                let i_x = hole.moment_of_inertia_x() * n;
                let i_y = hole.moment_of_inertia_y() * n;
                let i_xy = hole.product_of_inertia_xy() * n;

                total_area += signed_area;
                first_moment_x += signed_area * centroid.x;
                first_moment_y += signed_area * centroid.y;
                ix += i_x;
                iy += i_y;
                ixy += i_xy;
            }
        }

        assert!(
            total_area.abs() > f64::EPSILON,
            "Transformed section area too small"
        );

        // Transformed centroid
        let centroid = Point::new(first_moment_x / total_area, first_moment_y / total_area);

        // Parallel axis theorem to centroidal axes
        let ix_c = ix - total_area * centroid.y.powi(2);
        let iy_c = iy - total_area * centroid.x.powi(2);
        let ixy_c = ixy - total_area * centroid.x * centroid.y;

        crate::section_properties::SectionProperties {
            area: total_area,
            centroid,
            ix: ix_c,
            iy: iy_c,
            ixy: ixy_c,
        }
    }

    /// Compute section properties for a specific material group only.
    pub fn material_group_properties(
        &self,
        group_index: usize,
    ) -> Option<crate::section_properties::SectionProperties> {
        if group_index >= self.material_groups.len() {
            return None;
        }
        let group = &self.material_groups[group_index];

        let mut area = 0.0;
        let mut fx = 0.0;
        let mut fy = 0.0;
        let mut ix = 0.0;
        let mut iy = 0.0;
        let mut ixy = 0.0;

        for &poly_idx in &group.polygon_indices {
            let poly = if poly_idx == 0 {
                &self.outer
            } else if poly_idx > 0 && poly_idx <= self.holes.len() {
                &self.holes[poly_idx - 1]
            } else {
                continue;
            };

            let signed_area = poly.signed_area();
            let centroid = poly.centroid();

            area += signed_area;
            fx += signed_area * centroid.x;
            fy += signed_area * centroid.y;
            ix += poly.moment_of_inertia_x();
            iy += poly.moment_of_inertia_y();
            ixy += poly.product_of_inertia_xy();
        }

        if area.abs() < f64::EPSILON {
            return None;
        }

        let centroid = Point::new(fx / area, fy / area);
        let ix_c = ix - area * centroid.y.powi(2);
        let iy_c = iy - area * centroid.x.powi(2);
        let ixy_c = ixy - area * centroid.x * centroid.y;

        Some(crate::section_properties::SectionProperties {
            area,
            centroid,
            ix: ix_c,
            iy: iy_c,
            ixy: ixy_c,
        })
    }

    /// Get the number of material groups.
    pub fn n_groups(&self) -> usize {
        self.material_groups.len()
    }

    /// Get material group by index.
    pub fn group(&self, index: usize) -> Option<&MaterialGroup> {
        self.material_groups.get(index)
    }

    /// Add a material group to the section.
    pub fn add_group(&mut self, group: MaterialGroup) {
        self.material_groups.push(group);
    }
}

/// Steel-concrete composite beam section (EN 1994).
#[derive(Debug, Clone)]
pub struct SteelConcreteComposite {
    pub steel_section: Box<dyn ParametricSection>,
    pub concrete_slab: CompositeSlab,
    pub shear_connectors: ShearConnectorLayout,
    pub construction_stage: ConstructionStage,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CompositeSlab {
    pub width: f64,          // Effective width (beff)
    pub thickness: f64,      // Total slab thickness
    pub deck_thickness: f64, // Metal deck thickness (if any)
    pub deck_orientation: DeckOrientation,
    pub concrete_grade: ConcreteGrade,
    pub reinforcement: Vec<RebarLayer>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeckOrientation {
    Parallel,      // Deck ribs parallel to beam
    Perpendicular, // Deck ribs perpendicular to beam
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConcreteGrade {
    C20_25,
    C25_30,
    C30_37,
    C35_45,
    C40_50,
    C45_55,
    C50_60,
}

impl ConcreteGrade {
    pub fn fck(&self) -> f64 {
        match self {
            ConcreteGrade::C20_25 => 20e6,
            ConcreteGrade::C25_30 => 25e6,
            ConcreteGrade::C30_37 => 30e6,
            ConcreteGrade::C35_45 => 35e6,
            ConcreteGrade::C40_50 => 40e6,
            ConcreteGrade::C45_55 => 45e6,
            ConcreteGrade::C50_60 => 50e6,
        }
    }

    pub fn fcm(&self) -> f64 {
        self.fck() + 8e6
    }

    pub fn ecm(&self) -> f64 {
        // EN 1992-1-1 Eq 3.1
        22e3 * (self.fcm() / 10e6).powf(0.3) * 1e6 // Pa
    }

    pub fn material(&self) -> Material {
        use crate::material::Material;
        Material::with_all(
            self.ecm(),
            self.ecm() / (2.0 * (1.0 + 0.2)),
            0.2,
            2400.0,
            10e-6,
            self.fck(),
            self.fcm(),
            &format!("Concrete {:?}", self),
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RebarLayer {
    pub y: f64, // Distance from top of slab (positive down)
    pub diameter: f64,
    pub spacing: f64,
    pub count_per_m: usize, // Bars per meter width
    pub material: Material,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShearConnectorLayout {
    pub type_: ShearConnectorType,
    pub diameter: f64,
    pub height: f64,
    pub spacing: f64,     // Longitudinal spacing
    pub n_per_row: usize, // Number per cross-section
    pub material: Material,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShearConnectorType {
    HeadedStud,
    Channel,
    Perfobond,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConstructionStage {
    /// Bare steel (construction stage)
    BareSteel,
    /// Steel + wet concrete (construction stage)
    SteelWetConcrete,
    /// Composite (final stage, full interaction)
    Composite,
    /// Composite with partial interaction
    PartialInteraction,
}

impl SteelConcreteComposite {
    pub fn new(
        steel_section: impl ParametricSection + 'static,
        concrete_slab: CompositeSlab,
        shear_connectors: ShearConnectorLayout,
    ) -> Self {
        Self {
            steel_section: Box::new(steel_section),
            concrete_slab,
            shear_connectors,
            construction_stage: ConstructionStage::Composite,
        }
    }

    pub fn with_stage(mut self, stage: ConstructionStage) -> Self {
        self.construction_stage = stage;
        self
    }

    /// Effective width of concrete slab per EN 1994-1-1
    pub fn effective_width(&self, span: f64, beam_spacing: f64) -> f64 {
        let beff = match self.concrete_slab.deck_orientation {
            DeckOrientation::Parallel => {
                // EN 1994-1-1 Cl 6.6.1.2
                let b0 = span / 8.0;
                (b0.min(beam_spacing)).min(self.concrete_slab.width)
            }
            DeckOrientation::Perpendicular => {
                // EN 1994-1-1 Cl 6.6.1.3
                let b0 = span / 8.0;
                (b0.min(beam_spacing / 2.0)).min(self.concrete_slab.width)
            }
        };
        beff
    }

    /// Modular ratio n = E_steel / E_concrete (long-term)
    pub fn modular_ratio(&self, long_term: bool) -> f64 {
        let steel_mat = crate::material::presets::STEEL_S355;
        let conc_mat = self.concrete_slab.concrete_grade.material();
        let n = steel_mat.modular_ratio(&conc_mat);
        if long_term {
            // Creep effect: n_long = n * (1 + phi) where phi is creep coefficient
            // Simplified: use n * 2 for long-term
            n * 2.0
        } else {
            n
        }
    }

    /// Build the composite section for a given stage.
    pub fn build_section(&self, span: f64, beam_spacing: f64) -> CompositeSection {
        let beff = self.effective_width(span, beam_spacing);
        let n = self.modular_ratio(matches!(
            self.construction_stage,
            ConstructionStage::Composite
        ));

        let steel_sec = self.steel_section.build();
        let steel_mat = crate::material::presets::STEEL_S355;
        let conc_mat = self.concrete_slab.concrete_grade.material();

        // Position concrete slab on top of steel section
        let steel_props = crate::section_properties::SectionProperties::from_section(&steel_sec);
        let steel_top_y = steel_props.centroid.y
            + steel_sec
                .outer
                .vertices
                .iter()
                .map(|v| v.y)
                .max_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap_or(0.0);

        // Concrete slab centroid relative to steel centroid
        let slab_y = steel_top_y + self.concrete_slab.thickness / 2.0;
        let slab_offset = Point::new(0.0, slab_y - steel_props.centroid.y);

        // Build composite section
        CompositeSection::from_components(vec![
            (
                Box::new(self.steel_section.build()),
                Point::new(0.0, 0.0),
                steel_mat,
            ),
            (
                Box::new(RectangularSection {
                    width: beff,
                    height: self.concrete_slab.thickness,
                }),
                slab_offset,
                conc_mat,
            ),
        ])
    }

    /// Compute transformed section properties for the composite stage.
    pub fn composite_properties(
        &self,
        span: f64,
        beam_spacing: f64,
    ) -> crate::section_properties::SectionProperties {
        let comp = self.build_section(span, beam_spacing);
        comp.transformed_properties()
    }

    /// Moment capacity (simplified plastic design per EN 1994).
    pub fn plastic_moment_capacity(&self, span: f64, beam_spacing: f64) -> f64 {
        // Simplified - full implementation would do plastic stress block analysis
        let props = self.composite_properties(span, beam_spacing);
        let steel_mat = crate::material::presets::STEEL_S355;
        let fy = steel_mat.yield_strength;
        // Mpl = fy * Wel (elastic) or fy * Wpl (plastic)
        // Simplified using elastic modulus
        props.ix.max(props.iy) / (self.height() / 2.0) * fy
    }

    fn height(&self) -> f64 {
        let steel_sec = self.steel_section.build();
        let steel_props = crate::section_properties::SectionProperties::from_section(&steel_sec);
        steel_sec
            .outer
            .vertices
            .iter()
            .map(|v| v.y)
            .max_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(0.0)
            - steel_sec
                .outer
                .vertices
                .iter()
                .map(|v| v.y)
                .min_by(|a, b| a.partial_cmp(b).unwrap())
                .unwrap_or(0.0)
            + self.concrete_slab.thickness
    }
}

/// Simple rectangular section for composite slab.
#[derive(Debug, Clone, Copy, PartialEq)]
struct RectangularSection {
    width: f64,
    height: f64,
}

impl ParametricSection for RectangularSection {
    fn build(&self) -> Section {
        Section::new(rectangle_polygon(self.width, self.height), Vec::new())
    }

    fn designation(&self) -> String {
        format!(
            "RECT {:.0}x{:.0}",
            self.width * 1000.0,
            self.height * 1000.0
        )
    }
}

/// Timber-concrete composite (TCC) section.
#[derive(Debug, Clone)]
pub struct TimberConcreteComposite {
    pub timber_section: Box<dyn ParametricSection>,
    pub concrete_slab: CompositeSlab,
    pub connection: TCCConnection,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TCCConnection {
    pub type_: TCCConnectorType,
    pub stiffness: f64, // k [N/mm] per connector
    pub spacing: f64,   // Longitudinal spacing [m]
    pub n_per_row: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TCCConnectorType {
    Screw,
    Nail,
    Dowel,
    Notch,
    Glue,
    ShearPlate,
}

impl TimberConcreteComposite {
    pub fn new(
        timber_section: impl ParametricSection + 'static,
        concrete_slab: CompositeSlab,
        connection: TCCConnection,
    ) -> Self {
        Self {
            timber_section: Box::new(timber_section),
            concrete_slab,
            connection,
        }
    }

    /// Connection stiffness parameter gamma (EN 1995-1-1 Annex B)
    pub fn gamma(&self, span: f64) -> f64 {
        // Simplified gamma calculation for TCC
        let k = self.connection.stiffness * 1000.0; // N/m
        let s = self.connection.spacing;
        let n = self.connection.n_per_row as f64;
        let k_ser = k * n / s; // Stiffness per meter [N/m/m]

        // Timber and concrete properties
        let timber_mat = crate::material::presets::TIMBER_GL24H;
        let conc_mat = self.concrete_slab.concrete_grade.material();

        // Get section properties
        let timber_sec = self.timber_section.build();
        let timber_props = crate::section_properties::SectionProperties::from_section(&timber_sec);

        // Approximate effective width
        let beff = self.concrete_slab.width;
        let tc = self.concrete_slab.thickness;

        // Gamma = 1 / (1 + (pi^2 * EI_eff) / (k_ser * span^2))
        // Simplified
        let ei_timber = timber_mat.youngs_modulus * timber_props.ix.max(timber_props.iy);
        let ei_concrete = conc_mat.youngs_modulus * beff * tc.powi(3) / 12.0;
        let ei_eff = ei_timber + ei_concrete;

        let gamma = 1.0 / (1.0 + (PI * PI * ei_eff) / (k_ser * span * span));
        gamma.clamp(0.0, 1.0)
    }
}

/// Sandwich panel (face sheets + core).
#[derive(Debug, Clone)]
pub struct SandwichPanel {
    pub face_sheet: Box<dyn ParametricSection>,
    pub core: Box<dyn ParametricSection>,
    pub face_material: Material,
    pub core_material: Material,
}

impl SandwichPanel {
    pub fn new(
        face_sheet: impl ParametricSection + 'static,
        core: impl ParametricSection + 'static,
        face_material: Material,
        core_material: Material,
    ) -> Self {
        Self {
            face_sheet: Box::new(face_sheet),
            core: Box::new(core),
            face_material,
            core_material,
        }
    }

    pub fn build(&self) -> CompositeSection {
        let face_sec = self.face_sheet.build();
        let core_sec = self.core.build();

        let face_props = crate::section_properties::SectionProperties::from_section(&face_sec);
        let core_props = crate::section_properties::SectionProperties::from_section(&core_sec);

        // Face sheets are typically at top and bottom of core
        let face_offset = Point::new(0.0, core_props.centroid.y + face_props.centroid.y);

        CompositeSection::from_components(vec![
            (
                Box::new(self.face_sheet.build()),
                Point::new(0.0, face_offset.y),
                self.face_material,
            ),
            (
                Box::new(self.core.build()),
                Point::new(0.0, 0.0),
                self.core_material,
            ),
            (
                Box::new(self.face_sheet.build()),
                Point::new(0.0, -face_offset.y),
                self.face_material,
            ),
        ])
    }

    pub fn transformed_properties(&self) -> crate::section_properties::SectionProperties {
        self.build().transformed_properties()
    }

    /// Sandwich panel bending stiffness (D) per unit width.
    pub fn bending_stiffness_d(&self, width: f64) -> f64 {
        // D = (Ef * tf * h^2) / 2 + (Ec * tc^3) / 12
        // Simplified - use transformed properties
        let props = self.transformed_properties();
        self.face_material.youngs_modulus * props.ix * width
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::presets::*;
    use crate::section_library::steel::ISection;

    #[test]
    fn composite_single_material() {
        let rect = RectangularConcreteSection::new(0.3, 0.5);
        let comp = CompositeSection::from_parametric(&rect, CONCRETE_C30_37);
        let props = comp.transformed_properties();
        assert!((props.area - 0.15).abs() < 1e-6);
    }

    #[test]
    fn composite_steel_concrete() {
        let steel = ISection::from_designation("IPE300").expect("IPE300");
        let slab = CompositeSlab {
            width: 1.0,
            thickness: 0.15,
            deck_thickness: 0.0,
            deck_orientation: DeckOrientation::Perpendicular,
            concrete_grade: ConcreteGrade::C30_37,
            reinforcement: vec![],
        };
        let connectors = ShearConnectorLayout {
            type_: ShearConnectorType::HeadedStud,
            diameter: 0.019,
            height: 0.1,
            spacing: 0.3,
            n_per_row: 2,
            material: STEEL_S355,
        };
        let composite = SteelConcreteComposite::new(steel, slab, connectors)
            .with_stage(ConstructionStage::Composite);

        let props = composite.composite_properties(6.0, 3.0);
        assert!(props.area > 0.0);
        assert!(props.ix > 0.0);
    }

    #[test]
    fn concrete_grade_properties() {
        let c30 = ConcreteGrade::C30_37;
        assert!((c30.fck() - 30e6).abs() < 1e-3);
        assert!((c30.ecm() - 33e9).abs() < 1e9); // approximate
    }

    #[test]
    fn modular_ratio_steel_concrete() {
        let steel = ISection::from_designation("IPE300").expect("IPE300");
        let slab = CompositeSlab {
            width: 1.0,
            thickness: 0.15,
            deck_thickness: 0.0,
            deck_orientation: DeckOrientation::Perpendicular,
            concrete_grade: ConcreteGrade::C30_37,
            reinforcement: vec![],
        };
        let connectors = ShearConnectorLayout {
            type_: ShearConnectorType::HeadedStud,
            diameter: 0.019,
            height: 0.1,
            spacing: 0.3,
            n_per_row: 2,
            material: STEEL_S355,
        };
        let composite = SteelConcreteComposite::new(steel, slab, connectors);
        let n_short = composite.modular_ratio(false);
        let n_long = composite.modular_ratio(true);
        assert!(n_short > 6.0 && n_short < 7.0); // ~200/33 ≈ 6.06
        assert!((n_long / n_short - 2.0).abs() < 0.1);
    }

    #[test]
    fn sandwich_panel() {
        use crate::section_library::primitive::RectangularSection as PrimRect;
        let face = PrimRect::new(1.0, 0.002);
        let core = PrimRect::new(1.0, 0.1);
        let panel = SandwichPanel::new(
            face,
            core,
            STEEL_S355,
            crate::material::presets::ALUMINUM_6061_T6,
        );
        let props = panel.transformed_properties();
        assert!(props.area > 0.0);
        assert!(props.ix > 0.0);
    }
}

