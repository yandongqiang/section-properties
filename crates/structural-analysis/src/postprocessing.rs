//! Post-processing data: envelopes and derived results.
//!
//! This module aggregates already-computed [`FrameAnalysisResult`] data. It
//! never re-assembles a stiffness matrix and never mutates analysis state.
//!
//! # Layer
//!
//! ```text
//! Analysis Result → Post-processing Data → Visualization
//! ```

use crate::FemError;
use crate::beam_fem::{Dof, SectionForces};
use crate::frame::{FrameAnalysisResult, MemberHandle};
use crate::load::LoadSource;
use crate::truss3d::TrussAnalysisResult3D;

/// Governing-source indices for the three section-force components. Every
/// index refers to [`Envelope::sources`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SectionForceSources {
    /// Source index governing `N`.
    pub axial: Option<usize>,
    /// Source index governing `V`.
    pub shear: Option<usize>,
    /// Source index governing `M`.
    pub moment: Option<usize>,
}

impl SectionForceSources {
    fn none() -> Self {
        Self {
            axial: None,
            shear: None,
            moment: None,
        }
    }
}

/// One sampled point of a force envelope across multiple load cases.
///
/// `min` and `max` track the minimum and maximum of each section-force
/// component independently. They are not absolute-value envelopes. Ties keep
/// the first input result; each component records its own source index.
///
/// # Sampling limitation
///
/// Values are evaluated only at the requested `xi` locations. Interior extrema
/// between samples can be missed; increase the envelope sample count when
/// distributed loads can peak between them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnvelopeSample {
    /// Member index in model order.
    pub member_index: usize,
    /// Member-local normalized coordinate in `[0, 1]`.
    pub xi: f64,
    /// Minimum section forces observed at this point.
    pub min: SectionForces,
    /// Maximum section forces observed at this point.
    pub max: SectionForces,
    /// Source indices governing the three `min` components.
    pub min_sources: SectionForceSources,
    /// Source indices governing the three `max` components.
    pub max_sources: SectionForceSources,
}

/// Signed min/max of one scalar quantity with governing source indices.
///
/// Ties keep the first input result. An unpopulated reaction DOF keeps the
/// sentinels `min = +∞`, `max = -∞` and `None` sources; use
/// [`is_populated`](Self::is_populated) before reading the values.
#[derive(Debug, Clone, PartialEq)]
pub struct Extremum {
    /// Minimum value observed.
    pub min: f64,
    /// Index into [`Envelope::sources`] governing `min`.
    pub min_source: Option<usize>,
    /// Maximum value observed.
    pub max: f64,
    /// Index into [`Envelope::sources`] governing `max`.
    pub max_source: Option<usize>,
}

impl Extremum {
    fn new() -> Self {
        Self {
            min: f64::INFINITY,
            min_source: None,
            max: f64::NEG_INFINITY,
            max_source: None,
        }
    }

    fn update(&mut self, value: f64, source: usize) {
        if value < self.min {
            self.min = value;
            self.min_source = Some(source);
        }
        if value > self.max {
            self.max = value;
            self.max_source = Some(source);
        }
    }

    /// `true` when at least one result contributed to this DOF.
    pub fn is_populated(&self) -> bool {
        self.min_source.is_some() || self.max_source.is_some()
    }
}

/// Per-node displacement envelope across multiple load cases.
#[derive(Debug, Clone, PartialEq)]
pub struct NodeDisplacementSample {
    /// Node index in model order.
    pub node_index: usize,
    /// Horizontal displacement envelope.
    pub ux: Extremum,
    /// Vertical displacement envelope.
    pub uy: Extremum,
    /// Rotation envelope.
    pub rz: Extremum,
}

/// Per-node support reaction envelope across multiple load cases.
///
/// Only DOFs that carry a physical support reaction are populated. Free-DOF
/// round-off residuals are never sampled. For an oblique roller, both global
/// translational components are populated because they are the global
/// components of the single resultant along the constraint direction.
#[derive(Debug, Clone, PartialEq)]
pub struct NodeReactionSample {
    /// Node index in model order.
    pub node_index: usize,
    /// Horizontal reaction envelope.
    pub rx: Extremum,
    /// Vertical reaction envelope.
    pub ry: Extremum,
    /// Moment reaction envelope.
    pub mz: Extremum,
}

/// Multi-case envelope over frame results.
///
/// The envelope owns a snapshot of every input result's typed [`LoadSource`].
/// Every governing-source field is an index into [`sources`](Self::sources),
/// so duplicate case names remain unambiguous.
///
/// Displacements at a DOF prescribed by a load case are an echo of that
/// boundary condition, not a computed serviceability response. The governing
/// source's `has_prescribed_displacements` flag makes that distinction
/// available to downstream consumers.
#[derive(Debug, Clone)]
pub struct Envelope {
    /// One sample per `(member, xi)` in member-major order.
    pub samples: Vec<EnvelopeSample>,
    /// Number of results that contributed, in input order.
    pub n_results: usize,
    /// Number of members in the model.
    pub n_members: usize,
    /// Samples per member, including endpoints.
    pub n_per_member: usize,
    /// Per-node displacement envelopes.
    pub node_displacements: Vec<NodeDisplacementSample>,
    /// Per-node support reaction envelopes.
    pub support_reactions: Vec<NodeReactionSample>,
    /// Number of nodes in the model.
    pub n_nodes: usize,
    /// Typed load sources, aligned with result input order.
    pub sources: Vec<LoadSource>,
}

impl Envelope {
    /// Build an envelope from multiple solved frame results.
    ///
    /// All results must agree on member and node counts. Member forces are
    /// sampled at `n_per_member` evenly spaced `xi` points; interior extrema
    /// between samples may be missed. Reactions are sampled once per result and
    /// read from the supported DOFs only.
    ///
    /// # Errors
    ///
    /// - [`FemError::InvalidInput`] if `results` is empty.
    /// - [`FemError::InvalidInput`] if `n_per_member < 2`.
    /// - [`FemError::InvalidInput`] if result counts disagree.
    /// - [`FemError::InvalidInput`] if a sampled value is non-finite.
    /// - Propagates errors from [`FrameAnalysisResult::section_forces`].
    pub fn from_frame_results(
        results: &[&FrameAnalysisResult],
        n_per_member: usize,
    ) -> Result<Self, FemError> {
        if results.is_empty() {
            return Err(FemError::InvalidInput(
                "envelope requires at least one result".to_string(),
            ));
        }
        if n_per_member < 2 {
            return Err(FemError::InvalidInput(format!(
                "n_per_member must be >= 2, got {n_per_member}"
            )));
        }

        let n_members = results[0].n_members();
        let n_nodes = results[0].n_nodes();
        for r in results {
            if r.n_members() != n_members {
                return Err(FemError::InvalidInput(format!(
                    "all results must have the same member count; got {} and {}",
                    n_members,
                    r.n_members()
                )));
            }
            if r.n_nodes() != n_nodes {
                return Err(FemError::InvalidInput(format!(
                    "all results must have the same node count; got {} and {}",
                    n_nodes,
                    r.n_nodes()
                )));
            }
        }

        let mut samples = Vec::with_capacity(n_members.saturating_mul(n_per_member));
        for member in 0..n_members {
            let handle = MemberHandle::from_index(member);
            for s in 0..n_per_member {
                let xi = s as f64 / (n_per_member - 1) as f64;
                let mut min = SectionForces::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
                let mut max =
                    SectionForces::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
                let mut min_sources = SectionForceSources::none();
                let mut max_sources = SectionForceSources::none();

                for (source_index, r) in results.iter().enumerate() {
                    let sf = r.section_forces(handle, xi)?;
                    require_finite("section force", sf.axial, member, xi)?;
                    require_finite("section force", sf.shear, member, xi)?;
                    require_finite("section force", sf.moment, member, xi)?;

                    if sf.axial < min.axial {
                        min.axial = sf.axial;
                        min_sources.axial = Some(source_index);
                    }
                    if sf.shear < min.shear {
                        min.shear = sf.shear;
                        min_sources.shear = Some(source_index);
                    }
                    if sf.moment < min.moment {
                        min.moment = sf.moment;
                        min_sources.moment = Some(source_index);
                    }
                    if sf.axial > max.axial {
                        max.axial = sf.axial;
                        max_sources.axial = Some(source_index);
                    }
                    if sf.shear > max.shear {
                        max.shear = sf.shear;
                        max_sources.shear = Some(source_index);
                    }
                    if sf.moment > max.moment {
                        max.moment = sf.moment;
                        max_sources.moment = Some(source_index);
                    }
                }

                samples.push(EnvelopeSample {
                    member_index: member,
                    xi,
                    min,
                    max,
                    min_sources,
                    max_sources,
                });
            }
        }

        let node_displacements = Self::compute_node_displacements(results, n_nodes)?;
        let support_reactions = Self::compute_support_reactions(results, n_nodes)?;

        Ok(Self {
            samples,
            n_results: results.len(),
            n_members,
            n_per_member,
            node_displacements,
            support_reactions,
            n_nodes,
            sources: results.iter().map(|r| r.load_source().clone()).collect(),
        })
    }

    fn compute_node_displacements(
        results: &[&FrameAnalysisResult],
        n_nodes: usize,
    ) -> Result<Vec<NodeDisplacementSample>, FemError> {
        let mut envelope = vec![std::array::from_fn(|_| Extremum::new()); n_nodes];
        for (source_index, r) in results.iter().enumerate() {
            let displacements = r.displacements();
            for node in 0..n_nodes {
                for dof in Dof::ALL {
                    let value = displacements[node * 3 + dof.index()];
                    require_node_finite("displacement", value, node, dof)?;
                    envelope[node][dof.index()].update(value, source_index);
                }
            }
        }
        Ok(envelope
            .into_iter()
            .enumerate()
            .map(|(node_index, [ux, uy, rz])| NodeDisplacementSample {
                node_index,
                ux,
                uy,
                rz,
            })
            .collect())
    }

    fn compute_support_reactions(
        results: &[&FrameAnalysisResult],
        n_nodes: usize,
    ) -> Result<Vec<NodeReactionSample>, FemError> {
        let mut envelope = vec![std::array::from_fn(|_| Extremum::new()); n_nodes];
        for (source_index, r) in results.iter().enumerate() {
            let reactions = r.reactions();
            for (node, dof) in r.support_dofs() {
                let value = reactions[node * 3 + dof.index()];
                require_node_finite("reaction", value, node, dof)?;
                envelope[node][dof.index()].update(value, source_index);
            }
        }
        Ok(envelope
            .into_iter()
            .enumerate()
            .map(|(node_index, [rx, ry, mz])| NodeReactionSample {
                node_index,
                rx,
                ry,
                mz,
            })
            .collect())
    }

    /// Typed source at an input index.
    pub fn source(&self, index: usize) -> Option<&LoadSource> {
        self.sources.get(index)
    }

    /// All samples for a member, in `xi` order.
    pub fn member_samples(&self, member_index: usize) -> &[EnvelopeSample] {
        if member_index >= self.n_members {
            return &self.samples[0..0];
        }
        let start = member_index * self.n_per_member;
        let end = start + self.n_per_member;
        &self.samples[start..end]
    }

    /// Displacement envelope for a node.
    pub fn node_displacement(&self, node_index: usize) -> Option<&NodeDisplacementSample> {
        self.node_displacements.get(node_index)
    }

    /// Support reaction envelope for a node.
    pub fn support_reaction(&self, node_index: usize) -> Option<&NodeReactionSample> {
        self.support_reactions.get(node_index)
    }

    /// Maximum absolute moment across the sampled member points and sources.
    pub fn max_abs_moment(&self) -> f64 {
        self.samples
            .iter()
            .map(|s| s.min.moment.abs().max(s.max.moment.abs()))
            .fold(0.0_f64, f64::max)
    }

    /// Maximum absolute shear across the sampled member points and sources.
    pub fn max_abs_shear(&self) -> f64 {
        self.samples
            .iter()
            .map(|s| s.min.shear.abs().max(s.max.shear.abs()))
            .fold(0.0_f64, f64::max)
    }

    /// Maximum absolute axial force across the sampled member points and sources.
    pub fn max_abs_axial(&self) -> f64 {
        self.samples
            .iter()
            .map(|s| s.min.axial.abs().max(s.max.axial.abs()))
            .fold(0.0_f64, f64::max)
    }
}

fn require_finite(what: &str, value: f64, member: usize, xi: f64) -> Result<(), FemError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(FemError::InvalidInput(format!(
            "{what} is non-finite at member {member}, xi {xi}: {value}"
        )))
    }
}

fn require_node_finite(what: &str, value: f64, node: usize, dof: Dof) -> Result<(), FemError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(FemError::InvalidInput(format!(
            "{what} is non-finite at node {node} DOF {}: {value}",
            dof.name()
        )))
    }
}

// ===========================================================================
// 3D Truss Envelope
// ===========================================================================

/// Per-node displacement envelope for a 3D truss across multiple load cases.
///
/// Each translational component (`ux`, `uy`, `uz`) is tracked independently
/// with signed min/max and governing source index.
#[derive(Debug, Clone, PartialEq)]
pub struct Truss3DNodeDisplacementSample {
    /// Node index in model order.
    pub node_index: usize,
    /// Horizontal displacement envelope (global x).
    pub ux: Extremum,
    /// Vertical displacement envelope (global y).
    pub uy: Extremum,
    /// Out-of-plane displacement envelope (global z).
    pub uz: Extremum,
}

/// Per-node support reaction envelope for a 3D truss across multiple load cases.
///
/// Only DOFs that carry a physical support reaction are populated. Free-DOF
/// round-off residuals are never sampled. A DOF that is unconstrained in every
/// input result remains unpopulated (`min = +∞`, `max = -∞`).
#[derive(Debug, Clone, PartialEq)]
pub struct Truss3DNodeReactionSample {
    /// Node index in model order.
    pub node_index: usize,
    /// Horizontal reaction envelope (global x).
    pub rx: Extremum,
    /// Vertical reaction envelope (global y).
    pub ry: Extremum,
    /// Out-of-plane reaction envelope (global z).
    pub rz: Extremum,
}

/// Per-element axial-force envelope for a 3D truss across multiple load cases.
///
/// Axial force is tension-positive, matching
/// [`TrussAnalysisResult3D::member_axial_force`].
#[derive(Debug, Clone, PartialEq)]
pub struct Truss3DAxialForceSample {
    /// Element index in model order.
    pub element_index: usize,
    /// Axial force envelope (tension positive).
    pub axial: Extremum,
}

/// Multi-case envelope over 3D truss results.
///
/// Aggregates displacements, support reactions, and axial forces across
/// multiple [`TrussAnalysisResult3D`] values. Every governing-source field is
/// an index into [`sources`](Self::sources), so duplicate case names remain
/// unambiguous.
///
/// # Compatibility
///
/// All input results must agree on node and element counts, node coordinates
/// (within a relative tolerance of `1e-9`), and element connectivity (exact
/// match). Results from different models (different geometry or connectivity)
/// are rejected with [`FemError::InvalidInput`].
///
/// # Reaction semantics
///
/// Reactions are sampled only at constrained DOFs (see
/// [`TrussAnalysisResult3D::constrained_dofs`]). Free-DOF residuals are
/// excluded. A DOF constrained in some results but not others is populated
/// only from the results where it is constrained.
///
/// # Tie behavior
///
/// Ties keep the first input result, consistent with the Frame Envelope.
///
/// # Errors
///
/// [`FemError::InvalidInput`] if `results` is empty, if result counts
/// disagree, if node coordinates or element connectivity differ, or if any
/// sampled value is non-finite.
#[derive(Debug, Clone)]
pub struct Truss3DEnvelope {
    /// Per-node displacement envelopes, in node order.
    pub node_displacements: Vec<Truss3DNodeDisplacementSample>,
    /// Per-node support reaction envelopes, in node order.
    pub support_reactions: Vec<Truss3DNodeReactionSample>,
    /// Per-element axial-force envelopes, in element order.
    pub axial_forces: Vec<Truss3DAxialForceSample>,
    /// Number of results that contributed, in input order.
    pub n_results: usize,
    /// Number of nodes in the model.
    pub n_nodes: usize,
    /// Number of elements in the model.
    pub n_elements: usize,
    /// Typed load sources, aligned with result input order.
    pub sources: Vec<LoadSource>,
}

impl Truss3DEnvelope {
    /// Build an envelope from multiple solved 3D truss results.
    ///
    /// All results must agree on node count, element count, node coordinates
    /// (within `1e-9` relative tolerance), and element connectivity (exact
    /// match). Displacements, reactions, and axial forces are aggregated
    /// independently.
    ///
    /// # Errors
    ///
    /// - [`FemError::InvalidInput`] if `results` is empty.
    /// - [`FemError::InvalidInput`] if result node or element counts disagree.
    /// - [`FemError::InvalidInput`] if node coordinates differ beyond tolerance.
    /// - [`FemError::InvalidInput`] if any node coordinate is non-finite.
    /// - [`FemError::InvalidInput`] if element connectivity differs.
    /// - [`FemError::InvalidInput`] if any sampled value is non-finite.
    pub fn from_results(results: &[&TrussAnalysisResult3D]) -> Result<Self, FemError> {
        if results.is_empty() {
            return Err(FemError::InvalidInput(
                "envelope requires at least one result".to_string(),
            ));
        }

        let n_nodes = results[0].n_nodes();
        let n_elements = results[0].n_elements();
        for r in results {
            if r.n_nodes() != n_nodes {
                return Err(FemError::InvalidInput(format!(
                    "all results must have the same node count; got {} and {}",
                    n_nodes,
                    r.n_nodes()
                )));
            }
            if r.n_elements() != n_elements {
                return Err(FemError::InvalidInput(format!(
                    "all results must have the same element count; got {} and {}",
                    n_elements,
                    r.n_elements()
                )));
            }
        }

        Self::validate_topology(results)?;

        let node_displacements = Self::compute_displacements(results, n_nodes)?;
        let support_reactions = Self::compute_reactions(results, n_nodes)?;
        let axial_forces = Self::compute_axial_forces(results, n_elements)?;

        Ok(Self {
            node_displacements,
            support_reactions,
            axial_forces,
            n_results: results.len(),
            n_nodes,
            n_elements,
            sources: results.iter().map(|r| r.load_source().clone()).collect(),
        })
    }

    fn validate_topology(results: &[&TrussAnalysisResult3D]) -> Result<(), FemError> {
        let ref_coords = &results[0].node_coords;
        let ref_elements = &results[0].element_nodes;
        let tol = 1e-9;
        for (idx, r) in results.iter().enumerate().skip(1) {
            for (node, &(rx, ry, rz)) in ref_coords.iter().enumerate() {
                let (x, y, z) = r.node_coords[node];
                if !rx.is_finite()
                    || !ry.is_finite()
                    || !rz.is_finite()
                    || !x.is_finite()
                    || !y.is_finite()
                    || !z.is_finite()
                {
                    return Err(FemError::InvalidInput(format!(
                        "node {node} has non-finite coordinates between result 0 and \
                         result {idx}: ({rx}, {ry}, {rz}) vs ({x}, {y}, {z})"
                    )));
                }
                let scale = rx
                    .abs()
                    .max(ry.abs())
                    .max(rz.abs())
                    .max(x.abs())
                    .max(y.abs())
                    .max(z.abs())
                    .max(1.0);
                if (rx - x).abs() > tol * scale
                    || (ry - y).abs() > tol * scale
                    || (rz - z).abs() > tol * scale
                {
                    return Err(FemError::InvalidInput(format!(
                        "node {node} coordinates differ between result 0 and result {idx}: \
                         ({rx}, {ry}, {rz}) vs ({x}, {y}, {z})"
                    )));
                }
            }
            if r.element_nodes != *ref_elements {
                return Err(FemError::InvalidInput(format!(
                    "element connectivity differs between result 0 and result {idx}"
                )));
            }
        }
        Ok(())
    }

    fn compute_displacements(
        results: &[&TrussAnalysisResult3D],
        n_nodes: usize,
    ) -> Result<Vec<Truss3DNodeDisplacementSample>, FemError> {
        let mut envelope: Vec<[Extremum; 3]> = (0..n_nodes)
            .map(|_| std::array::from_fn(|_| Extremum::new()))
            .collect();
        for (source_index, r) in results.iter().enumerate() {
            let displacements = &r.displacements;
            for node in 0..n_nodes {
                for dof in 0..3 {
                    let value = displacements[node * 3 + dof];
                    if !value.is_finite() {
                        return Err(FemError::InvalidInput(format!(
                            "displacement is non-finite at node {node} DOF {dof}: {value}"
                        )));
                    }
                    envelope[node][dof].update(value, source_index);
                }
            }
        }
        Ok(envelope
            .into_iter()
            .enumerate()
            .map(|(node_index, [ux, uy, uz])| Truss3DNodeDisplacementSample {
                node_index,
                ux,
                uy,
                uz,
            })
            .collect())
    }

    fn compute_reactions(
        results: &[&TrussAnalysisResult3D],
        n_nodes: usize,
    ) -> Result<Vec<Truss3DNodeReactionSample>, FemError> {
        let mut envelope: Vec<[Extremum; 3]> = (0..n_nodes)
            .map(|_| std::array::from_fn(|_| Extremum::new()))
            .collect();
        for (source_index, r) in results.iter().enumerate() {
            let reactions = &r.reactions;
            for &(node, dof) in r.constrained_dofs() {
                if node >= n_nodes || dof >= 3 {
                    continue;
                }
                let value = reactions[node * 3 + dof];
                if !value.is_finite() {
                    return Err(FemError::InvalidInput(format!(
                        "reaction is non-finite at node {node} DOF {dof}: {value}"
                    )));
                }
                envelope[node][dof].update(value, source_index);
            }
        }
        Ok(envelope
            .into_iter()
            .enumerate()
            .map(|(node_index, [rx, ry, rz])| Truss3DNodeReactionSample {
                node_index,
                rx,
                ry,
                rz,
            })
            .collect())
    }

    fn compute_axial_forces(
        results: &[&TrussAnalysisResult3D],
        n_elements: usize,
    ) -> Result<Vec<Truss3DAxialForceSample>, FemError> {
        let mut envelope: Vec<Extremum> = (0..n_elements).map(|_| Extremum::new()).collect();
        for (source_index, r) in results.iter().enumerate() {
            let axial = r.member_axial_forces();
            for elem in 0..n_elements {
                let value = axial[elem];
                if !value.is_finite() {
                    return Err(FemError::InvalidInput(format!(
                        "axial force is non-finite at element {elem}: {value}"
                    )));
                }
                envelope[elem].update(value, source_index);
            }
        }
        Ok(envelope
            .into_iter()
            .enumerate()
            .map(|(element_index, axial)| Truss3DAxialForceSample {
                element_index,
                axial,
            })
            .collect())
    }

    /// Typed source at an input index.
    pub fn source(&self, index: usize) -> Option<&LoadSource> {
        self.sources.get(index)
    }

    /// Displacement envelope for a node.
    pub fn node_displacement(&self, node_index: usize) -> Option<&Truss3DNodeDisplacementSample> {
        self.node_displacements.get(node_index)
    }

    /// Support reaction envelope for a node.
    pub fn support_reaction(&self, node_index: usize) -> Option<&Truss3DNodeReactionSample> {
        self.support_reactions.get(node_index)
    }

    /// Axial-force envelope for an element.
    pub fn axial_force(&self, element_index: usize) -> Option<&Truss3DAxialForceSample> {
        self.axial_forces.get(element_index)
    }

    /// Maximum absolute axial force across all elements and sources.
    pub fn max_abs_axial(&self) -> f64 {
        self.axial_forces
            .iter()
            .map(|s| s.axial.min.abs().max(s.axial.max.abs()))
            .fold(0.0_f64, f64::max)
    }
}

// ===========================================================================
// 2D Truss Envelope
// ===========================================================================

use crate::truss::TrussAnalysisResult;

/// Per-node displacement envelope for a 2D truss across multiple load cases.
///
/// Each translational component (`ux`, `uy`) is tracked independently with
/// signed min/max and governing source index.
#[derive(Debug, Clone, PartialEq)]
pub struct TrussNodeDisplacementSample {
    /// Node index in model order.
    pub node_index: usize,
    /// Horizontal displacement envelope (global x).
    pub ux: Extremum,
    /// Vertical displacement envelope (global y).
    pub uy: Extremum,
}

/// Per-node support reaction envelope for a 2D truss across multiple load cases.
///
/// Only DOFs that carry a physical support reaction are populated. Free-DOF
/// round-off residuals are never sampled. A DOF that is unconstrained in every
/// input result remains unpopulated (`min = +∞`, `max = -∞`).
#[derive(Debug, Clone, PartialEq)]
pub struct TrussNodeReactionSample {
    /// Node index in model order.
    pub node_index: usize,
    /// Horizontal reaction envelope (global x).
    pub rx: Extremum,
    /// Vertical reaction envelope (global y).
    pub ry: Extremum,
}

/// Per-element axial-force envelope for a 2D truss across multiple load cases.
///
/// Axial force is tension-positive, matching
/// [`TrussAnalysisResult::axial_forces`].
#[derive(Debug, Clone, PartialEq)]
pub struct TrussAxialForceSample {
    /// Element index in model order.
    pub element_index: usize,
    /// Axial force envelope (tension positive).
    pub axial: Extremum,
}

/// Multi-case envelope over 2D truss results.
///
/// Aggregates displacements, support reactions, and axial forces across
/// multiple [`TrussAnalysisResult`] values. Every governing-source field is
/// an index into [`sources`](Self::sources), so duplicate case names remain
/// unambiguous.
///
/// # Topology validation
///
/// `TrussEnvelope` validates that all input results share the same node
/// coordinates (within a relative tolerance of `1e-9`) and element
/// connectivity (exact match). This is possible because
/// [`TrussAnalysisResult`] stores `node_coords` and `element_nodes`
/// snapshots. Results from different models (different geometry or
/// connectivity) are rejected with [`FemError::InvalidInput`].
/// [`Truss3DEnvelope`] applies the same validation for 3D results.
///
/// # Reaction semantics
///
/// Reactions are sampled only at constrained DOFs (see
/// [`TrussAnalysisResult::constrained_dofs`]). Free-DOF residuals are
/// excluded. A DOF constrained in some results but not others is populated
/// only from the results where it is constrained.
///
/// # Tie behavior
///
/// Ties keep the first input result, consistent with [`Envelope`] and
/// [`Truss3DEnvelope`].
///
/// # Errors
///
/// [`FemError::InvalidInput`] if `results` is empty, if result counts
/// disagree, if node coordinates or element connectivity differ, or if any
/// sampled value is non-finite.
#[derive(Debug, Clone)]
pub struct TrussEnvelope {
    /// Per-node displacement envelopes, in node order.
    pub node_displacements: Vec<TrussNodeDisplacementSample>,
    /// Per-node support reaction envelopes, in node order.
    pub support_reactions: Vec<TrussNodeReactionSample>,
    /// Per-element axial-force envelopes, in element order.
    pub axial_forces: Vec<TrussAxialForceSample>,
    /// Number of results that contributed, in input order.
    pub n_results: usize,
    /// Number of nodes in the model.
    pub n_nodes: usize,
    /// Number of elements in the model.
    pub n_elements: usize,
    /// Typed load sources, aligned with result input order.
    pub sources: Vec<LoadSource>,
}

impl TrussEnvelope {
    /// Build an envelope from multiple solved 2D truss results.
    ///
    /// All results must agree on node count, element count, node coordinates
    /// (within `1e-9` relative tolerance), and element connectivity (exact
    /// match). Displacements, reactions, and axial forces are aggregated
    /// independently.
    ///
    /// # Errors
    ///
    /// - [`FemError::InvalidInput`] if `results` is empty.
    /// - [`FemError::InvalidInput`] if result node or element counts disagree.
    /// - [`FemError::InvalidInput`] if node coordinates differ beyond tolerance.
    /// - [`FemError::InvalidInput`] if element connectivity differs.
    /// - [`FemError::InvalidInput`] if any sampled value is non-finite.
    pub fn from_results(results: &[&TrussAnalysisResult]) -> Result<Self, FemError> {
        if results.is_empty() {
            return Err(FemError::InvalidInput(
                "envelope requires at least one result".to_string(),
            ));
        }

        let n_nodes = results[0].n_nodes();
        let n_elements = results[0].n_elements();
        for r in results {
            if r.n_nodes() != n_nodes {
                return Err(FemError::InvalidInput(format!(
                    "all results must have the same node count; got {} and {}",
                    n_nodes,
                    r.n_nodes()
                )));
            }
            if r.n_elements() != n_elements {
                return Err(FemError::InvalidInput(format!(
                    "all results must have the same element count; got {} and {}",
                    n_elements,
                    r.n_elements()
                )));
            }
        }

        Self::validate_topology(results)?;

        let node_displacements = Self::compute_displacements(results, n_nodes)?;
        let support_reactions = Self::compute_reactions(results, n_nodes)?;
        let axial_forces = Self::compute_axial_forces(results, n_elements)?;

        Ok(Self {
            node_displacements,
            support_reactions,
            axial_forces,
            n_results: results.len(),
            n_nodes,
            n_elements,
            sources: results.iter().map(|r| r.load_source().clone()).collect(),
        })
    }

    fn validate_topology(results: &[&TrussAnalysisResult]) -> Result<(), FemError> {
        let ref_coords = &results[0].node_coords;
        let ref_elements = &results[0].element_nodes;
        let tol = 1e-9;
        for (idx, r) in results.iter().enumerate().skip(1) {
            for (node, &(rx, ry)) in ref_coords.iter().enumerate() {
                let (x, y) = r.node_coords[node];
                let scale = rx.abs().max(ry.abs()).max(x.abs()).max(y.abs()).max(1.0);
                if (rx - x).abs() > tol * scale || (ry - y).abs() > tol * scale {
                    return Err(FemError::InvalidInput(format!(
                        "node {node} coordinates differ between result 0 and result {idx}: \
                         ({rx}, {ry}) vs ({x}, {y})"
                    )));
                }
            }
            if r.element_nodes != *ref_elements {
                return Err(FemError::InvalidInput(format!(
                    "element connectivity differs between result 0 and result {idx}"
                )));
            }
        }
        Ok(())
    }

    fn compute_displacements(
        results: &[&TrussAnalysisResult],
        n_nodes: usize,
    ) -> Result<Vec<TrussNodeDisplacementSample>, FemError> {
        let mut envelope: Vec<[Extremum; 2]> = (0..n_nodes)
            .map(|_| std::array::from_fn(|_| Extremum::new()))
            .collect();
        for (source_index, r) in results.iter().enumerate() {
            let displacements = &r.displacements;
            for node in 0..n_nodes {
                for dof in 0..2 {
                    let value = displacements[node * 2 + dof];
                    if !value.is_finite() {
                        return Err(FemError::InvalidInput(format!(
                            "displacement is non-finite at node {node} DOF {dof}: {value}"
                        )));
                    }
                    envelope[node][dof].update(value, source_index);
                }
            }
        }
        Ok(envelope
            .into_iter()
            .enumerate()
            .map(|(node_index, [ux, uy])| TrussNodeDisplacementSample { node_index, ux, uy })
            .collect())
    }

    fn compute_reactions(
        results: &[&TrussAnalysisResult],
        n_nodes: usize,
    ) -> Result<Vec<TrussNodeReactionSample>, FemError> {
        let mut envelope: Vec<[Extremum; 2]> = (0..n_nodes)
            .map(|_| std::array::from_fn(|_| Extremum::new()))
            .collect();
        for (source_index, r) in results.iter().enumerate() {
            let reactions = &r.reactions;
            for &(node, dof) in r.constrained_dofs() {
                if node >= n_nodes || dof >= 2 {
                    continue;
                }
                let value = reactions[node * 2 + dof];
                if !value.is_finite() {
                    return Err(FemError::InvalidInput(format!(
                        "reaction is non-finite at node {node} DOF {dof}: {value}"
                    )));
                }
                envelope[node][dof].update(value, source_index);
            }
        }
        Ok(envelope
            .into_iter()
            .enumerate()
            .map(|(node_index, [rx, ry])| TrussNodeReactionSample { node_index, rx, ry })
            .collect())
    }

    fn compute_axial_forces(
        results: &[&TrussAnalysisResult],
        n_elements: usize,
    ) -> Result<Vec<TrussAxialForceSample>, FemError> {
        let mut envelope: Vec<Extremum> = (0..n_elements).map(|_| Extremum::new()).collect();
        for (source_index, r) in results.iter().enumerate() {
            let axial = &r.axial_forces;
            for elem in 0..n_elements {
                let value = axial[elem];
                if !value.is_finite() {
                    return Err(FemError::InvalidInput(format!(
                        "axial force is non-finite at element {elem}: {value}"
                    )));
                }
                envelope[elem].update(value, source_index);
            }
        }
        Ok(envelope
            .into_iter()
            .enumerate()
            .map(|(element_index, axial)| TrussAxialForceSample {
                element_index,
                axial,
            })
            .collect())
    }

    /// Typed source at an input index.
    pub fn source(&self, index: usize) -> Option<&LoadSource> {
        self.sources.get(index)
    }

    /// Displacement envelope for a node.
    pub fn node_displacement(&self, node_index: usize) -> Option<&TrussNodeDisplacementSample> {
        self.node_displacements.get(node_index)
    }

    /// Support reaction envelope for a node.
    pub fn support_reaction(&self, node_index: usize) -> Option<&TrussNodeReactionSample> {
        self.support_reactions.get(node_index)
    }

    /// Axial-force envelope for an element.
    pub fn axial_force(&self, element_index: usize) -> Option<&TrussAxialForceSample> {
        self.axial_forces.get(element_index)
    }

    /// Maximum absolute axial force across all elements and sources.
    pub fn max_abs_axial(&self) -> f64 {
        self.axial_forces
            .iter()
            .map(|s| s.axial.min.abs().max(s.axial.max.abs()))
            .fold(0.0_f64, f64::max)
    }
}
