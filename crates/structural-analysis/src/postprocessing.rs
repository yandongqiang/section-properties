//! Post-processing data: envelope and derived results.
//!
//! This module contains types that aggregate or transform already-computed
//! [`crate::frame::FrameAnalysisResult`] data. They do **not** call solver
//! methods or re-assemble stiffness matrices — they are pure consumers of
//! analysis results.
//!
//! # Layer
//!
//! These types sit between *Analysis Result* (Layer B) and *Visualization*
//! (Layer D):
//!
//! ```text
//! Analysis Result → Post-processing Data → Visualization
//! ```

use crate::FemError;
use crate::beam_fem::SectionForces;
use crate::frame::FrameAnalysisResult;

// ---------------------------------------------------------------------------
// Envelope
// ---------------------------------------------------------------------------

/// One sampled point of a force envelope across multiple load cases.
///
/// `min` and `max` track the minimum and maximum of each section-force
/// component (`N`, `V`, `M`) **independently** over all input results.
/// They are not an absolute-value envelope: `min` may be negative and
/// `max` may be positive, reflecting the true range.
///
/// # Coordinates
///
/// - [`member_index`](Self::member_index): member index in the model.
/// - [`xi`](Self::xi): member-local normalized coordinate in `[0, 1]`
///   (`xi = 0` at the member's start node, `xi = 1` at the end node).
///
/// # Sampling limitation
///
/// Envelope values are evaluated at the requested sampling locations and
/// do **not** guarantee capture of unsampled interior extrema. For members
/// with distributed loads, the true maximum moment may occur between
/// sample points. Increase `n_per_member` to improve coverage.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnvelopeSample {
    /// Member index in model order.
    pub member_index: usize,
    /// Member-local normalized coordinate, `0` at start, `1` at end.
    pub xi: f64,
    /// Minimum section forces observed at this point.
    pub min: SectionForces,
    /// Maximum section forces observed at this point.
    pub max: SectionForces,
}

/// Multi-case force envelope: min/max `N`, `V`, `M` per member per sample.
///
/// Constructed from a slice of already-solved [`FrameAnalysisResult`]s via
/// [`Envelope::from_frame_results`]. The envelope is a pure data snapshot:
/// it owns all its data, implements `Clone`, and does not hold solver or
/// model references.
///
/// # Semantics
///
/// - `min` = minimum over all input results (per component, independently).
/// - `max` = maximum over all input results (per component, independently).
/// - These are **not** absolute-value envelopes. If case A gives `M = -100`
///   and case B gives `M = +40`, then `min.moment = -100` and
///   `max.moment = +40`.
///
/// # Sampling
///
/// Each member is sampled at `n_per_member` evenly spaced `xi` points
/// (including `xi = 0` and `xi = 1`). The sampling is **fixed**, not
/// event-aware: it does not add extra points at load discontinuities.
/// Interior extrema between sample points may be missed.
#[derive(Debug, Clone)]
pub struct Envelope {
    /// One sample per `(member, xi)` in member-major order.
    pub samples: Vec<EnvelopeSample>,
    /// Number of results that contributed.
    pub n_results: usize,
    /// Number of members in the model.
    pub n_members: usize,
    /// Samples per member (including endpoints).
    pub n_per_member: usize,
}

impl Envelope {
    /// Build an envelope from multiple solved frame results.
    ///
    /// Each member is sampled at `n_per_member` evenly spaced points
    /// (`xi = 0, 1/(n-1), ..., 1`). At each point, `section_forces` is
    /// evaluated for every result, and the per-component min/max is tracked.
    ///
    /// All results must have the same number of members; otherwise
    /// [`FemError::InvalidInput`] is returned.
    ///
    /// # Errors
    ///
    /// - [`FemError::InvalidInput`] if `results` is empty.
    /// - [`FemError::InvalidInput`] if `n_per_member < 2`.
    /// - [`FemError::InvalidInput`] if results disagree on member count.
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
        for r in results {
            if r.n_members() != n_members {
                return Err(FemError::InvalidInput(format!(
                    "all results must have the same member count; got {} and {}",
                    n_members,
                    r.n_members()
                )));
            }
        }

        let mut samples = Vec::with_capacity(n_members * n_per_member);

        for member in 0..n_members {
            let handle = crate::frame::MemberHandle::from_index(member);
            for s in 0..n_per_member {
                let xi = s as f64 / (n_per_member - 1) as f64;

                let mut min = SectionForces::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
                let mut max =
                    SectionForces::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

                for r in results {
                    let sf = r.section_forces(handle, xi)?;
                    min.axial = min.axial.min(sf.axial);
                    min.shear = min.shear.min(sf.shear);
                    min.moment = min.moment.min(sf.moment);
                    max.axial = max.axial.max(sf.axial);
                    max.shear = max.shear.max(sf.shear);
                    max.moment = max.moment.max(sf.moment);
                }

                samples.push(EnvelopeSample {
                    member_index: member,
                    xi,
                    min,
                    max,
                });
            }
        }

        Ok(Self {
            samples,
            n_results: results.len(),
            n_members,
            n_per_member,
        })
    }

    /// All samples for a specific member, in `xi` order.
    pub fn member_samples(&self, member_index: usize) -> &[EnvelopeSample] {
        let start = member_index * self.n_per_member;
        let end = start + self.n_per_member;
        if end > self.samples.len() {
            &self.samples[0..0]
        } else {
            &self.samples[start..end]
        }
    }

    /// Maximum absolute moment across all samples and all cases.
    ///
    /// This is a convenience accessor for design checks that need the
    /// worst-case moment magnitude. It returns `max(|min.moment|, |max.moment|)`
    /// over all samples.
    pub fn max_abs_moment(&self) -> f64 {
        self.samples
            .iter()
            .map(|s| s.min.moment.abs().max(s.max.moment.abs()))
            .fold(0.0_f64, f64::max)
    }

    /// Maximum absolute shear across all samples and all cases.
    pub fn max_abs_shear(&self) -> f64 {
        self.samples
            .iter()
            .map(|s| s.min.shear.abs().max(s.max.shear.abs()))
            .fold(0.0_f64, f64::max)
    }

    /// Maximum absolute axial force across all samples and all cases.
    pub fn max_abs_axial(&self) -> f64 {
        self.samples
            .iter()
            .map(|s| s.min.axial.abs().max(s.max.axial.abs()))
            .fold(0.0_f64, f64::max)
    }
}
