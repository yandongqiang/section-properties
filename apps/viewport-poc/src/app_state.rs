//! Application state types — project, analysis result, and messages.
//!
//! These types separate the user's engineering data (the [`Project`]) from
//! rendering concerns.  The GUI may inspect the project but must not mutate
//! analysis results directly; solving is an explicit operation via
//! [`Project::solve`].

use section_properties::SolverSelection;
use structural_analysis::{FrameAnalysisResult3D, FrameSolver3D};

use crate::display;
use crate::edit_model::EditModel;

// ── Messages ───────────────────────────────────────────────────────────

/// Diagnostic message level for the status panel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageLevel {
    Info,
    Warning,
    Error,
}

/// A diagnostic message shown in the bottom status panel.
#[derive(Debug, Clone)]
pub struct AppMessage {
    pub level: MessageLevel,
    pub text: String,
}

impl AppMessage {
    pub fn info(text: impl Into<String>) -> Self {
        Self {
            level: MessageLevel::Info,
            text: text.into(),
        }
    }

    pub fn warning(text: impl Into<String>) -> Self {
        Self {
            level: MessageLevel::Warning,
            text: text.into(),
        }
    }

    pub fn error(text: impl Into<String>) -> Self {
        Self {
            level: MessageLevel::Error,
            text: text.into(),
        }
    }
}

// ── Analysis status ────────────────────────────────────────────────────

/// Coarse-grained status of the analysis result relative to the model.
///
/// Determined by comparing the model's revision counter against the
/// revision captured at solve time, plus a flag for the last solve
/// attempt.
///
/// # Priority
///
/// `Failed` takes priority over all other states — if the last solve
/// attempt failed, the user must be informed regardless of whether a
/// prior result exists.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AnalysisStatus {
    /// No analysis has ever been performed on the current model.
    NeverSolved,
    /// Results exist and match the current model revision.
    Current,
    /// Results exist but the model has been edited since the last solve.
    Stale,
    /// The last solve attempt failed.
    Failed,
}

// ── Solver options ─────────────────────────────────────────────────────

/// Linear-solver backend exposed in the Analysis job panel.
///
/// This is the **only** tunable the 3D frame solver actually honours.  It
/// maps 1:1 onto [`section_properties::SolverSelection`] and ultimately onto
/// [`structural_analysis::FrameSolver3D::set_solver`].  The default is
/// [`SolverBackend::Auto`], which reproduces the historical behaviour
/// (`FrameModel3D::solve` uses `SolverSelection::Auto`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SolverBackend {
    /// Registry-based automatic selection.
    #[default]
    Auto,
    /// Dense Gaussian elimination (general, any size; best for small systems).
    Dense,
    /// Skyline LDLᵀ (symmetric, positive definite only).
    SkylineLdlt,
    /// Sparse LU (general; best for medium/large systems).
    SparseLu,
    /// Conjugate gradient (symmetric, positive definite only).
    Cg,
    /// Incomplete-Cholesky preconditioned CG (symmetric, positive definite).
    Iccg,
}

impl SolverBackend {
    /// All backends in menu order.
    pub const ALL: [SolverBackend; 6] = [
        SolverBackend::Auto,
        SolverBackend::Dense,
        SolverBackend::SkylineLdlt,
        SolverBackend::SparseLu,
        SolverBackend::Cg,
        SolverBackend::Iccg,
    ];

    /// Short label for the GUI combo box.
    pub const fn label(self) -> &'static str {
        match self {
            SolverBackend::Auto => "Auto",
            SolverBackend::Dense => "Dense (Gaussian)",
            SolverBackend::SkylineLdlt => "Skyline LDLᵀ",
            SolverBackend::SparseLu => "Sparse LU",
            SolverBackend::Cg => "CG",
            SolverBackend::Iccg => "ICCG",
        }
    }

    /// Canonical registry name, or `None` for [`SolverBackend::Auto`].
    pub const fn name(self) -> Option<&'static str> {
        match self {
            SolverBackend::Auto => None,
            SolverBackend::Dense => Some("dense"),
            SolverBackend::SkylineLdlt => Some("skyline_ldlt"),
            SolverBackend::SparseLu => Some("sparse_lu"),
            SolverBackend::Cg => Some("cg"),
            SolverBackend::Iccg => Some("iccg"),
        }
    }

    /// Convert to the crate's solver selection.
    ///
    /// Explicit backends are *authoritative*: if the requested backend cannot
    /// legally handle the matrix, the solve returns an error (it is never
    /// silently substituted).
    pub fn to_selection(self) -> SolverSelection {
        match self.name() {
            Some(name) => SolverSelection::named(name),
            None => SolverSelection::Auto,
        }
    }
}

// ── Analysis result ────────────────────────────────────────────────────

/// Analysis result paired with the model revision and solver backend it was
/// produced from.
///
/// The [`solved_revision`](Self::solved_revision) is captured at solve
/// time so that subsequent edits can be detected by comparing it against
/// the model's current revision.  [`solved_backend`](Self::solved_backend)
/// records the requested solver so that changing the backend also marks the
/// result stale.
pub struct AnalysisResult {
    pub result: FrameAnalysisResult3D,
    pub solved_revision: u64,
    /// Solver backend requested for the solve that produced this result.
    pub solved_backend: SolverBackend,
}

impl AnalysisResult {
    pub fn new(
        result: FrameAnalysisResult3D,
        solved_revision: u64,
        solved_backend: SolverBackend,
    ) -> Self {
        Self {
            result,
            solved_revision,
            solved_backend,
        }
    }
}

// ── Project ────────────────────────────────────────────────────────────

/// The user's engineering project.
///
/// Holds the structural model and an optional analysis result.  The GUI
/// must not mutate the analysis result directly — use [`Project::solve`]
/// to (re-)run the analysis.
pub struct Project {
    pub name: String,
    pub model: Option<EditModel>,
    pub analysis: Option<AnalysisResult>,
    /// Solver backend selected for the next solve.
    solver_backend: SolverBackend,
    last_solve_failed: bool,
    /// Message from the most recent failed solve attempt (for the job panel).
    last_solve_error: Option<String>,
}

impl Default for Project {
    fn default() -> Self {
        Self::empty()
    }
}

impl Project {
    /// Create an empty project.
    pub fn empty() -> Self {
        Self {
            name: "Untitled".to_string(),
            model: None,
            analysis: None,
            solver_backend: SolverBackend::default(),
            last_solve_failed: false,
            last_solve_error: None,
        }
    }

    /// `true` if no model is loaded.
    pub fn is_empty(&self) -> bool {
        self.model.is_none()
    }

    /// Current analysis status relative to the model.
    ///
    /// See [`AnalysisStatus`] for the priority rules.
    pub fn analysis_status(&self) -> AnalysisStatus {
        if self.last_solve_failed {
            return AnalysisStatus::Failed;
        }
        match (&self.model, &self.analysis) {
            (None, _) | (Some(_), None) => AnalysisStatus::NeverSolved,
            (Some(model), Some(a)) => {
                if a.solved_revision == model.revision() && a.solved_backend == self.solver_backend {
                    AnalysisStatus::Current
                } else {
                    AnalysisStatus::Stale
                }
            }
        }
    }

    /// Solver backend selected for the next solve.
    pub fn solver_backend(&self) -> SolverBackend {
        self.solver_backend
    }

    /// Select the solver backend.
    ///
    /// Existing results become [`AnalysisStatus::Stale`] when the backend
    /// differs from the one that produced them — the panel prompts a re-run
    /// rather than presenting results from a different solver as current.
    pub fn set_solver_backend(&mut self, backend: SolverBackend) {
        self.solver_backend = backend;
    }

    /// Reset solver options to their defaults (currently just the backend).
    pub fn reset_solver_options(&mut self) {
        self.solver_backend = SolverBackend::default();
    }

    /// Message from the most recent failed solve attempt, if any.
    pub fn last_solve_error(&self) -> Option<&str> {
        self.last_solve_error.as_deref()
    }

    /// Solver backend that produced the current result, if results are current.
    pub fn solved_backend(&self) -> Option<SolverBackend> {
        if self.analysis_status() == AnalysisStatus::Current {
            self.analysis.as_ref().map(|a| a.solved_backend)
        } else {
            None
        }
    }

    /// `true` if analysis results are available and current.
    pub fn has_results(&self) -> bool {
        self.analysis_status() == AnalysisStatus::Current
    }

    /// `true` if results exist but are stale (model edited since solve).
    pub fn results_are_stale(&self) -> bool {
        self.analysis_status() == AnalysisStatus::Stale
    }

    /// `true` if the last solve attempt failed.
    pub fn solve_failed(&self) -> bool {
        self.analysis_status() == AnalysisStatus::Failed
    }

    /// Borrow the current analysis result, or `None` if stale/failed/none.
    ///
    /// Implements display Policy A: only [`AnalysisStatus::Current`]
    /// results are returned.  Stale, failed, or absent results are hidden
    /// so the UI never displays outdated diagnostics.
    pub fn analysis_result(&self) -> Option<&FrameAnalysisResult3D> {
        if self.analysis_status() == AnalysisStatus::Current {
            self.analysis.as_ref().map(|a| &a.result)
        } else {
            None
        }
    }

    /// Load a new model, replacing any current model.
    ///
    /// Existing analysis results are discarded — the new model has its
    /// own revision counter starting at 0, so any prior result is
    /// meaningless.  The failed flag is also reset.
    pub fn load_model(&mut self, name: String, model: EditModel) {
        self.name = name;
        self.model = Some(model);
        self.analysis = None;
        self.last_solve_failed = false;
        self.last_solve_error = None;
    }

    /// Run the FEM analysis on the current model.
    ///
    /// On success, stores the result tagged with the current model
    /// revision and clears the failed flag.  On failure, sets the
    /// failed flag but leaves any prior result intact (though it will
    /// not be displayed — see [`analysis_result`](Self::analysis_result)).
    ///
    /// This is the only way to produce a new analysis result — the GUI
    /// must not fabricate results.
    pub fn solve(&mut self) -> Result<(), String> {
        let outcome = self.run_analysis();
        self.last_solve_failed = outcome.is_err();
        self.last_solve_error = outcome.as_ref().err().cloned();
        outcome
    }

    /// Internal: produce a new analysis result without touching the
    /// failed flag.  Only modifies `self.analysis` on success.
    fn run_analysis(&mut self) -> Result<(), String> {
        let edit = self.model.as_ref().ok_or("No model loaded")?;
        let revision = edit.revision();
        let backend = self.solver_backend;
        let frame_model = edit.to_frame_model().map_err(|e| e.message())?;
        let mut solver = FrameSolver3D::from_model(&frame_model).map_err(|e| format!("{e}"))?;
        solver.set_solver(backend.to_selection());
        let result = solver.solve().map_err(|e| format!("{e}"))?;
        self.analysis = Some(AnalysisResult::new(result, revision, backend));
        Ok(())
    }

    /// Replace the current model, preserving the project name.
    ///
    /// Invalidates any existing analysis result (the new model's revision
    /// will not match the prior `solved_revision`).  Does NOT reset the
    /// model's revision — the caller is responsible for ensuring the
    /// revision invariant (e.g. via `ensure_revision_at_least` for
    /// undo/redo).
    pub fn replace_model(&mut self, model: EditModel) {
        self.model = Some(model);
        self.analysis = None;
        self.last_solve_failed = false;
        self.last_solve_error = None;
    }

    /// Replace the current model as the result of an **in-place edit** (the
    /// new model is a modified copy of the previous one — every `commit_edit`
    /// candidate is).
    ///
    /// Unlike [`replace_model`], a previously-solved result is **retained**
    /// rather than discarded, so the bumped model revision is reported as
    /// [`AnalysisStatus::Stale`] instead of [`AnalysisStatus::NeverSolved`].
    /// The result stays hidden while stale: [`analysis_result`](Self::analysis_result)
    /// still applies Policy A and returns `None` unless the status is
    /// [`AnalysisStatus::Current`].  A model that was never solved has no
    /// result to retain and remains [`AnalysisStatus::NeverSolved`].
    ///
    /// The caller must ensure `model` really is an edit of the current model
    /// (its revision differs), so a retained result is never mistaken for
    /// current.
    pub fn replace_model_after_edit(&mut self, model: EditModel) {
        self.model = Some(model);
        self.last_solve_failed = false;
        self.last_solve_error = None;
    }

    /// Clear the project to the empty state.
    pub fn clear(&mut self) {
        self.name = "Untitled".to_string();
        self.model = None;
        self.analysis = None;
        self.last_solve_failed = false;
        self.last_solve_error = None;
    }
}

// ── Model / result summaries ───────────────────────────────────────────

/// Read-only counts and validity warnings for the model, as shown in the
/// Analysis job panel.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ModelSummary {
    pub n_nodes: usize,
    pub n_members: usize,
    /// Total degrees of freedom (`6 × nodes`).
    pub total_dof: usize,
    /// DOFs constrained by supports.
    pub constrained_dof: usize,
    pub n_supports: usize,
    pub n_nodal_loads: usize,
    /// Uniform distributed loads + member point loads.
    pub n_member_loads: usize,
    /// Distinct member materials, identified by their `(E, G)` pair.
    pub n_materials: usize,
    /// Human-readable model-validity warnings (empty = none).
    pub warnings: Vec<String>,
}

impl ModelSummary {
    /// Build a summary from an editable model.
    pub fn from_model(model: &EditModel) -> Self {
        let n_nodes = model.n_nodes();
        let n_members = model.n_members();
        let total_dof = n_nodes * 6;

        let n_supports = model.supports().len();
        let constrained_dof: usize = model
            .supports()
            .iter()
            .map(|(_, spec)| spec.n_constrained())
            .sum();

        let n_nodal_loads = model.nodal_loads().len();
        let n_member_loads =
            model.member_uniform_loads().len() + model.member_point_loads().len();

        // Distinct materials by (E, G) bit patterns (f64 has no Hash/Eq).
        let mut materials: Vec<(u64, u64)> = Vec::new();
        for m in model.members() {
            let key = (m.section.E().to_bits(), m.section.G().to_bits());
            if !materials.contains(&key) {
                materials.push(key);
            }
        }

        let mut warnings = Vec::new();
        if n_nodes == 0 {
            warnings.push("Model has no nodes.".to_string());
        }
        if n_members == 0 {
            warnings.push("Model has no members.".to_string());
        }
        if n_nodes > 0 && n_supports == 0 {
            warnings.push(
                "No supports defined \u{2014} the structure is unconstrained and the solve \
                 will fail."
                    .to_string(),
            );
        }

        Self {
            n_nodes,
            n_members,
            total_dof,
            constrained_dof,
            n_supports,
            n_nodal_loads,
            n_member_loads,
            n_materials: materials.len(),
            warnings,
        }
    }

    /// Number of unconstrained (free) DOFs.
    pub fn free_dof(&self) -> usize {
        self.total_dof.saturating_sub(self.constrained_dof)
    }

    /// `true` if the model has no nodes.
    pub fn is_empty(&self) -> bool {
        self.n_nodes == 0
    }
}

/// Read-only headline figures from a successful analysis result.
///
/// The 3D frame solver returns displacements, reactions and member end
/// forces only — it exposes **no** equilibrium report, iteration count or
/// residual, so none are shown here.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ResultsSummary {
    pub n_nodes: usize,
    pub n_members: usize,
    pub total_dof: usize,
    pub constrained_dof: usize,
    /// Largest translational displacement magnitude `max |u|` (m).
    pub max_translation: f64,
    /// Largest rotational displacement magnitude `max |\u{03b8}|` (rad).
    pub max_rotation: f64,
    /// Largest absolute reaction component (N for forces, N\u{00b7}m for moments).
    pub max_reaction: f64,
}

impl ResultsSummary {
    /// Build a summary from an analysis result snapshot.
    pub fn from_result(result: &FrameAnalysisResult3D) -> Self {
        let n_nodes = result.n_nodes;
        let translations = display::displacement_magnitudes(&result.displacements, n_nodes);
        let max_translation = display::max_abs(&translations);

        let mut max_rotation = 0.0_f64;
        for i in 0..n_nodes {
            let base = i * 6 + 3;
            if base + 2 < result.displacements.len() {
                let r = (result.displacements[base] * result.displacements[base]
                    + result.displacements[base + 1] * result.displacements[base + 1]
                    + result.displacements[base + 2] * result.displacements[base + 2])
                    .sqrt();
                if r.is_finite() {
                    max_rotation = max_rotation.max(r);
                }
            }
        }

        Self {
            n_nodes,
            n_members: result.n_members,
            total_dof: n_nodes * 6,
            constrained_dof: result.constrained_dofs.len(),
            max_translation,
            max_rotation,
            max_reaction: display::max_abs(&result.reactions),
        }
    }

    /// Number of unconstrained (free) DOFs.
    pub fn free_dof(&self) -> usize {
        self.total_dof.saturating_sub(self.constrained_dof)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model;

    #[test]
    fn empty_project() {
        let p = Project::empty();
        assert!(p.is_empty());
        assert!(!p.has_results());
        assert!(!p.results_are_stale());
        assert_eq!(p.analysis_status(), AnalysisStatus::NeverSolved);
    }

    #[test]
    fn default_is_empty() {
        let p = Project::default();
        assert!(p.is_empty());
    }

    #[test]
    fn load_model_discards_prior_results() {
        let mut p = Project::empty();
        p.load_model("Portal".into(), model::build_portal_edit());
        p.solve().unwrap();
        assert!(p.has_results());

        p.load_model("Inclined".into(), model::build_inclined_edit());
        assert!(!p.has_results());
        assert!(!p.results_are_stale());
        assert_eq!(p.analysis_status(), AnalysisStatus::NeverSolved);
    }

    #[test]
    fn load_model_without_prior_result() {
        let mut p = Project::empty();
        p.load_model("Portal".into(), model::build_portal_edit());
        assert!(!p.results_are_stale());
        assert!(!p.has_results());
        assert_eq!(p.analysis_status(), AnalysisStatus::NeverSolved);
    }

    #[test]
    fn solve_produces_current_result() {
        let mut p = Project::empty();
        p.load_model("Portal".into(), model::build_portal_edit());
        p.solve().unwrap();
        assert!(p.has_results());
        assert!(!p.results_are_stale());
        assert_eq!(p.analysis_status(), AnalysisStatus::Current);
    }

    #[test]
    fn solve_after_stale_produces_current() {
        let mut p = Project::empty();
        p.load_model("Portal".into(), model::build_portal_edit());
        p.solve().unwrap();
        // Edit the model → revision bumps → status becomes Stale.
        let id = p.model.as_ref().unwrap().nodes()[0].id;
        p.model
            .as_mut()
            .unwrap()
            .update_node(id, 1.0, 0.0, 0.0)
            .unwrap();
        assert_eq!(p.analysis_status(), AnalysisStatus::Stale);
        p.solve().unwrap();
        assert_eq!(p.analysis_status(), AnalysisStatus::Current);
    }

    #[test]
    fn solve_without_model_errors_and_sets_failed() {
        let mut p = Project::empty();
        assert!(p.solve().is_err());
        assert_eq!(p.analysis_status(), AnalysisStatus::Failed);
    }

    #[test]
    fn failed_solve_then_successful_solve_clears_failed() {
        let mut p = Project::empty();
        // Solve with no model → Failed.
        assert!(p.solve().is_err());
        assert_eq!(p.analysis_status(), AnalysisStatus::Failed);
        // Load a valid model and solve → Current.
        p.load_model("Portal".into(), model::build_portal_edit());
        p.solve().unwrap();
        assert_eq!(p.analysis_status(), AnalysisStatus::Current);
    }

    #[test]
    fn clear_resets_to_empty() {
        let mut p = Project::empty();
        p.load_model("Portal".into(), model::build_portal_edit());
        p.solve().unwrap();
        p.clear();
        assert!(p.is_empty());
        assert!(!p.has_results());
        assert_eq!(p.name, "Untitled");
        assert_eq!(p.analysis_status(), AnalysisStatus::NeverSolved);
    }

    #[test]
    fn analysis_result_policy_a_hides_non_current() {
        let mut p = Project::empty();
        p.load_model("Portal".into(), model::build_portal_edit());
        // NeverSolved → None.
        assert!(p.analysis_result().is_none());
        p.solve().unwrap();
        // Current → Some.
        assert!(p.analysis_result().is_some());
        // Edit → Stale → None (Policy A).
        let id = p.model.as_ref().unwrap().nodes()[0].id;
        p.model
            .as_mut()
            .unwrap()
            .update_node(id, 1.0, 0.0, 0.0)
            .unwrap();
        assert!(p.analysis_result().is_none());
    }

    // ── Edit-invalidation transitions ──

    /// Solve → edit → `Stale` (not `NeverSolved`), and the stale result is
    /// still hidden (Policy A).
    #[test]
    fn edit_after_solve_reports_stale() {
        let mut p = Project::empty();
        p.load_model("Portal".into(), model::build_portal_edit());
        p.solve().unwrap();
        assert_eq!(p.analysis_status(), AnalysisStatus::Current);

        // An engineering edit: mutate a clone (bumps the revision) and commit.
        let mut edited = p.model.as_ref().unwrap().clone();
        let id = edited.nodes()[0].id;
        edited.update_node(id, 1.0, 0.0, 0.0).unwrap();
        assert_ne!(edited.revision(), p.model.as_ref().unwrap().revision());
        p.replace_model_after_edit(edited);

        assert_eq!(p.analysis_status(), AnalysisStatus::Stale);
        assert!(p.results_are_stale());
        assert!(!p.has_results());
        // Policy A: a stale result is never displayed.
        assert!(p.analysis_result().is_none());
    }

    /// Fresh model → edit → `NeverSolved` (nothing to retain).
    #[test]
    fn edit_before_any_solve_stays_never_solved() {
        let mut p = Project::empty();
        p.load_model("Portal".into(), model::build_portal_edit());
        assert_eq!(p.analysis_status(), AnalysisStatus::NeverSolved);

        let mut edited = p.model.as_ref().unwrap().clone();
        let id = edited.nodes()[0].id;
        edited.update_node(id, 1.0, 0.0, 0.0).unwrap();
        p.replace_model_after_edit(edited);

        assert_eq!(p.analysis_status(), AnalysisStatus::NeverSolved);
        assert!(!p.results_are_stale());
        assert!(p.analysis_result().is_none());
    }

    /// Solve → edit → solve → `Current`.
    #[test]
    fn solving_after_an_edit_returns_to_current() {
        let mut p = Project::empty();
        p.load_model("Portal".into(), model::build_portal_edit());
        p.solve().unwrap();

        let mut edited = p.model.as_ref().unwrap().clone();
        let id = edited.nodes()[0].id;
        edited.update_node(id, 1.0, 0.0, 0.0).unwrap();
        p.replace_model_after_edit(edited);
        assert_eq!(p.analysis_status(), AnalysisStatus::Stale);

        p.solve().unwrap();
        assert_eq!(p.analysis_status(), AnalysisStatus::Current);
        assert!(p.analysis_result().is_some());
    }

    #[test]
    fn message_constructors() {
        let m = AppMessage::info("hello");
        assert_eq!(m.level, MessageLevel::Info);
        assert_eq!(m.text, "hello");

        let m = AppMessage::warning("careful");
        assert_eq!(m.level, MessageLevel::Warning);

        let m = AppMessage::error("broken");
        assert_eq!(m.level, MessageLevel::Error);
    }

    #[test]
    fn message_clone() {
        let m = AppMessage::info("test");
        let m2 = m.clone();
        assert_eq!(m.level, m2.level);
        assert_eq!(m.text, m2.text);
    }

    // ── Solver options ──

    #[test]
    fn solver_backend_default_is_auto() {
        let p = Project::empty();
        assert_eq!(p.solver_backend(), SolverBackend::Auto);
        assert_eq!(SolverBackend::default(), SolverBackend::Auto);
    }

    #[test]
    fn solver_backend_names_and_selection() {
        assert_eq!(SolverBackend::Auto.name(), None);
        assert_eq!(SolverBackend::Dense.name(), Some("dense"));
        assert_eq!(SolverBackend::SkylineLdlt.name(), Some("skyline_ldlt"));
        assert_eq!(SolverBackend::SparseLu.name(), Some("sparse_lu"));
        assert_eq!(SolverBackend::Cg.name(), Some("cg"));
        assert_eq!(SolverBackend::Iccg.name(), Some("iccg"));

        assert_eq!(SolverBackend::Auto.to_selection(), SolverSelection::Auto);
        assert_eq!(
            SolverBackend::SparseLu.to_selection(),
            SolverSelection::named("sparse_lu")
        );
        // ALL has one entry per backend, all labels non-empty.
        assert_eq!(SolverBackend::ALL.len(), 6);
        for b in SolverBackend::ALL {
            assert!(!b.label().is_empty());
        }
    }

    #[test]
    fn changing_solver_backend_marks_results_stale() {
        let mut p = Project::empty();
        p.load_model("Portal".into(), model::build_portal_edit());
        p.solve().unwrap();
        assert_eq!(p.analysis_status(), AnalysisStatus::Current);

        p.set_solver_backend(SolverBackend::SparseLu);
        assert_eq!(p.analysis_status(), AnalysisStatus::Stale);
        assert!(p.analysis_result().is_none());

        // Re-solving with the new backend restores Current.
        p.solve().unwrap();
        assert_eq!(p.analysis_status(), AnalysisStatus::Current);
        assert_eq!(p.solved_backend(), Some(SolverBackend::SparseLu));
    }

    #[test]
    fn reset_solver_options_restores_default() {
        let mut p = Project::empty();
        p.set_solver_backend(SolverBackend::Cg);
        p.reset_solver_options();
        assert_eq!(p.solver_backend(), SolverBackend::Auto);
    }

    #[test]
    fn solve_records_solved_backend() {
        let mut p = Project::empty();
        p.load_model("Portal".into(), model::build_portal_edit());
        assert_eq!(p.solved_backend(), None);
        p.solve().unwrap();
        assert_eq!(p.solved_backend(), Some(SolverBackend::Auto));
    }

    // ── Error surfacing ──

    #[test]
    fn failed_solve_records_and_clears_error() {
        let mut p = Project::empty();
        // No model → Failed with a stored message (never a silent zero).
        assert!(p.solve().is_err());
        assert!(p.last_solve_error().is_some());
        assert_eq!(p.analysis_status(), AnalysisStatus::Failed);

        // A successful solve clears the stored error.
        p.load_model("Portal".into(), model::build_portal_edit());
        p.solve().unwrap();
        assert!(p.last_solve_error().is_none());
    }

    // ── Model summary ──

    #[test]
    fn model_summary_portal() {
        let m = model::build_portal_edit();
        let s = ModelSummary::from_model(&m);
        assert_eq!(s.n_nodes, 4);
        assert_eq!(s.n_members, 3);
        assert_eq!(s.total_dof, 24);
        assert_eq!(s.constrained_dof, 12); // two fixed nodes × 6 DOF
        assert_eq!(s.free_dof(), 12);
        assert_eq!(s.n_supports, 2);
        assert_eq!(s.n_nodal_loads, 1);
        assert_eq!(s.n_member_loads, 0);
        assert_eq!(s.n_materials, 1);
        assert!(s.warnings.is_empty());
        assert!(!s.is_empty());
    }

    #[test]
    fn model_summary_empty_warns() {
        let s = ModelSummary::from_model(&EditModel::new());
        assert!(s.is_empty());
        assert_eq!(s.total_dof, 0);
        // "no nodes" + "no members" (no unsupported warning without nodes).
        assert_eq!(s.warnings.len(), 2);
    }

    // ── Results summary ──

    #[test]
    fn results_summary_portal() {
        let mut p = Project::empty();
        p.load_model("Portal".into(), model::build_portal_edit());
        p.solve().unwrap();
        let r = p.analysis_result().unwrap();
        let s = ResultsSummary::from_result(r);
        assert_eq!(s.n_nodes, 4);
        assert_eq!(s.n_members, 3);
        assert_eq!(s.total_dof, 24);
        assert_eq!(s.constrained_dof, 12);
        assert_eq!(s.free_dof(), 12);
        assert!(s.max_translation > 0.0);
        assert!(s.max_reaction > 0.0);
    }

    #[test]
    fn results_summary_matches_model_counts() {
        let mut p = Project::empty();
        p.load_model("Inclined".into(), model::build_inclined_edit());
        p.solve().unwrap();
        let m = ModelSummary::from_model(p.model.as_ref().unwrap());
        let r = ResultsSummary::from_result(p.analysis_result().unwrap());
        assert_eq!(r.n_nodes, m.n_nodes);
        assert_eq!(r.n_members, m.n_members);
        assert_eq!(r.total_dof, m.total_dof);
        assert_eq!(r.constrained_dof, m.constrained_dof);
    }
}
