//! Application state types — project, analysis result, and messages.
//!
//! These types separate the user's engineering data (the [`Project`]) from
//! rendering concerns.  The GUI may inspect the project but must not mutate
//! analysis results directly; solving is an explicit operation via
//! [`Project::solve`].

use section_properties::SolverSelection;
use structural_analysis::{FrameAnalysisResult3D, FrameSolver3D};

use crate::display;
use crate::edit_model::{ComboTerm, EditModel, LoadCaseId};

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

/// Which solved result the display surfaces: one load case, or one
/// load combination (built by linear superposition of its cases).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ActiveResult {
    /// A load case's direct results (the default).
    #[default]
    Case,
    /// A load combination's superposed results.
    Combination,
}

/// Analysis result paired with the model revision and solver backend it was
/// produced from.
///
/// The [`solved_revision`](Self::solved_revision) is captured at solve
/// time so that subsequent edits can be detected by comparing it against
/// the model's current revision.  [`solved_backend`](Self::solved_backend)
/// records the requested solver so that changing the backend also marks the
/// result stale.
///
/// Every **load case** is solved independently (the stiffness matrix is
/// identical, only the load vector differs), and every **combination** is
/// assembled from them by linear superposition — exact for a linear-static
/// analysis and far cheaper than re-solving.
pub struct AnalysisResult {
    /// Per-case results, in model case order.
    pub cases: Vec<CaseResult>,
    /// Per-combination results, in model combination order.
    pub combinations: Vec<ComboResult>,
    solved_revision: u64,
    solved_backend: SolverBackend,
}

/// One load case's solved result.
pub struct CaseResult {
    /// The load case ID this result belongs to.
    pub case: LoadCaseId,
    /// Case name at solve time (for the result selector).
    pub name: String,
    /// The solved linear-static result.
    pub result: FrameAnalysisResult3D,
}

/// One load combination's superposed result.
pub struct ComboResult {
    /// The combination ID this result belongs to.
    pub combination: LoadCaseId,
    /// Combination name at solve time (for the result selector).
    pub name: String,
    /// The superposed result (`Σ factor · case result`).
    pub result: FrameAnalysisResult3D,
}

impl AnalysisResult {
    pub fn new(
        cases: Vec<CaseResult>,
        combinations: Vec<ComboResult>,
        solved_revision: u64,
        solved_backend: SolverBackend,
    ) -> Self {
        Self {
            cases,
            combinations,
            solved_revision,
            solved_backend,
        }
    }

    /// Model revision this result was solved from.
    pub fn solved_revision(&self) -> u64 {
        self.solved_revision
    }

    /// Solver backend that produced this result.
    pub fn solved_backend(&self) -> SolverBackend {
        self.solved_backend
    }

    /// A load case's result, if that case was solved.
    pub fn case_result(&self, case: LoadCaseId) -> Option<&FrameAnalysisResult3D> {
        self.cases
            .iter()
            .find(|c| c.case == case)
            .map(|c| &c.result)
    }

    /// A combination's result, if that combination was solved.
    pub fn combination_result(&self, combination: LoadCaseId) -> Option<&FrameAnalysisResult3D> {
        self.combinations
            .iter()
            .find(|c| c.combination == combination)
            .map(|c| &c.result)
    }

    /// `(case id, name)` pairs for the result selector, in case order.
    pub fn case_labels(&self) -> Vec<(LoadCaseId, String)> {
        self.cases
            .iter()
            .map(|c| (c.case, c.name.clone()))
            .collect()
    }

    /// `(combination id, name)` pairs for the result selector, in order.
    pub fn combination_labels(&self) -> Vec<(LoadCaseId, String)> {
        self.combinations
            .iter()
            .map(|c| (c.combination, c.name.clone()))
            .collect()
    }

    /// Superpose a combination's terms over the per-case results.
    ///
    /// Returns `None` when a term references a case that was not solved or
    /// the per-case results are topology-incompatible (they cannot be: all
    /// cases come from the same model revision).  Only the load-dependent
    /// vectors (`displacements`, `reactions`, `member_end_forces`) are scaled
    /// and summed; the model topology is copied from the first term.
    pub fn combine(cases: &[CaseResult], terms: &[ComboTerm]) -> Option<FrameAnalysisResult3D> {
        let mut iter = terms.iter();
        let first = iter.next()?;
        let base = cases.iter().find(|c| c.case == first.case)?;
        let mut out = base.result.clone();
        let scale = |v: &mut [f64], k: f64| {
            for x in v.iter_mut() {
                *x *= k;
            }
        };
        scale(&mut out.displacements, first.factor);
        scale(&mut out.reactions, first.factor);
        for f in out.member_end_forces.iter_mut() {
            for x in f.iter_mut() {
                *x *= first.factor;
            }
        }
        for term in iter {
            let src = cases.iter().find(|c| c.case == term.case)?;
            for (o, &s) in out
                .displacements
                .iter_mut()
                .zip(src.result.displacements.iter())
            {
                *o += term.factor * s;
            }
            for (o, &s) in out.reactions.iter_mut().zip(src.result.reactions.iter()) {
                *o += term.factor * s;
            }
            for (o, s) in out
                .member_end_forces
                .iter_mut()
                .zip(src.result.member_end_forces.iter())
            {
                for (x, &y) in o.iter_mut().zip(s.iter()) {
                    *x += term.factor * y;
                }
            }
        }
        Some(out)
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
    /// ID of the load case / combination whose results are displayed.
    active_result_id: LoadCaseId,
    /// Whether [`active_result_id`](Self::active_result_id) names a load
    /// case or a load combination.
    active_result_kind: ActiveResult,
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
            active_result_id: LoadCaseId(0),
            active_result_kind: ActiveResult::Case,
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
                if a.solved_revision == model.revision() && a.solved_backend == self.solver_backend
                {
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

    /// Which result the display currently surfaces: a load case or a
    /// load combination.
    pub fn active_result_kind(&self) -> ActiveResult {
        self.active_result_kind
    }

    /// ID of the displayed result (a `LoadCaseId` or a combination ID,
    /// depending on [`active_result_kind`](Self::active_result_kind)).
    pub fn active_result_id(&self) -> LoadCaseId {
        self.active_result_id
    }

    /// Select which **load case**'s results to display.
    ///
    /// Ignored (leaves the current selection untouched) when that case was
    /// not solved — the selection can never point at a result that does
    /// not exist.
    pub fn set_active_case(&mut self, case: LoadCaseId) {
        if self
            .analysis
            .as_ref()
            .and_then(|a| a.case_result(case))
            .is_none()
        {
            return;
        }
        self.active_result_kind = ActiveResult::Case;
        self.active_result_id = case;
    }

    /// Select which **load combination**'s results to display.
    ///
    /// Ignored when that combination was not solved.
    pub fn set_active_combination(&mut self, combination: LoadCaseId) {
        if self
            .analysis
            .as_ref()
            .and_then(|a| a.combination_result(combination))
            .is_none()
        {
            return;
        }
        self.active_result_kind = ActiveResult::Combination;
        self.active_result_id = combination;
    }

    /// Short label of the displayed result (e.g. `Case DL`, `Combo 1.35DL+1.4LL`).
    pub fn active_result_label(&self) -> Option<String> {
        let a = self.analysis.as_ref()?;
        Some(match self.active_result_kind {
            ActiveResult::Case => {
                let c = a.cases.iter().find(|c| c.case == self.active_result_id)?;
                format!("Case {}", c.name)
            }
            ActiveResult::Combination => {
                let c = a
                    .combinations
                    .iter()
                    .find(|c| c.combination == self.active_result_id)?;
                format!("Combo {}", c.name)
            }
        })
    }

    /// Borrow the current analysis result, or `None` if stale/failed/none.
    ///
    /// Implements display Policy A: only [`AnalysisStatus::Current`]
    /// results are returned.  Stale, failed, or absent results are hidden
    /// so the UI never displays outdated diagnostics.
    ///
    /// When results are current this returns the result selected by
    /// [`set_active_case`](Self::set_active_case) /
    /// [`set_active_combination`](Self::set_active_combination).
    pub fn analysis_result(&self) -> Option<&FrameAnalysisResult3D> {
        if self.analysis_status() != AnalysisStatus::Current {
            return None;
        }
        let a = self.analysis.as_ref()?;
        match self.active_result_kind {
            ActiveResult::Case => a.case_result(self.active_result_id),
            ActiveResult::Combination => a.combination_result(self.active_result_id),
        }
    }

    /// Load a new model, replacing any current model.
    ///
    /// Existing analysis results are discarded — the new model has its
    /// own revision counter starting at 0, so any prior result is
    /// meaningless.  The failed flag is also reset.
    pub fn load_model(&mut self, name: String, model: EditModel) {
        self.name = name;
        self.select_active_case(model.active_case());
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

        // Validate the load combinations up-front: a combination that
        // references a deleted load case can never be solved, and failing
        // here names the culprit instead of producing silently wrong sums.
        for combo in edit.combinations() {
            let missing = edit.dangling_combination_cases(combo.id);
            if !missing.is_empty() {
                let ids: Vec<String> = missing.iter().map(|m| format!("#{}", m.0)).collect();
                return Err(format!(
                    "load combination '{}' references missing load case(s): {}",
                    combo.name,
                    ids.join(", ")
                ));
            }
            if combo.terms.is_empty() {
                return Err(format!(
                    "load combination '{}' has no terms defined",
                    combo.name
                ));
            }
        }

        // Solve every load case.  The stiffness matrix is identical across
        // cases; only the load vector (and self weight) differs, so this is
        // a per-case re-solve rather than a full re-assembly each time.
        let mut cases: Vec<CaseResult> = Vec::with_capacity(edit.n_load_cases());
        for case in edit.load_cases() {
            let frame_model = edit
                .to_frame_model_for_case(case.id)
                .map_err(|e| e.message())?;
            let mut solver = FrameSolver3D::from_model(&frame_model).map_err(|e| format!("{e}"))?;
            solver.set_solver(backend.to_selection());
            let result = solver.solve().map_err(|e| format!("{e}"))?;
            cases.push(CaseResult {
                case: case.id,
                name: case.name.clone(),
                result,
            });
        }

        // Combinations by linear superposition (exact for linear analysis).
        let mut combinations: Vec<ComboResult> = Vec::with_capacity(edit.n_combinations());
        for combo in edit.combinations() {
            let result = AnalysisResult::combine(&cases, &combo.terms).ok_or_else(|| {
                format!(
                    "load combination '{}' references an unsolved load case",
                    combo.name
                )
            })?;
            combinations.push(ComboResult {
                combination: combo.id,
                name: combo.name.clone(),
                result,
            });
        }

        self.analysis = Some(AnalysisResult::new(cases, combinations, revision, backend));

        // Keep the display selection valid: if the previously selected case
        // or combination no longer exists in the fresh result, fall back to
        // the first solved load case.
        let still_present = match self.active_result_kind {
            ActiveResult::Case => matches!(
                self.analysis.as_ref(),
                Some(a) if a.case_result(self.active_result_id).is_some()
            ),
            ActiveResult::Combination => matches!(
                self.analysis.as_ref(),
                Some(a) if a.combination_result(self.active_result_id).is_some()
            ),
        };
        if !still_present {
            self.active_result_kind = ActiveResult::Case;
            self.active_result_id = self
                .analysis
                .as_ref()
                .and_then(|a| a.cases.first())
                .map(|c| c.case)
                .unwrap_or(LoadCaseId(0));
        }
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
        self.select_active_case(model.active_case());
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
        self.select_active_case(model.active_case());
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
        self.select_active_case(LoadCaseId(0));
    }

    /// Point the result selection at a load case (None if unsolved is a
    /// no-op — see [set_active_case](Self::set_active_case)).
    fn select_active_case(&mut self, case: LoadCaseId) {
        self.active_result_kind = ActiveResult::Case;
        self.active_result_id = case;
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
    /// Nodal loads in the **active** load case.
    pub n_nodal_loads: usize,
    /// Uniform distributed loads + member point loads in the active case.
    pub n_member_loads: usize,
    /// Number of load cases (always ≥ 1).
    pub n_load_cases: usize,
    /// Number of load combinations.
    pub n_combinations: usize,
    /// Whether the active load case includes member self weight.
    pub self_weight: bool,
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

        let n_nodal_loads = model.n_nodal_loads();
        let n_member_loads = model.n_member_uniform_loads() + model.member_point_loads().len();
        let n_load_cases = model.n_load_cases();
        let n_combinations = model.n_combinations();
        let self_weight = model
            .load_case(model.active_case())
            .map(|c| c.self_weight)
            .unwrap_or(false);

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
            n_load_cases,
            n_combinations,
            self_weight,
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
    use crate::edit_model::{ComboTerm, NodalLoadSpec};
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

    // ── Load cases & combinations ──

    /// Build a small portal with **two independent load cases** sharing one
    /// structure: DL carries the lateral load at the left top node, LL the
    /// vertical load at the right top node (distinct nodes so neither case's
    /// load can mask the other's).
    fn two_case_portal() -> EditModel {
        let mut m = model::build_portal_edit();
        let ll = m.add_load_case("LL").unwrap();
        let top_right = m
            .nodes()
            .iter()
            .rev()
            .find(|n| n.y > 0.0)
            .map(|n| n.id)
            .expect("portal has elevated nodes");
        m.set_active_case(ll).unwrap();
        m.set_nodal_load(top_right, NodalLoadSpec::new(0.0, -3e3, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        // Leave the model viewing its default case (the active case is view
        // state, not data).
        m.set_active_case(LoadCaseId(0)).unwrap();
        m
    }

    #[test]
    fn solve_stores_every_load_case() {
        let mut p = Project::empty();
        p.load_model("Portal".into(), two_case_portal());
        p.solve().unwrap();

        let a = p.analysis.as_ref().expect("result");
        assert_eq!(a.cases.len(), 2);
        assert_eq!(a.cases[0].name, "DL");
        assert_eq!(a.cases[1].name, "LL");
        // The two cases solve the same structure with different loads, so
        // the responses differ.
        assert_ne!(
            a.cases[0].result.displacements,
            a.cases[1].result.displacements
        );
        // The default display is the first load case.
        assert_eq!(p.active_result_kind(), ActiveResult::Case);
        assert_eq!(p.active_result_id(), LoadCaseId(0));
        assert_eq!(p.active_result_label().as_deref(), Some("Case DL"));
    }

    #[test]
    fn combination_result_is_the_weighted_sum_of_cases() {
        let mut p = Project::empty();
        p.load_model("Portal".into(), two_case_portal());
        p.solve().unwrap();

        let disps = |p: &Project, kind: ActiveResult, id: LoadCaseId| -> Vec<f64> {
            match kind {
                ActiveResult::Case => p
                    .analysis
                    .as_ref()
                    .unwrap()
                    .case_result(id)
                    .unwrap()
                    .displacements
                    .clone(),
                ActiveResult::Combination => p
                    .analysis
                    .as_ref()
                    .unwrap()
                    .combination_result(id)
                    .unwrap()
                    .displacements
                    .clone(),
            }
        };

        // Superposition identity: 1.35·DL + 1.4·LL.
        let a = p.analysis.as_ref().unwrap();
        let dl = a.case_result(LoadCaseId(0)).unwrap().displacements.clone();
        let ll = a.case_result(LoadCaseId(1)).unwrap().displacements.clone();
        assert_ne!(dl, ll);

        // Now add the combination to the model and re-solve.
        let mut m = two_case_portal();
        let combo = m.add_load_combination("1.35DL+1.4LL").unwrap();
        m.set_combination_terms(
            combo,
            vec![
                ComboTerm {
                    case: LoadCaseId(0),
                    factor: 1.35,
                },
                ComboTerm {
                    case: LoadCaseId(1),
                    factor: 1.4,
                },
            ],
        )
        .unwrap();
        p.load_model("Portal".into(), m);
        p.solve().unwrap();

        let combined = disps(&p, ActiveResult::Combination, combo);
        for (i, (c, (d, l))) in combined.iter().zip(dl.iter().zip(ll.iter())).enumerate() {
            let want = 1.35 * d + 1.4 * l;
            assert!(
                (c - want).abs() < 1e-9 * (1.0 + want.abs()),
                "combined[{i}] = {c}, want {want}"
            );
        }

        // The combination's reactions superpose the same way.
        let a = p.analysis.as_ref().unwrap();
        let r_dl = a.case_result(LoadCaseId(0)).unwrap().reactions.clone();
        let r_ll = a.case_result(LoadCaseId(1)).unwrap().reactions.clone();
        let r_combo = a.combination_result(combo).unwrap().reactions.clone();
        for (i, (c, (d, l))) in r_combo.iter().zip(r_dl.iter().zip(r_ll.iter())).enumerate() {
            assert!(
                (c - (1.35 * d + 1.4 * l)).abs() < 1e-9 * (1.0 + c.abs()),
                "reaction[{i}] = {c}"
            );
        }
    }

    #[test]
    fn combination_result_matches_single_solve_of_the_same_loads() {
        // Cross-check the superposition against a *direct* solve: a model
        // where both cases' loads sit in one case must produce the same
        // displacements as the 1.0·DL + 1.0·LL combination.
        let combined_model = two_case_portal();
        let ll = combined_model.load_cases()[1].id;
        let mut merged = combined_model.clone();
        merged.set_active_case(LoadCaseId(0)).unwrap();
        for (node, load) in combined_model.nodal_loads_in(ll) {
            merged
                .set_nodal_load(
                    node,
                    NodalLoadSpec::new(load.fx, load.fy, load.fz, 0.0, 0.0, 0.0),
                )
                .unwrap();
        }
        // Keep the combined load exactly what 1.0·DL + 1.0·LL means: DL's
        // own load is already in case 0 (portal preset) and LL's was merged
        // additively per node by `set_nodal_load` above.

        let mut p = Project::empty();
        p.load_model("Portal".into(), two_case_portal());
        let combo = {
            let m = p.model.as_mut().unwrap();
            let c = m.add_load_combination("1.0DL+1.0LL").unwrap();
            m.set_combination_terms(
                c,
                vec![
                    ComboTerm {
                        case: LoadCaseId(0),
                        factor: 1.0,
                    },
                    ComboTerm {
                        case: LoadCaseId(1),
                        factor: 1.0,
                    },
                ],
            )
            .unwrap();
            c
        };
        p.solve().unwrap();
        let combo_disps = p
            .analysis
            .as_ref()
            .unwrap()
            .combination_result(combo)
            .unwrap()
            .displacements
            .clone();

        let mut q = Project::empty();
        q.load_model("Portal".into(), merged);
        q.solve().unwrap();
        let merged_disps = q.analysis_result().unwrap().displacements.clone();

        for (i, (c, m)) in combo_disps.iter().zip(merged_disps.iter()).enumerate() {
            assert!(
                (c - m).abs() < 1e-9 * (1.0 + m.abs()),
                "superposed[{i}] = {c}, direct = {m}"
            );
        }
    }

    #[test]
    fn dangling_combination_fails_the_solve() {
        let mut m = two_case_portal();
        let combo = m.add_load_combination("1.4LL").unwrap();
        m.set_combination_terms(
            combo,
            vec![ComboTerm {
                case: LoadCaseId(1),
                factor: 1.4,
            }],
        )
        .unwrap();
        // Delete the referenced case → the combination dangles.
        m.delete_load_case(LoadCaseId(1)).unwrap();

        let mut p = Project::empty();
        p.load_model("Portal".into(), m);
        let err = p.solve().unwrap_err();
        assert!(
            err.contains("load combination") && err.contains("1.4LL"),
            "error should name the combination, got: {err}"
        );
        assert_eq!(p.analysis_status(), AnalysisStatus::Failed);
    }

    #[test]
    fn empty_combination_fails_the_solve() {
        let mut m = two_case_portal();
        m.add_load_combination("Empty").unwrap();
        let mut p = Project::empty();
        p.load_model("Portal".into(), m);
        let err = p.solve().unwrap_err();
        assert!(
            err.contains("no terms"),
            "error should explain the missing terms, got: {err}"
        );
    }

    #[test]
    fn selecting_a_case_changes_the_displayed_result() {
        let mut p = Project::empty();
        p.load_model("Portal".into(), two_case_portal());
        p.solve().unwrap();

        let dl = p.analysis_result().unwrap().displacements.clone();
        p.set_active_case(LoadCaseId(1));
        assert_eq!(p.active_result_id(), LoadCaseId(1));
        assert_eq!(p.active_result_label().as_deref(), Some("Case LL"));
        let ll = p.analysis_result().unwrap().displacements.clone();
        assert_ne!(dl, ll, "the displayed result must follow the selection");

        // Selecting an unsolved case is a no-op.
        p.set_active_case(LoadCaseId(77));
        assert_eq!(p.active_result_id(), LoadCaseId(1));
    }

    #[test]
    fn self_weight_option_changes_the_case_result() {
        let mut m = model::build_portal_edit();
        m.set_case_self_weight(LoadCaseId(0), true).unwrap();
        let mut p = Project::empty();
        p.load_model("Portal".into(), m);
        p.solve().unwrap();
        let with_sw = p.analysis_result().unwrap().displacements.clone();

        let mut plain = model::build_portal_edit();
        plain.set_case_self_weight(LoadCaseId(0), false).unwrap();
        let mut q = Project::empty();
        q.load_model("Portal".into(), plain);
        q.solve().unwrap();
        assert_ne!(
            with_sw,
            q.analysis_result().unwrap().displacements,
            "enabling self weight must change the dead-load response"
        );
    }

    #[test]
    fn model_summary_reports_cases() {
        let m = two_case_portal();
        let s = ModelSummary::from_model(&m);
        assert_eq!(s.n_load_cases, 2);
        assert_eq!(s.n_combinations, 0);
        assert!(!s.self_weight);
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
