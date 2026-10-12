//! Application state types — project, analysis result, and messages.
//!
//! These types separate the user's engineering data (the [`Project`]) from
//! rendering concerns.  The GUI may inspect the project but must not mutate
//! analysis results directly; solving is an explicit operation via
//! [`Project::solve`].

use structural_analysis::FrameAnalysisResult3D;

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

// ── Analysis result ────────────────────────────────────────────────────

/// Analysis result paired with the model revision it was produced from.
///
/// The [`solved_revision`](Self::solved_revision) is captured at solve
/// time so that subsequent edits can be detected by comparing it against
/// the model's current revision.
pub struct AnalysisResult {
    pub result: FrameAnalysisResult3D,
    pub solved_revision: u64,
}

impl AnalysisResult {
    pub fn new(result: FrameAnalysisResult3D, solved_revision: u64) -> Self {
        Self {
            result,
            solved_revision,
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
    last_solve_failed: bool,
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
            last_solve_failed: false,
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
                if a.solved_revision == model.revision() {
                    AnalysisStatus::Current
                } else {
                    AnalysisStatus::Stale
                }
            }
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
        outcome
    }

    /// Internal: produce a new analysis result without touching the
    /// failed flag.  Only modifies `self.analysis` on success.
    fn run_analysis(&mut self) -> Result<(), String> {
        let edit = self.model.as_ref().ok_or("No model loaded")?;
        let revision = edit.revision();
        let frame_model = edit.to_frame_model().map_err(|e| e.message())?;
        let result = frame_model.solve().map_err(|e| format!("{e}"))?;
        self.analysis = Some(AnalysisResult::new(result, revision));
        Ok(())
    }

    /// Clear the project to the empty state.
    pub fn clear(&mut self) {
        self.name = "Untitled".to_string();
        self.model = None;
        self.analysis = None;
        self.last_solve_failed = false;
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
}
