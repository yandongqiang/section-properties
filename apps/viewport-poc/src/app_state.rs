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

// ── Analysis result ────────────────────────────────────────────────────

/// Analysis result with staleness tracking.
///
/// When the model is edited, the existing result is marked stale rather than
/// discarded, so the UI can show "results are outdated" and the user can
/// decide whether to re-solve.
pub struct AnalysisResult {
    pub result: FrameAnalysisResult3D,
    pub is_stale: bool,
}

impl AnalysisResult {
    pub fn fresh(result: FrameAnalysisResult3D) -> Self {
        Self {
            result,
            is_stale: false,
        }
    }

    pub fn mark_stale(&mut self) {
        self.is_stale = true;
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
        }
    }

    /// `true` if no model is loaded.
    pub fn is_empty(&self) -> bool {
        self.model.is_none()
    }

    /// `true` if analysis results are available (may be stale).
    pub fn has_results(&self) -> bool {
        self.analysis.is_some()
    }

    /// `true` if results exist and are marked stale.
    pub fn results_are_stale(&self) -> bool {
        self.analysis.as_ref().map_or(false, |a| a.is_stale)
    }

    /// Load a new model, replacing any current model.
    ///
    /// Existing analysis results are marked **stale** (not discarded) so the
    /// UI can inform the user that the displayed results correspond to the
    /// previous model.
    pub fn load_model(&mut self, name: String, model: EditModel) {
        if let Some(ref mut a) = self.analysis {
            a.mark_stale();
        }
        self.name = name;
        self.model = Some(model);
    }

    /// Run the FEM analysis on the current model.
    ///
    /// On success, stores a fresh (non-stale) result.  This is the only way
    /// to produce a new analysis result — the GUI must not fabricate results.
    pub fn solve(&mut self) -> Result<(), String> {
        let edit = self.model.as_ref().ok_or("No model loaded")?;
        let frame_model = edit.to_frame_model().map_err(|e| e.message())?;
        let result = frame_model.solve().map_err(|e| format!("{e:?}"))?;
        self.analysis = Some(AnalysisResult::fresh(result));
        Ok(())
    }

    /// Mark the analysis result as stale (e.g. after a model edit).
    pub fn mark_analysis_stale(&mut self) {
        if let Some(ref mut a) = self.analysis {
            a.mark_stale();
        }
    }

    /// Clear the project to the empty state.
    pub fn clear(&mut self) {
        self.name = "Untitled".to_string();
        self.model = None;
        self.analysis = None;
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
    }

    #[test]
    fn default_is_empty() {
        let p = Project::default();
        assert!(p.is_empty());
    }

    #[test]
    fn load_model_marks_existing_results_stale() {
        let mut p = Project::empty();
        p.load_model("Portal".into(), model::build_portal_edit());
        p.solve().unwrap();
        assert!(!p.results_are_stale());

        p.load_model("Inclined".into(), model::build_inclined_edit());
        assert!(p.results_are_stale());
        assert!(p.has_results());
    }

    #[test]
    fn load_model_without_prior_result_does_not_mark_stale() {
        let mut p = Project::empty();
        p.load_model("Portal".into(), model::build_portal_edit());
        assert!(!p.results_are_stale());
        assert!(!p.has_results());
    }

    #[test]
    fn solve_produces_fresh_result() {
        let mut p = Project::empty();
        p.load_model("Portal".into(), model::build_portal_edit());
        p.solve().unwrap();
        assert!(p.has_results());
        assert!(!p.results_are_stale());
    }

    #[test]
    fn solve_after_stale_produces_fresh() {
        let mut p = Project::empty();
        p.load_model("Portal".into(), model::build_portal_edit());
        p.solve().unwrap();
        p.load_model("Inclined".into(), model::build_inclined_edit());
        assert!(p.results_are_stale());
        p.solve().unwrap();
        assert!(!p.results_are_stale());
    }

    #[test]
    fn solve_without_model_errors() {
        let mut p = Project::empty();
        assert!(p.solve().is_err());
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
