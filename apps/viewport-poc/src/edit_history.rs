//! Bounded undo/redo history for engineering model edits.
//!
//! Uses a snapshot-based approach: each committed edit stores a clone of
//! the `EditModel` *before* the edit, tagged with a short human label
//! describing the edit (e.g. `"Add node N5"`, `"Delete member M2"`).  Undo
//! restores the most recent snapshot; redo reverses the undo.  The history is
//! bounded by a capacity (default 50); the oldest entry is discarded when
//! full.
//!
//! # Labels (Phase 159-B)
//!
//! The [`record_labeled`](EditHistory::record_labeled) entry point stores a
//! per-edit short description used by the bottom **History** dock tab.  The
//! legacy [`record`](EditHistory::record) keeps working (it records an empty
//! label), so older call sites and tests are unaffected.  Labels ride along
//! with their snapshots through undo/redo, so a redoable entry keeps the
//! description of the edit it would re-apply.
//!
//! # Revision invariant
//!
//! Every restored snapshot receives a new runtime revision (via
//! [`EditModel::ensure_revision_at_least`]) so that stale analysis
//! results are never mistaken for current.  The revision counter is
//! monotonic across the entire edit session, including undo/redo.

use crate::edit_model::EditModel;

/// One stored model transition: the model *before* the edit plus a short
/// label describing the edit that followed it.
struct Snapshot {
    model: EditModel,
    label: String,
}

/// Bounded snapshot-based edit history.
pub struct EditHistory {
    undo: Vec<Snapshot>,
    redo: Vec<Snapshot>,
    capacity: usize,
}

impl Default for EditHistory {
    fn default() -> Self {
        Self::new(DEFAULT_CAPACITY)
    }
}

/// Default history capacity.
pub const DEFAULT_CAPACITY: usize = 50;

impl EditHistory {
    /// Create a history with the given capacity.
    pub fn new(capacity: usize) -> Self {
        Self {
            undo: Vec::new(),
            redo: Vec::new(),
            capacity,
        }
    }

    /// Record a model transition with no label (legacy entry point).
    ///
    /// Kept for callers (and tests) that do not describe their edit; the
    /// History dock shows such entries as `"(edit)"`.
    pub fn record(&mut self, previous: EditModel) {
        self.record_labeled(previous, String::new());
    }

    /// Record a model transition together with a short label describing the
    /// edit (e.g. `"Add node N5"`).
    ///
    /// `previous` is the model state *before* the edit.  The redo stack is
    /// cleared because the edit branch has diverged.
    pub fn record_labeled(&mut self, previous: EditModel, label: impl Into<String>) {
        if self.capacity == 0 {
            return;
        }
        if self.undo.len() >= self.capacity {
            self.undo.remove(0);
        }
        self.undo.push(Snapshot {
            model: previous,
            label: label.into(),
        });
        self.redo.clear();
    }

    /// Undo: pop the most recent snapshot from the undo stack and push
    /// `current` onto the redo stack.  Returns the model to restore, or
    /// `None` if there is nothing to undo.
    ///
    /// The popped snapshot's label is carried onto the redo entry, so the
    /// edit can be described identically if it is re-applied.
    pub fn undo(&mut self, current: EditModel) -> Option<EditModel> {
        let snapshot = self.undo.pop()?;
        self.redo.push(Snapshot {
            model: current,
            label: snapshot.label,
        });
        Some(snapshot.model)
    }

    /// Redo: pop the most recent snapshot from the redo stack and push
    /// `current` onto the undo stack.  Returns the model to restore, or
    /// `None` if there is nothing to redo.
    pub fn redo(&mut self, current: EditModel) -> Option<EditModel> {
        let snapshot = self.redo.pop()?;
        self.undo.push(Snapshot {
            model: current,
            label: snapshot.label,
        });
        Some(snapshot.model)
    }

    /// `true` if undo is available.
    pub fn can_undo(&self) -> bool {
        !self.undo.is_empty()
    }

    /// `true` if redo is available.
    pub fn can_redo(&self) -> bool {
        !self.redo.is_empty()
    }

    /// Clear all history (undo and redo).
    pub fn clear(&mut self) {
        self.undo.clear();
        self.redo.clear();
    }

    /// Number of undo entries.
    pub fn undo_depth(&self) -> usize {
        self.undo.len()
    }

    /// Number of redo entries.
    pub fn redo_depth(&self) -> usize {
        self.redo.len()
    }

    /// Current capacity.
    pub fn capacity(&self) -> usize {
        self.capacity
    }

    // ── Label accessors (Phase 159-B) ───────────────────────────────────

    /// Short descriptions of the committed edits on the undo stack, **newest
    /// first**.
    pub fn undo_labels(&self) -> Vec<&str> {
        self.undo.iter().rev().map(|s| s.label.as_str()).collect()
    }

    /// Short descriptions of the undone edits available to redo, **most
    /// recently undone first** (i.e. the next redo first).
    pub fn redo_labels(&self) -> Vec<&str> {
        self.redo.iter().rev().map(|s| s.label.as_str()).collect()
    }

    /// Label of the edit that Undo would revert (the top of the undo stack).
    pub fn next_undo_label(&self) -> Option<&str> {
        self.undo.last().map(|s| s.label.as_str())
    }

    /// Label of the edit that Redo would re-apply (the top of the redo stack).
    pub fn next_redo_label(&self) -> Option<&str> {
        self.redo.last().map(|s| s.label.as_str())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_model(rev: u64) -> EditModel {
        let mut m = EditModel::new();
        m.ensure_revision_at_least(rev);
        m
    }

    #[test]
    fn empty_history_cannot_undo_or_redo() {
        let h = EditHistory::default();
        assert!(!h.can_undo());
        assert!(!h.can_redo());
    }

    #[test]
    fn record_makes_undo_available() {
        let mut h = EditHistory::default();
        h.record(make_model(0));
        assert!(h.can_undo());
        assert!(!h.can_redo());
    }

    #[test]
    fn record_clears_redo() {
        let mut h = EditHistory::default();
        h.record(make_model(0));
        h.record(make_model(1));
        let _ = h.undo(make_model(2));
        assert!(h.can_redo());
        h.record(make_model(3));
        assert!(!h.can_redo());
    }

    #[test]
    fn undo_restores_previous_and_enables_redo() {
        let mut h = EditHistory::default();
        let m0 = make_model(0);
        let m1 = make_model(1);
        h.record(m0.clone());
        let restored = h.undo(m1.clone()).unwrap();
        assert_eq!(restored.revision(), m0.revision());
        assert!(!h.can_undo());
        assert!(h.can_redo());
    }

    #[test]
    fn redo_restores_next_and_enables_undo() {
        let mut h = EditHistory::default();
        let m0 = make_model(0);
        let m1 = make_model(1);
        h.record(m0.clone());
        let _ = h.undo(m1.clone());
        let restored = h.redo(m0.clone()).unwrap();
        assert_eq!(restored.revision(), m1.revision());
        assert!(h.can_undo());
        assert!(!h.can_redo());
    }

    #[test]
    fn clear_empties_both_stacks() {
        let mut h = EditHistory::default();
        h.record(make_model(0));
        h.record(make_model(1));
        h.clear();
        assert!(!h.can_undo());
        assert!(!h.can_redo());
    }

    #[test]
    fn capacity_drops_oldest() {
        let mut h = EditHistory::new(3);
        h.record(make_model(0));
        h.record(make_model(1));
        h.record(make_model(2));
        h.record(make_model(3));
        assert_eq!(h.undo_depth(), 3);
        // The oldest (rev 0) should have been dropped.
        // Undo three times and verify we don't see rev 0.
        let r1 = h.undo(make_model(4)).unwrap();
        assert_eq!(r1.revision(), 3);
        let r2 = h.undo(make_model(4)).unwrap();
        assert_eq!(r2.revision(), 2);
        let r3 = h.undo(make_model(4)).unwrap();
        assert_eq!(r3.revision(), 1);
        assert!(!h.can_undo());
    }

    #[test]
    fn zero_capacity_never_records() {
        let mut h = EditHistory::new(0);
        h.record(make_model(0));
        assert!(!h.can_undo());
    }

    // ── Labels (Phase 159-B) ────────────────────────────────────────────

    #[test]
    fn labels_are_listed_newest_first() {
        let mut h = EditHistory::default();
        h.record_labeled(make_model(0), "Add node N5");
        h.record_labeled(make_model(1), "Delete member M2");
        h.record_labeled(make_model(2), "Apply section");
        assert_eq!(
            h.undo_labels(),
            vec!["Apply section", "Delete member M2", "Add node N5"]
        );
        assert_eq!(h.next_undo_label(), Some("Apply section"));
        assert_eq!(h.undo_depth(), 3);
    }

    #[test]
    fn labels_ride_along_through_undo_and_redo() {
        let mut h = EditHistory::default();
        h.record_labeled(make_model(0), "Add node N5");
        h.record_labeled(make_model(1), "Delete member M2");
        // Undo the most recent edit: it moves to the redo stack with its label.
        let _ = h.undo(make_model(2));
        assert_eq!(h.undo_labels(), vec!["Add node N5"]);
        assert_eq!(h.redo_labels(), vec!["Delete member M2"]);
        assert_eq!(h.next_redo_label(), Some("Delete member M2"));
        // Redo it: the label returns to the undo stack.
        let _ = h.redo(make_model(1));
        assert_eq!(h.undo_labels(), vec!["Delete member M2", "Add node N5"]);
        assert!(h.redo_labels().is_empty());
    }

    #[test]
    fn legacy_record_yields_an_empty_label() {
        let mut h = EditHistory::default();
        h.record(make_model(0));
        assert_eq!(h.undo_labels(), vec![""]);
        assert_eq!(h.next_undo_label(), Some(""));
    }

    #[test]
    fn labels_are_dropped_with_the_capacity() {
        let mut h = EditHistory::new(2);
        h.record_labeled(make_model(0), "a");
        h.record_labeled(make_model(1), "b");
        h.record_labeled(make_model(2), "c");
        // "a" (the oldest) is discarded.
        assert_eq!(h.undo_labels(), vec!["c", "b"]);
    }
}
