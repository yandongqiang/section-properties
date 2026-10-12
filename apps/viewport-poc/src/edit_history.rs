//! Bounded undo/redo history for engineering model edits.
//!
//! Uses a snapshot-based approach: each committed edit stores a clone of
//! the `EditModel` *before* the edit.  Undo restores the most recent
//! snapshot; redo reverses the undo.  The history is bounded by a
//! capacity (default 50); the oldest entry is discarded when full.
//!
//! # Revision invariant
//!
//! Every restored snapshot receives a new runtime revision (via
//! [`EditModel::ensure_revision_at_least`]) so that stale analysis
//! results are never mistaken for current.  The revision counter is
//! monotonic across the entire edit session, including undo/redo.

use crate::edit_model::EditModel;

/// Bounded snapshot-based edit history.
pub struct EditHistory {
    undo: Vec<EditModel>,
    redo: Vec<EditModel>,
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

    /// Record a model transition.
    ///
    /// `previous` is the model state *before* the edit.  The redo stack
    /// is cleared because the edit branch has diverged.
    pub fn record(&mut self, previous: EditModel) {
        if self.undo.len() >= self.capacity && self.capacity > 0 {
            self.undo.remove(0);
        }
        if self.capacity > 0 {
            self.undo.push(previous);
            self.redo.clear();
        }
    }

    /// Undo: pop the most recent snapshot from the undo stack and push
    /// `current` onto the redo stack.  Returns the model to restore, or
    /// `None` if there is nothing to undo.
    pub fn undo(&mut self, current: EditModel) -> Option<EditModel> {
        let previous = self.undo.pop()?;
        self.redo.push(current);
        Some(previous)
    }

    /// Redo: pop the most recent snapshot from the redo stack and push
    /// `current` onto the undo stack.  Returns the model to restore, or
    /// `None` if there is nothing to redo.
    pub fn redo(&mut self, current: EditModel) -> Option<EditModel> {
        let next = self.redo.pop()?;
        self.undo.push(current);
        Some(next)
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
}
