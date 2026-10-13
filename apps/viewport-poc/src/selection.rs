//! Selection state for the viewport.
//!
//! Uses stable [`NodeId`] / [`MemberId`] identifiers that survive model
//! edits (node/member deletion and re-creation).
//!
//! Two representations live here:
//!
//! * [`SelectionState`] — the legacy **single**-entity view (`None` / `Node` /
//!   `Member`).  It is still the currency of the per-entity panels (the quick
//!   edit popup, the context menu, the section module, the force diagrams) and
//!   is the type every external call site already speaks.
//! * [`SelectionSet`] — the authoritative **multi**-selection.  It holds an
//!   ordered set of nodes and members plus a *primary* (last-clicked) entity,
//!   so a single-entity caller can keep working via [`SelectionSet::primary`]
//!   / [`SelectionSet::single`].

use crate::edit_model::{MemberId, NodeId};

/// Currently selected entity (legacy single-entity view).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SelectionState {
    /// Nothing selected.
    None,
    /// Node selected by stable ID.
    Node(NodeId),
    /// Member selected by stable ID.
    Member(MemberId),
}

impl Default for SelectionState {
    fn default() -> Self {
        Self::None
    }
}

impl SelectionState {
    /// Returns `true` if a node is selected.
    pub fn is_node(&self) -> bool {
        matches!(self, Self::Node(_))
    }

    /// Returns `true` if a member is selected.
    pub fn is_member(&self) -> bool {
        matches!(self, Self::Member(_))
    }

    /// Selected node ID, or `None`.
    pub fn node_id(&self) -> Option<NodeId> {
        match self {
            Self::Node(id) => Some(*id),
            _ => None,
        }
    }

    /// Selected member ID, or `None`.
    pub fn member_id(&self) -> Option<MemberId> {
        match self {
            Self::Member(id) => Some(*id),
            _ => None,
        }
    }
}

// ── Multi-selection ───────────────────────────────────────────────────

/// An ordered multi-selection of nodes and members.
///
/// Nodes and members are stored in insertion order (deduplicated); the
/// [`primary`](Self::primary) entity is the most recently added one and drives
/// every single-entity panel (inspector detail view, force diagrams, section
/// binding).  When exactly one entity is selected, [`single`](Self::single)
/// returns it and behaviour matches the former single-selection model exactly.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SelectionSet {
    nodes: Vec<NodeId>,
    members: Vec<MemberId>,
    primary: Option<SelectionState>,
}

impl SelectionSet {
    /// Build a set holding a single entity (`SelectionState::None` yields an
    /// empty set).
    pub fn from_single(state: SelectionState) -> Self {
        let mut set = Self::default();
        set.set_single(state);
        set
    }

    /// `true` when nothing is selected.
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty() && self.members.is_empty()
    }

    /// Total number of selected entities (nodes + members).
    pub fn len(&self) -> usize {
        self.nodes.len() + self.members.len()
    }

    /// Number of selected nodes.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Number of selected members.
    pub fn member_count(&self) -> usize {
        self.members.len()
    }

    /// Selected node IDs, in insertion order.
    pub fn nodes(&self) -> &[NodeId] {
        &self.nodes
    }

    /// Selected member IDs, in insertion order.
    pub fn members(&self) -> &[MemberId] {
        &self.members
    }

    /// Is this node selected?
    pub fn contains_node(&self, id: NodeId) -> bool {
        self.nodes.contains(&id)
    }

    /// Is this member selected?
    pub fn contains_member(&self, id: MemberId) -> bool {
        self.members.contains(&id)
    }

    /// Is this entity selected?  `SelectionState::None` is never contained.
    pub fn contains_state(&self, state: SelectionState) -> bool {
        match state {
            SelectionState::Node(id) => self.contains_node(id),
            SelectionState::Member(id) => self.contains_member(id),
            SelectionState::None => false,
        }
    }

    /// The most recently added entity, or `SelectionState::None` when empty.
    ///
    /// This is the "primary"/last-clicked entity that the single-entity panels
    /// consume.
    pub fn primary(&self) -> SelectionState {
        self.primary.unwrap_or(SelectionState::None)
    }

    /// The selected entity **only** when exactly one entity is selected.
    pub fn single(&self) -> Option<SelectionState> {
        if self.len() == 1 {
            match self.primary {
                Some(SelectionState::None) | None => None,
                other => other,
            }
        } else {
            None
        }
    }

    /// Remove every entity.
    pub fn clear(&mut self) {
        self.nodes.clear();
        self.members.clear();
        self.primary = None;
    }

    /// Replace the whole selection with a single entity (`None` clears).
    pub fn set_single(&mut self, state: SelectionState) {
        self.clear();
        self.add_state(state);
    }

    /// Add a node (idempotent); it becomes the primary.
    pub fn add_node(&mut self, id: NodeId) {
        if !self.nodes.contains(&id) {
            self.nodes.push(id);
        }
        self.primary = Some(SelectionState::Node(id));
    }

    /// Add a member (idempotent); it becomes the primary.
    pub fn add_member(&mut self, id: MemberId) {
        if !self.members.contains(&id) {
            self.members.push(id);
        }
        self.primary = Some(SelectionState::Member(id));
    }

    /// Add an entity (idempotent).  `None` is a no-op.
    pub fn add_state(&mut self, state: SelectionState) {
        match state {
            SelectionState::Node(id) => self.add_node(id),
            SelectionState::Member(id) => self.add_member(id),
            SelectionState::None => {}
        }
    }

    /// Remove an entity if present.  If it was the primary, the primary falls
    /// back to the last remaining entity.
    pub fn remove_state(&mut self, state: SelectionState) {
        match state {
            SelectionState::Node(id) => {
                self.nodes.retain(|&n| n != id);
            }
            SelectionState::Member(id) => {
                self.members.retain(|&m| m != id);
            }
            SelectionState::None => return,
        }
        if self.primary == Some(state) {
            self.primary = self.last();
        }
    }

    /// Add the entity if absent, remove it if present (Shift+click).
    pub fn toggle_state(&mut self, state: SelectionState) {
        if self.contains_state(state) {
            self.remove_state(state);
        } else {
            self.add_state(state);
        }
    }

    /// Extend with many entities (each becomes the primary in turn).
    pub fn extend_states(&mut self, states: impl IntoIterator<Item = SelectionState>) {
        for s in states {
            self.add_state(s);
        }
    }

    /// Iterate over the selected entities (nodes first, then members).
    pub fn iter(&self) -> impl Iterator<Item = SelectionState> + '_ {
        self.nodes
            .iter()
            .copied()
            .map(SelectionState::Node)
            .chain(self.members.iter().copied().map(SelectionState::Member))
    }

    /// Drop entities that fail their predicate (used to prune deleted IDs).
    /// The primary is kept if it survives, otherwise it falls back to the last
    /// remaining entity (or `None`).
    pub fn retain<FN, FM>(&mut self, node_ok: FN, member_ok: FM)
    where
        FN: Fn(NodeId) -> bool,
        FM: Fn(MemberId) -> bool,
    {
        self.nodes.retain(|&n| node_ok(n));
        self.members.retain(|&m| member_ok(m));
        match self.primary {
            Some(p) if self.contains_state(p) => {}
            _ => self.primary = self.last(),
        }
    }

    /// Last remaining entity (members take precedence, matching the
    /// `nodes → members` iteration order).
    fn last(&self) -> Option<SelectionState> {
        if let Some(&m) = self.members.last() {
            Some(SelectionState::Member(m))
        } else {
            self.nodes.last().copied().map(SelectionState::Node)
        }
    }
}

// ── Box-select hit test ───────────────────────────────────────────────

/// Largest a rubber-band drag may be and still count as a click (points).
pub const BOX_SELECT_CLICK_TOLERANCE: f64 = 4.0;

/// A screen-space axis-aligned rectangle (viewport-local points), normalised so
/// `min ≤ max` on both axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScreenRect {
    pub min: [f64; 2],
    pub max: [f64; 2],
}

impl ScreenRect {
    /// Build from two opposite corners (order independent).
    pub fn from_corners(a: [f64; 2], b: [f64; 2]) -> Self {
        Self {
            min: [a[0].min(b[0]), a[1].min(b[1])],
            max: [a[0].max(b[0]), a[1].max(b[1])],
        }
    }

    /// Width in points.
    pub fn width(&self) -> f64 {
        self.max[0] - self.min[0]
    }

    /// Height in points.
    pub fn height(&self) -> f64 {
        self.max[1] - self.min[1]
    }

    /// `true` when the rectangle is too small to be a deliberate box select —
    /// the caller should treat it as a click instead.
    pub fn is_click(&self) -> bool {
        self.width() < BOX_SELECT_CLICK_TOLERANCE && self.height() < BOX_SELECT_CLICK_TOLERANCE
    }

    /// Is the point inside (edges inclusive)?
    pub fn contains(&self, p: [f64; 2]) -> bool {
        p[0] >= self.min[0] && p[0] <= self.max[0] && p[1] >= self.min[1] && p[1] <= self.max[1]
    }

    /// A node whose projection `p` falls inside the rectangle is selected.
    pub fn hits_point(&self, p: [f64; 2]) -> bool {
        self.contains(p)
    }

    /// A member is selected when **both** projected endpoints fall inside the
    /// rectangle (its projection is treated as its two end points).
    pub fn hits_segment(&self, a: [f64; 2], b: [f64; 2]) -> bool {
        self.contains(a) && self.contains(b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_to_none() {
        assert_eq!(SelectionState::default(), SelectionState::None);
    }

    #[test]
    fn node_helpers() {
        let s = SelectionState::Node(NodeId(3));
        assert!(s.is_node());
        assert!(!s.is_member());
        assert_eq!(s.node_id(), Some(NodeId(3)));
        assert_eq!(s.member_id(), None);
    }

    #[test]
    fn member_helpers() {
        let s = SelectionState::Member(MemberId(5));
        assert!(!s.is_node());
        assert!(s.is_member());
        assert_eq!(s.node_id(), None);
        assert_eq!(s.member_id(), Some(MemberId(5)));
    }

    // ── SelectionSet ────────────────────────────────────────────────────

    #[test]
    fn set_defaults_to_empty() {
        let set = SelectionSet::default();
        assert!(set.is_empty());
        assert_eq!(set.len(), 0);
        assert_eq!(set.primary(), SelectionState::None);
        assert_eq!(set.single(), None);
    }

    #[test]
    fn set_single_roundtrip() {
        let set = SelectionSet::from_single(SelectionState::Node(NodeId(2)));
        assert_eq!(set.len(), 1);
        assert!(set.contains_node(NodeId(2)));
        assert_eq!(set.primary(), SelectionState::Node(NodeId(2)));
        assert_eq!(set.single(), Some(SelectionState::Node(NodeId(2))));

        // A `None` single selection is empty.
        let none = SelectionSet::from_single(SelectionState::None);
        assert!(none.is_empty());
        assert_eq!(none.single(), None);
    }

    #[test]
    fn set_add_is_idempotent_and_sets_primary() {
        let mut set = SelectionSet::default();
        set.add_node(NodeId(1));
        set.add_node(NodeId(1));
        set.add_member(MemberId(7));
        assert_eq!(set.node_count(), 1);
        assert_eq!(set.member_count(), 1);
        assert_eq!(set.len(), 2);
        // The most recently added entity is primary.
        assert_eq!(set.primary(), SelectionState::Member(MemberId(7)));
        // Order is preserved: nodes then members.
        let states: Vec<_> = set.iter().collect();
        assert_eq!(
            states,
            vec![
                SelectionState::Node(NodeId(1)),
                SelectionState::Member(MemberId(7))
            ]
        );
    }

    #[test]
    fn set_single_is_none_when_multiple() {
        let mut set = SelectionSet::default();
        set.add_node(NodeId(1));
        set.add_node(NodeId(2));
        assert_eq!(set.single(), None);
        // `primary` still resolves (last-clicked).
        assert_eq!(set.primary(), SelectionState::Node(NodeId(2)));
    }

    #[test]
    fn set_toggle_roundtrip() {
        let mut set = SelectionSet::default();
        set.toggle_state(SelectionState::Node(NodeId(3)));
        assert!(set.contains_node(NodeId(3)));
        set.toggle_state(SelectionState::Node(NodeId(3)));
        assert!(!set.contains_node(NodeId(3)));
        assert!(set.is_empty());
        assert_eq!(set.primary(), SelectionState::None);
    }

    #[test]
    fn set_remove_primary_falls_back() {
        let mut set = SelectionSet::default();
        set.add_node(NodeId(1));
        set.add_member(MemberId(2));
        assert_eq!(set.primary(), SelectionState::Member(MemberId(2)));
        set.remove_state(SelectionState::Member(MemberId(2)));
        // Falls back to the only remaining entity.
        assert_eq!(set.primary(), SelectionState::Node(NodeId(1)));
        assert_eq!(set.len(), 1);
    }

    #[test]
    fn set_remove_absent_is_noop() {
        let mut set = SelectionSet::default();
        set.add_node(NodeId(1));
        set.remove_state(SelectionState::Member(MemberId(9)));
        assert_eq!(set.len(), 1);
        assert_eq!(set.primary(), SelectionState::Node(NodeId(1)));
        // Removing `None` never touches anything.
        set.remove_state(SelectionState::None);
        assert_eq!(set.len(), 1);
    }

    #[test]
    fn set_clear() {
        let mut set = SelectionSet::default();
        set.add_node(NodeId(1));
        set.add_member(MemberId(2));
        set.clear();
        assert!(set.is_empty());
        assert_eq!(set.primary(), SelectionState::None);
    }

    #[test]
    fn set_set_single_replaces() {
        let mut set = SelectionSet::default();
        set.add_node(NodeId(1));
        set.add_member(MemberId(2));
        set.set_single(SelectionState::Member(MemberId(5)));
        assert_eq!(set.len(), 1);
        assert!(!set.contains_node(NodeId(1)));
        assert!(set.contains_member(MemberId(5)));
        assert_eq!(set.single(), Some(SelectionState::Member(MemberId(5))));
    }

    #[test]
    fn set_extend_states() {
        let mut set = SelectionSet::default();
        set.extend_states([
            SelectionState::Node(NodeId(1)),
            SelectionState::Member(MemberId(2)),
            SelectionState::Node(NodeId(1)),
        ]);
        assert_eq!(set.node_count(), 1);
        assert_eq!(set.member_count(), 1);
        assert_eq!(set.len(), 2);
    }

    #[test]
    fn set_retain_prunes_deleted_ids() {
        let mut set = SelectionSet::default();
        set.add_node(NodeId(1));
        set.add_node(NodeId(2));
        set.add_member(MemberId(3));
        set.add_member(MemberId(4));
        // Node 1 and member 4 were deleted; node 2 is primary.
        set.remove_state(SelectionState::Member(MemberId(4)));
        set.add_node(NodeId(2)); // re-add → primary = Node(2)
        assert_eq!(set.primary(), SelectionState::Node(NodeId(2)));

        set.retain(|n| n == NodeId(2), |m| m == MemberId(3));
        assert!(!set.contains_node(NodeId(1)));
        assert!(set.contains_node(NodeId(2)));
        assert!(set.contains_member(MemberId(3)));
        assert!(!set.contains_member(MemberId(4)));
        assert_eq!(set.primary(), SelectionState::Node(NodeId(2)));
    }

    #[test]
    fn set_retain_drops_deleted_primary() {
        let mut set = SelectionSet::default();
        set.add_node(NodeId(1));
        set.add_node(NodeId(2));
        // Primary is Node(2); delete it and keep Node(1).
        set.retain(|n| n == NodeId(1), |_| false);
        assert_eq!(set.len(), 1);
        assert_eq!(set.primary(), SelectionState::Node(NodeId(1)));
    }

    #[test]
    fn set_retain_to_empty_clears_primary() {
        let mut set = SelectionSet::default();
        set.add_node(NodeId(1));
        set.add_member(MemberId(2));
        set.retain(|_| false, |_| false);
        assert!(set.is_empty());
        assert_eq!(set.primary(), SelectionState::None);
    }

    #[test]
    fn set_contains_state() {
        let mut set = SelectionSet::default();
        set.add_node(NodeId(1));
        set.add_member(MemberId(2));
        assert!(set.contains_state(SelectionState::Node(NodeId(1))));
        assert!(set.contains_state(SelectionState::Member(MemberId(2))));
        assert!(!set.contains_state(SelectionState::Node(NodeId(9))));
        assert!(!set.contains_state(SelectionState::None));
    }

    // ── ScreenRect (box-select math) ────────────────────────────────────

    #[test]
    fn screen_rect_normalises_corners() {
        let r = ScreenRect::from_corners([10.0, 40.0], [30.0, 20.0]);
        assert_eq!(r.min, [10.0, 20.0]);
        assert_eq!(r.max, [30.0, 40.0]);
        assert_eq!(r.width(), 20.0);
        assert_eq!(r.height(), 20.0);
    }

    #[test]
    fn screen_rect_contains_inclusive() {
        let r = ScreenRect::from_corners([0.0, 0.0], [10.0, 10.0]);
        assert!(r.contains([0.0, 0.0]));
        assert!(r.contains([10.0, 10.0]));
        assert!(r.contains([5.0, 5.0]));
        assert!(!r.contains([-0.1, 5.0]));
        assert!(!r.contains([5.0, 10.1]));
    }

    #[test]
    fn screen_rect_hit_predicates() {
        let r = ScreenRect::from_corners([0.0, 0.0], [10.0, 10.0]);
        // Node: point inside.
        assert!(r.hits_point([5.0, 5.0]));
        assert!(!r.hits_point([15.0, 5.0]));
        // Member: both endpoints inside.
        assert!(r.hits_segment([1.0, 1.0], [9.0, 9.0]));
        assert!(!r.hits_segment([1.0, 1.0], [11.0, 9.0]));
        assert!(!r.hits_segment([1.0, 1.0], [5.0, -1.0]));
    }

    #[test]
    fn screen_rect_is_click_for_degenerate_boxes() {
        let r = ScreenRect::from_corners([5.0, 5.0], [6.0, 6.0]);
        assert!(r.is_click());
        let r = ScreenRect::from_corners([5.0, 5.0], [50.0, 6.0]);
        assert!(!r.is_click());
    }
}
