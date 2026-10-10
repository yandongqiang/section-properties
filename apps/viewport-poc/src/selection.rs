//! Selection state for the viewport.
//!
//! Uses stable [`NodeId`] / [`MemberId`] identifiers that survive model
//! edits (node/member deletion and re-creation).

use crate::edit_model::{MemberId, NodeId};

/// Currently selected entity.
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
}
