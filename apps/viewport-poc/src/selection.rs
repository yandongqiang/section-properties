//! Selection state for the viewport.

/// Currently selected entity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SelectionState {
    /// Nothing selected.
    None,
    /// Node selected by index.
    Node(usize),
    /// Member selected by index.
    Member(usize),
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

    /// Selected node index, or `None`.
    pub fn node_idx(&self) -> Option<usize> {
        match self {
            Self::Node(i) => Some(*i),
            _ => None,
        }
    }

    /// Selected member index, or `None`.
    pub fn member_idx(&self) -> Option<usize> {
        match self {
            Self::Member(i) => Some(*i),
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
        let s = SelectionState::Node(3);
        assert!(s.is_node());
        assert!(!s.is_member());
        assert_eq!(s.node_idx(), Some(3));
        assert_eq!(s.member_idx(), None);
    }

    #[test]
    fn member_helpers() {
        let s = SelectionState::Member(5);
        assert!(!s.is_node());
        assert!(s.is_member());
        assert_eq!(s.node_idx(), None);
        assert_eq!(s.member_idx(), Some(5));
    }
}
