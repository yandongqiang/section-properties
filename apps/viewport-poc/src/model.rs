//! Deterministic test models and synthetic generators.

use structural_analysis::{Dof3D, FrameModel3D, FrameSection3D};

use crate::edit_model::EditModel;

/// Available preset models for the model browser.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PresetModel {
    Portal,
    Inclined,
    Grid1x1,
    Grid3x3,
    Grid10x10,
}

impl PresetModel {
    /// All available presets, in display order.
    pub fn all() -> &'static [PresetModel] {
        &[
            PresetModel::Portal,
            PresetModel::Inclined,
            PresetModel::Grid1x1,
            PresetModel::Grid3x3,
            PresetModel::Grid10x10,
        ]
    }

    /// Human-readable name.
    pub fn name(&self) -> &'static str {
        match self {
            PresetModel::Portal => "Portal Frame",
            PresetModel::Inclined => "Inclined Frame",
            PresetModel::Grid1x1 => "Grid 1x1",
            PresetModel::Grid3x3 => "Grid 3x3",
            PresetModel::Grid10x10 => "Grid 10x10",
        }
    }

    /// Build the structural model.
    pub fn build(&self) -> FrameModel3D {
        match self {
            PresetModel::Portal => build_portal_frame(),
            PresetModel::Inclined => build_inclined_frame(),
            PresetModel::Grid1x1 => build_grid_frame(1, 1),
            PresetModel::Grid3x3 => build_grid_frame(3, 3),
            PresetModel::Grid10x10 => build_grid_frame(10, 10),
        }
    }

    /// Build the editable model.
    pub fn build_edit(&self) -> EditModel {
        match self {
            PresetModel::Portal => build_portal_edit(),
            PresetModel::Inclined => build_inclined_edit(),
            PresetModel::Grid1x1 => build_grid_edit(1, 1),
            PresetModel::Grid3x3 => build_grid_edit(3, 3),
            PresetModel::Grid10x10 => build_grid_edit(10, 10),
        }
    }
}

/// Steel I-section properties.
fn steel_section() -> FrameSection3D {
    FrameSection3D::new(200e9, 80e9, 1e-3, 2e-6, 1e-6, 3e-6).unwrap()
}

/// Build a deterministic 3-member portal frame.
///
/// ```text
///   N2 ──────── N3
///   │           │
///   │           │
///   N0          N1
/// ```
///
/// - Columns: 3 m tall, along Y.
/// - Beam: 4 m wide, along X.
/// - Bases (N0, N1) fully fixed.
/// - Horizontal load at N2.
pub fn build_portal_frame() -> FrameModel3D {
    let mut m = FrameModel3D::new();
    let n0 = m.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = m.add_node(4.0, 0.0, 0.0).unwrap();
    let n2 = m.add_node(0.0, 3.0, 0.0).unwrap();
    let n3 = m.add_node(4.0, 3.0, 0.0).unwrap();

    let sec = steel_section();
    m.add_member(n0, n2, sec, [0.0, 0.0, 1.0]).unwrap();
    m.add_member(n2, n3, sec, [0.0, 0.0, 1.0]).unwrap();
    m.add_member(n1, n3, sec, [0.0, 0.0, 1.0]).unwrap();

    m.fix_node(n0).unwrap();
    m.fix_node(n1).unwrap();

    m.add_nodal_load(n2, 5e3, 0.0, 0.0, 0.0, 0.0, 0.0).unwrap();

    m
}

/// Build a 3-D frame with an inclined member to exercise general orientation.
///
/// ```text
///   N2 ─── N3
///   │      │
///   │      │
///   N0     N1
///         /
///        N4 (offset in Z)
/// ```
pub fn build_inclined_frame() -> FrameModel3D {
    let mut m = FrameModel3D::new();
    let n0 = m.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = m.add_node(4.0, 0.0, 0.0).unwrap();
    let n2 = m.add_node(0.0, 3.0, 0.0).unwrap();
    let n3 = m.add_node(4.0, 3.0, 0.0).unwrap();
    let n4 = m.add_node(5.0, 0.0, 1.0).unwrap();

    let sec = steel_section();
    m.add_member(n0, n2, sec, [0.0, 0.0, 1.0]).unwrap();
    m.add_member(n2, n3, sec, [0.0, 0.0, 1.0]).unwrap();
    m.add_member(n1, n3, sec, [0.0, 0.0, 1.0]).unwrap();
    m.add_member(n1, n4, sec, [0.3, 0.7, 0.5]).unwrap();

    m.fix_node(n0).unwrap();
    m.fix_node(n1).unwrap();
    m.fix_dof(n4, Dof3D::Ux, 0.0).unwrap();
    m.fix_dof(n4, Dof3D::Uy, 0.0).unwrap();
    m.fix_dof(n4, Dof3D::Uz, 0.0).unwrap();

    m.add_nodal_load(n2, 3e3, -2e3, 0.0, 0.0, 0.0, 0.0).unwrap();

    m
}

/// Build a synthetic grid frame for performance testing.
///
/// Creates `nx × nz` bays of columns and beams.  Total nodes ≈
/// `(nx+1) * (nz+1) * 2`, total members ≈ `nx * (nz+1) * 2 + (nx+1) * nz`.
pub fn build_grid_frame(nx: usize, nz: usize) -> FrameModel3D {
    let mut m = FrameModel3D::new();
    let sec = steel_section();
    let bay = 4.0;
    let height = 3.0;

    let mut bottom = vec![vec![0usize; nz + 1]; nx + 1];
    let mut top = vec![vec![0usize; nz + 1]; nx + 1];

    for i in 0..=nx {
        for k in 0..=nz {
            let x = i as f64 * bay;
            let z = k as f64 * bay;
            bottom[i][k] = m.add_node(x, 0.0, z).unwrap();
            top[i][k] = m.add_node(x, height, z).unwrap();
        }
    }

    for i in 0..=nx {
        for k in 0..=nz {
            m.add_member(bottom[i][k], top[i][k], sec, [0.0, 0.0, 1.0])
                .unwrap();
        }
    }
    for i in 0..nx {
        for k in 0..=nz {
            m.add_member(top[i][k], top[i + 1][k], sec, [0.0, 0.0, 1.0])
                .unwrap();
        }
    }
    for i in 0..=nx {
        for k in 0..nz {
            m.add_member(top[i][k], top[i][k + 1], sec, [1.0, 0.0, 0.0])
                .unwrap();
        }
    }

    for i in 0..=nx {
        for k in 0..=nz {
            m.fix_node(bottom[i][k]).unwrap();
        }
    }

    if nx > 0 && nz > 0 {
        m.add_nodal_load(top[0][0], 1e3, 0.0, 0.0, 0.0, 0.0, 0.0)
            .unwrap();
    }

    m
}

/// Build a deterministic 3-member portal frame as an editable model.
pub fn build_portal_edit() -> EditModel {
    let mut m = EditModel::new();
    let n0 = m.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = m.add_node(4.0, 0.0, 0.0).unwrap();
    let n2 = m.add_node(0.0, 3.0, 0.0).unwrap();
    let n3 = m.add_node(4.0, 3.0, 0.0).unwrap();

    m.add_member(n0, n2).unwrap();
    m.add_member(n2, n3).unwrap();
    m.add_member(n1, n3).unwrap();

    m.fix_node(n0).unwrap();
    m.fix_node(n1).unwrap();

    m.add_nodal_load(n2, [5e3, 0.0, 0.0, 0.0, 0.0, 0.0])
        .unwrap();

    m
}

/// Build an inclined frame as an editable model.
pub fn build_inclined_edit() -> EditModel {
    let mut m = EditModel::new();
    let n0 = m.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = m.add_node(4.0, 0.0, 0.0).unwrap();
    let n2 = m.add_node(0.0, 3.0, 0.0).unwrap();
    let n3 = m.add_node(4.0, 3.0, 0.0).unwrap();
    let n4 = m.add_node(5.0, 0.0, 1.0).unwrap();

    m.add_member(n0, n2).unwrap();
    m.add_member(n2, n3).unwrap();
    m.add_member(n1, n3).unwrap();
    let m4 = m.add_member(n1, n4).unwrap();
    m.set_member_ref_vec(m4, [0.3, 0.7, 0.5]).unwrap();

    m.fix_node(n0).unwrap();
    m.fix_node(n1).unwrap();
    m.fix_dof(n4, Dof3D::Ux).unwrap();
    m.fix_dof(n4, Dof3D::Uy).unwrap();
    m.fix_dof(n4, Dof3D::Uz).unwrap();

    m.add_nodal_load(n2, [3e3, -2e3, 0.0, 0.0, 0.0, 0.0])
        .unwrap();

    m
}

/// Build a grid frame as an editable model.
pub fn build_grid_edit(nx: usize, nz: usize) -> EditModel {
    use crate::edit_model::NodeId;

    let mut m = EditModel::new();
    let bay = 4.0;
    let height = 3.0;

    let mut bottom = vec![vec![NodeId(0); nz + 1]; nx + 1];
    let mut top = vec![vec![NodeId(0); nz + 1]; nx + 1];

    for i in 0..=nx {
        for k in 0..=nz {
            let x = i as f64 * bay;
            let z = k as f64 * bay;
            bottom[i][k] = m.add_node(x, 0.0, z).unwrap();
            top[i][k] = m.add_node(x, height, z).unwrap();
        }
    }

    for i in 0..=nx {
        for k in 0..=nz {
            m.add_member(bottom[i][k], top[i][k]).unwrap();
        }
    }
    for i in 0..nx {
        for k in 0..=nz {
            m.add_member(top[i][k], top[i + 1][k]).unwrap();
        }
    }
    for i in 0..=nx {
        for k in 0..nz {
            let mid = m.add_member(top[i][k], top[i][k + 1]).unwrap();
            m.set_member_ref_vec(mid, [1.0, 0.0, 0.0]).unwrap();
        }
    }

    for i in 0..=nx {
        for k in 0..=nz {
            m.fix_node(bottom[i][k]).unwrap();
        }
    }

    if nx > 0 && nz > 0 {
        m.add_nodal_load(top[0][0], [1e3, 0.0, 0.0, 0.0, 0.0, 0.0])
            .unwrap();
    }

    m
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn portal_frame_structure() {
        let m = build_portal_frame();
        assert_eq!(m.n_nodes(), 4);
        assert_eq!(m.n_members(), 3);
    }

    #[test]
    fn inclined_frame_structure() {
        let m = build_inclined_frame();
        assert_eq!(m.n_nodes(), 5);
        assert_eq!(m.n_members(), 4);
    }

    #[test]
    fn grid_frame_node_count() {
        let m = build_grid_frame(2, 2);
        assert_eq!(m.n_nodes(), 18);
        assert!(m.n_members() > 0);
    }

    #[test]
    fn portal_frame_solves() {
        let m = build_portal_frame();
        let result = m.solve();
        assert!(
            result.is_ok(),
            "portal frame should solve: {:?}",
            result.err()
        );
    }

    #[test]
    fn inclined_frame_solves() {
        let m = build_inclined_frame();
        let result = m.solve();
        assert!(
            result.is_ok(),
            "inclined frame should solve: {:?}",
            result.err()
        );
    }

    #[test]
    fn grid_frame_solves() {
        let m = build_grid_frame(2, 2);
        let result = m.solve();
        assert!(
            result.is_ok(),
            "grid frame should solve: {:?}",
            result.err()
        );
    }

    #[test]
    fn preset_names_are_unique() {
        let names: Vec<&str> = PresetModel::all().iter().map(|p| p.name()).collect();
        for i in 0..names.len() {
            for j in (i + 1)..names.len() {
                assert_ne!(names[i], names[j], "duplicate preset name: {}", names[i]);
            }
        }
    }

    #[test]
    fn preset_all_builds_solve() {
        for &preset in PresetModel::all() {
            let m = preset.build();
            assert!(m.solve().is_ok(), "preset '{}' should solve", preset.name());
        }
    }

    #[test]
    fn preset_grid_10x10_node_count() {
        let m = PresetModel::Grid10x10.build();
        assert_eq!(m.n_nodes(), 242);
        assert_eq!(m.n_members(), 341);
    }

    #[test]
    fn portal_edit_structure() {
        let m = build_portal_edit();
        assert_eq!(m.n_nodes(), 4);
        assert_eq!(m.n_members(), 3);
    }

    #[test]
    fn inclined_edit_structure() {
        let m = build_inclined_edit();
        assert_eq!(m.n_nodes(), 5);
        assert_eq!(m.n_members(), 4);
    }

    #[test]
    fn grid_edit_node_count() {
        let m = build_grid_edit(2, 2);
        assert_eq!(m.n_nodes(), 18);
        assert!(m.n_members() > 0);
    }

    #[test]
    fn portal_edit_solves() {
        let m = build_portal_edit();
        let fm = m.to_frame_model().unwrap();
        assert!(fm.solve().is_ok());
    }

    #[test]
    fn inclined_edit_solves() {
        let m = build_inclined_edit();
        let fm = m.to_frame_model().unwrap();
        assert!(fm.solve().is_ok());
    }

    #[test]
    fn grid_edit_solves() {
        let m = build_grid_edit(2, 2);
        let fm = m.to_frame_model().unwrap();
        assert!(fm.solve().is_ok());
    }

    #[test]
    fn preset_all_builds_edit_solve() {
        for &preset in PresetModel::all() {
            let m = preset.build_edit();
            let fm = m.to_frame_model().unwrap_or_else(|e| {
                panic!(
                    "build_edit to_frame_model failed for '{}': {e:?}",
                    preset.name()
                )
            });
            assert!(
                fm.solve().is_ok(),
                "preset '{}' build_edit should solve",
                preset.name()
            );
        }
    }

    #[test]
    fn preset_grid_10x10_edit_node_count() {
        let m = PresetModel::Grid10x10.build_edit();
        assert_eq!(m.n_nodes(), 242);
        assert_eq!(m.n_members(), 341);
    }
}
