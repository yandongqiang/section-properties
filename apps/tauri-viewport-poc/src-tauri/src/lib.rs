//! Tauri viewport PoC — scene DTO and backend commands.
//!
//! The Rust side builds a structural model, solves it, and serialises the
//! complete result into a [`SceneDto`] that the TypeScript frontend renders
//! with Three.js.  The frontend never calls solver internals; all data crosses
//! the FFI boundary as plain serialisable structs.

use serde::Serialize;
use structural_analysis::{Dof3D, FrameAnalysisResult3D, FrameModel3D, FrameSection3D};

// ---------------------------------------------------------------------------
// DTO types
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize)]
pub struct Vec3Dto {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct NodeDto {
    pub id: usize,
    pub position: Vec3Dto,
    pub displacement: Vec3Dto,
    pub rotation: Vec3Dto,
    pub reaction_force: Vec3Dto,
    pub reaction_moment: Vec3Dto,
}

#[derive(Debug, Clone, Serialize)]
pub struct MemberDto {
    pub id: usize,
    pub node_i: usize,
    pub node_j: usize,
    pub ref_vec: Vec3Dto,
    pub local_x_axis: Vec3Dto,
    pub local_y_axis: Vec3Dto,
    pub local_z_axis: Vec3Dto,
    pub end_forces_local: [f64; 12],
    pub end_forces_global: [f64; 12],
}

#[derive(Debug, Clone, Serialize)]
pub struct SceneDto {
    pub nodes: Vec<NodeDto>,
    pub members: Vec<MemberDto>,
    pub n_nodes: usize,
    pub n_members: usize,
}

// ---------------------------------------------------------------------------
// Deterministic model builders (mirrors egui PoC model.rs)
// ---------------------------------------------------------------------------

fn steel_section() -> FrameSection3D {
    FrameSection3D::new(200e9, 80e9, 1e-3, 2e-6, 1e-6, 3e-6).unwrap()
}

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

#[allow(clippy::needless_range_loop)]
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
    for i in 0..nx {
        for k in 0..nz {
            m.add_member(top[i][k], top[i][k + 1], sec, [1.0, 0.0, 0.0])
                .unwrap();
        }
    }

    for i in 0..nx {
        for k in 0..nz {
            m.fix_node(bottom[i][k]).unwrap();
        }
    }

    if nx > 0 && nz > 0 {
        m.add_nodal_load(top[0][0], 1e3, 0.0, 0.0, 0.0, 0.0, 0.0)
            .unwrap();
    }

    m
}

// ---------------------------------------------------------------------------
// Scene DTO construction
// ---------------------------------------------------------------------------

/// Build a complete [`SceneDto`] from a solved model.
///
/// This is the single point where solver data is converted to the
/// frontend-facing DTO.  All indices, axes, and forces are resolved here.
pub fn build_scene(model: &FrameModel3D) -> Result<SceneDto, String> {
    let result: FrameAnalysisResult3D = model.solve().map_err(|e| format!("{e:?}"))?;
    build_scene_from_result(&result)
}

/// Build a [`SceneDto`] directly from an analysis result.
///
/// Separated from [`build_scene`] so tests can construct results independently.
pub fn build_scene_from_result(result: &FrameAnalysisResult3D) -> Result<SceneDto, String> {
    let mut nodes = Vec::with_capacity(result.n_nodes);
    for i in 0..result.n_nodes {
        let (x, y, z) = result.node_coords[i];
        let base = i * 6;
        nodes.push(NodeDto {
            id: i,
            position: Vec3Dto { x, y, z },
            displacement: Vec3Dto {
                x: result.displacements[base],
                y: result.displacements[base + 1],
                z: result.displacements[base + 2],
            },
            rotation: Vec3Dto {
                x: result.displacements[base + 3],
                y: result.displacements[base + 4],
                z: result.displacements[base + 5],
            },
            reaction_force: Vec3Dto {
                x: result.reactions[base],
                y: result.reactions[base + 1],
                z: result.reactions[base + 2],
            },
            reaction_moment: Vec3Dto {
                x: result.reactions[base + 3],
                y: result.reactions[base + 4],
                z: result.reactions[base + 5],
            },
        });
    }

    let mut members = Vec::with_capacity(result.n_members);
    for j in 0..result.n_members {
        let (ni, nj) = result.member_nodes[j];
        let rv = result.member_ref_vecs[j];
        let (x_hat, y_hat, z_hat) = result.member_local_axes(j).map_err(|e| format!("{e:?}"))?;
        let f_global = result
            .member_end_forces_global(j)
            .map_err(|e| format!("{e:?}"))?;
        members.push(MemberDto {
            id: j,
            node_i: ni,
            node_j: nj,
            ref_vec: Vec3Dto {
                x: rv[0],
                y: rv[1],
                z: rv[2],
            },
            local_x_axis: Vec3Dto {
                x: x_hat[0],
                y: x_hat[1],
                z: x_hat[2],
            },
            local_y_axis: Vec3Dto {
                x: y_hat[0],
                y: y_hat[1],
                z: y_hat[2],
            },
            local_z_axis: Vec3Dto {
                x: z_hat[0],
                y: z_hat[1],
                z: z_hat[2],
            },
            end_forces_local: result.member_end_forces[j],
            end_forces_global: f_global,
        });
    }

    Ok(SceneDto {
        nodes,
        members,
        n_nodes: result.n_nodes,
        n_members: result.n_members,
    })
}

// ---------------------------------------------------------------------------
// Tauri commands
// ---------------------------------------------------------------------------

#[tauri::command]
fn get_scene() -> Result<SceneDto, String> {
    let model = build_portal_frame();
    build_scene(&model)
}

#[tauri::command]
fn get_scene_inclined() -> Result<SceneDto, String> {
    let model = build_inclined_frame();
    build_scene(&model)
}

#[tauri::command]
fn get_scene_grid(nx: usize, nz: usize) -> Result<SceneDto, String> {
    let model = build_grid_frame(nx, nz);
    build_scene(&model)
}

// ---------------------------------------------------------------------------
// App entry
// ---------------------------------------------------------------------------

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .invoke_handler(tauri::generate_handler![
            get_scene,
            get_scene_inclined,
            get_scene_grid
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
