//! Versioned project persistence — save/load `.saproject` files.
//!
//! The file format is JSON with an explicit `format_version` field.
//! Only version 1 is currently supported.
//!
//! # Transaction safety
//!
//! - **Save**: serialize to a temporary file, then rename atomically.
//! - **Load**: parse and validate fully before replacing the current model.
//!
//! # Stable IDs
//!
//! Node and member IDs are preserved exactly across save/load cycles.
//! The `next_node_id` / `next_member_id` counters are also preserved
//! so that new entities created after loading do not collide with
//! existing IDs.

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::edit_model::{
    EditMember, EditModel, EditNode, MemberId, MemberPointLoad3D, MemberUniformLoad3D,
    NodalLoadSpec, NodeId, SupportSpec,
};
use crate::section_def::SectionDef;
use structural_analysis::FrameSection3D;

// ── File format version ───────────────────────────────────────────────

/// Currently supported file format version.
pub const CURRENT_FORMAT_VERSION: u32 = 3;

// ── DTO types ────────────────────────────────────────────────────────

/// Top-level project file.
#[derive(Serialize, Deserialize)]
pub struct ProjectFileV1 {
    pub format_version: u32,
    pub model: ModelFileV1,
}

/// Model data within a project file.
#[derive(Serialize, Deserialize)]
pub struct ModelFileV1 {
    pub next_node_id: u32,
    pub next_member_id: u32,
    pub nodes: Vec<NodeFileV1>,
    pub members: Vec<MemberFileV1>,
    pub supports: Vec<SupportFileV1>,
    pub nodal_loads: Vec<NodalLoadFileV1>,
}

/// Node DTO.
#[derive(Serialize, Deserialize, Clone)]
pub struct NodeFileV1 {
    pub id: u32,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Member DTO.
#[derive(Serialize, Deserialize, Clone)]
pub struct MemberFileV1 {
    pub id: u32,
    pub node_i: u32,
    pub node_j: u32,
    pub ref_vec: [f64; 3],
    pub e: f64,
    pub g: f64,
    pub area: f64,
    pub iy: f64,
    pub iz: f64,
    pub j: f64,
}

/// Support DTO.
#[derive(Serialize, Deserialize, Clone)]
pub struct SupportFileV1 {
    pub node_id: u32,
    /// 6 DOF values: `None` = free, `Some(v)` = constrained to v.
    /// Order: Ux, Uy, Uz, Rx, Ry, Rz.
    pub dofs: [Option<f64>; 6],
}

/// Nodal load DTO.
#[derive(Serialize, Deserialize, Clone)]
pub struct NodalLoadFileV1 {
    pub node_id: u32,
    pub fx: f64,
    pub fy: f64,
    pub fz: f64,
    pub mx: f64,
    pub my: f64,
    pub mz: f64,
}

// ── V2 DTO types ─────────────────────────────────────────────────────

/// Top-level project file (version 2).
#[derive(Serialize, Deserialize)]
pub struct ProjectFileV2 {
    pub format_version: u32,
    pub model: ModelFileV2,
}

/// Model data within a project file (version 2).
#[derive(Serialize, Deserialize)]
pub struct ModelFileV2 {
    pub next_node_id: u32,
    pub next_member_id: u32,
    pub nodes: Vec<NodeFileV1>,
    pub members: Vec<MemberFileV1>,
    pub supports: Vec<SupportFileV1>,
    pub nodal_loads: Vec<NodalLoadFileV1>,
    pub member_uniform_loads: Vec<MemberUniformLoadFileV2>,
}

/// Member uniform load DTO (version 2).
#[derive(Serialize, Deserialize)]
pub struct MemberUniformLoadFileV2 {
    pub member_id: u32,
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
}

// ── V3 DTO types ─────────────────────────────────────────────────────

/// Model data within a project file (version 3).
#[derive(Serialize, Deserialize)]
pub struct ModelFileV3 {
    pub next_node_id: u32,
    pub next_member_id: u32,
    pub nodes: Vec<NodeFileV1>,
    pub members: Vec<MemberFileV1>,
    pub supports: Vec<SupportFileV1>,
    pub nodal_loads: Vec<NodalLoadFileV1>,
    pub member_uniform_loads: Vec<MemberUniformLoadFileV2>,
    pub member_point_loads: Vec<MemberPointLoadFileV3>,
}

/// Member point load DTO (version 3).
#[derive(Serialize, Deserialize)]
pub struct MemberPointLoadFileV3 {
    pub member_id: u32,
    pub xi: f64,
    pub fx: f64,
    pub fy: f64,
    pub fz: f64,
}

// ── Section-module state (additive, optional) ────────────────────────

/// Top-level project file (version 3).
///
/// The `section` field is an **additive, optional** extension: it is skipped
/// when `None` (so files written before this feature existed are byte-for-byte
/// unchanged) and defaults to `None` when absent (so old files still load).
/// Readers that do not know the field ignore it, since this format does not
/// use `deny_unknown_fields`.
#[derive(Serialize, Deserialize)]
pub struct ProjectFileV3 {
    pub format_version: u32,
    pub model: ModelFileV3,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub section: Option<SectionDef>,
}

// ── Errors ───────────────────────────────────────────────────────────

/// Persistence error.
#[derive(Debug, Clone, PartialEq)]
pub enum PersistenceError {
    /// File format version is not supported.
    UnsupportedVersion { found: u32, supported: u32 },
    /// JSON parsing failed.
    JsonError(String),
    /// File I/O failed.
    IoError(String),
    /// Engineering data validation failed.
    Validation(String),
    /// ID counter would overflow.
    IdExhaustion(String),
}

impl std::fmt::Display for PersistenceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnsupportedVersion { found, supported } => {
                write!(
                    f,
                    "unsupported format version {found} (only version {supported} is supported)"
                )
            }
            Self::JsonError(msg) => write!(f, "JSON error: {msg}"),
            Self::IoError(msg) => write!(f, "I/O error: {msg}"),
            Self::Validation(msg) => write!(f, "validation error: {msg}"),
            Self::IdExhaustion(msg) => write!(f, "ID exhaustion: {msg}"),
        }
    }
}

impl std::error::Error for PersistenceError {}

// ── DTO conversion: EditModel → DTO ──────────────────────────────────

fn model_to_dto(model: &EditModel) -> ModelFileV1 {
    let nodes: Vec<NodeFileV1> = model
        .nodes()
        .iter()
        .map(|n| NodeFileV1 {
            id: n.id.0,
            x: n.x,
            y: n.y,
            z: n.z,
        })
        .collect();

    let members: Vec<MemberFileV1> = model
        .members()
        .iter()
        .map(|m| MemberFileV1 {
            id: m.id.0,
            node_i: m.node_i.0,
            node_j: m.node_j.0,
            ref_vec: m.ref_vec,
            e: m.section.E(),
            g: m.section.G(),
            area: m.section.area(),
            iy: m.section.iy(),
            iz: m.section.iz(),
            j: m.section.j(),
        })
        .collect();

    let supports: Vec<SupportFileV1> = model
        .supports()
        .iter()
        .map(|(nid, spec)| SupportFileV1 {
            node_id: nid.0,
            dofs: spec.dofs(),
        })
        .collect();

    let nodal_loads: Vec<NodalLoadFileV1> = model
        .nodal_loads()
        .iter()
        .map(|(nid, load)| NodalLoadFileV1 {
            node_id: nid.0,
            fx: load.fx,
            fy: load.fy,
            fz: load.fz,
            mx: load.mx,
            my: load.my,
            mz: load.mz,
        })
        .collect();

    ModelFileV1 {
        next_node_id: model.next_node_id(),
        next_member_id: model.next_member_id(),
        nodes,
        members,
        supports,
        nodal_loads,
    }
}

fn model_to_dto_v3(model: &EditModel) -> ModelFileV3 {
    let v1 = model_to_dto(model);
    let member_uniform_loads: Vec<MemberUniformLoadFileV2> = model
        .member_uniform_loads()
        .iter()
        .map(|(mid, load)| MemberUniformLoadFileV2 {
            member_id: mid.0,
            qx: load.qx,
            qy: load.qy,
            qz: load.qz,
        })
        .collect();
    let member_point_loads: Vec<MemberPointLoadFileV3> = model
        .member_point_loads()
        .iter()
        .map(|(mid, load)| MemberPointLoadFileV3 {
            member_id: mid.0,
            xi: load.xi,
            fx: load.fx,
            fy: load.fy,
            fz: load.fz,
        })
        .collect();
    ModelFileV3 {
        next_node_id: v1.next_node_id,
        next_member_id: v1.next_member_id,
        nodes: v1.nodes,
        members: v1.members,
        supports: v1.supports,
        nodal_loads: v1.nodal_loads,
        member_uniform_loads,
        member_point_loads,
    }
}

// ── DTO conversion: DTO → EditModel (with validation) ────────────────

fn dto_to_model(dto: &ModelFileV1) -> Result<EditModel, PersistenceError> {
    validate_dto(dto)?;

    let nodes: Vec<EditNode> = dto
        .nodes
        .iter()
        .map(|n| EditNode {
            id: NodeId(n.id),
            x: n.x,
            y: n.y,
            z: n.z,
        })
        .collect();

    let members: Vec<EditMember> = dto
        .members
        .iter()
        .map(|m| EditMember {
            id: MemberId(m.id),
            node_i: NodeId(m.node_i),
            node_j: NodeId(m.node_j),
            ref_vec: m.ref_vec,
            section: FrameSection3D::new(m.e, m.g, m.area, m.iy, m.iz, m.j)
                .expect("validated above"),
        })
        .collect();

    let supports: Vec<(NodeId, SupportSpec)> = dto
        .supports
        .iter()
        .map(|s| (NodeId(s.node_id), SupportSpec::custom(s.dofs)))
        .collect();

    let nodal_loads: Vec<(NodeId, NodalLoadSpec)> = dto
        .nodal_loads
        .iter()
        .map(|l| {
            (
                NodeId(l.node_id),
                NodalLoadSpec {
                    fx: l.fx,
                    fy: l.fy,
                    fz: l.fz,
                    mx: l.mx,
                    my: l.my,
                    mz: l.mz,
                },
            )
        })
        .collect();

    Ok(EditModel::from_parts(
        nodes,
        members,
        supports,
        nodal_loads,
        Vec::new(),
        Vec::new(),
        dto.next_node_id,
        dto.next_member_id,
    ))
}

// ── DTO conversion: V2 DTO → EditModel (with validation) ─────────────

fn dto_v2_to_model(dto: &ModelFileV2) -> Result<EditModel, PersistenceError> {
    let v1 = ModelFileV1 {
        next_node_id: dto.next_node_id,
        next_member_id: dto.next_member_id,
        nodes: dto.nodes.clone(),
        members: dto.members.clone(),
        supports: dto.supports.clone(),
        nodal_loads: dto.nodal_loads.clone(),
    };
    validate_dto(&v1)?;

    use PersistenceError::Validation as V;

    let mut seen_member_loads = std::collections::HashSet::new();
    let member_exists = |id: u32| dto.members.iter().any(|m| m.id == id);
    for ml in &dto.member_uniform_loads {
        if !member_exists(ml.member_id) {
            return Err(V(format!(
                "member uniform load references nonexistent member {}",
                ml.member_id
            )));
        }
        if !seen_member_loads.insert(ml.member_id) {
            return Err(V(format!(
                "duplicate member uniform load for member {}",
                ml.member_id
            )));
        }
        if !ml.qx.is_finite() || !ml.qy.is_finite() || !ml.qz.is_finite() {
            return Err(V(format!(
                "member uniform load for member {} has non-finite values",
                ml.member_id
            )));
        }
    }

    let nodes: Vec<EditNode> = dto
        .nodes
        .iter()
        .map(|n| EditNode {
            id: NodeId(n.id),
            x: n.x,
            y: n.y,
            z: n.z,
        })
        .collect();

    let members: Vec<EditMember> = dto
        .members
        .iter()
        .map(|m| EditMember {
            id: MemberId(m.id),
            node_i: NodeId(m.node_i),
            node_j: NodeId(m.node_j),
            ref_vec: m.ref_vec,
            section: FrameSection3D::new(m.e, m.g, m.area, m.iy, m.iz, m.j)
                .expect("validated above"),
        })
        .collect();

    let supports: Vec<(NodeId, SupportSpec)> = dto
        .supports
        .iter()
        .map(|s| (NodeId(s.node_id), SupportSpec::custom(s.dofs)))
        .collect();

    let nodal_loads: Vec<(NodeId, NodalLoadSpec)> = dto
        .nodal_loads
        .iter()
        .map(|l| {
            (
                NodeId(l.node_id),
                NodalLoadSpec {
                    fx: l.fx,
                    fy: l.fy,
                    fz: l.fz,
                    mx: l.mx,
                    my: l.my,
                    mz: l.mz,
                },
            )
        })
        .collect();

    let member_uniform_loads: Vec<(MemberId, MemberUniformLoad3D)> = dto
        .member_uniform_loads
        .iter()
        .map(|ml| {
            (
                MemberId(ml.member_id),
                MemberUniformLoad3D {
                    qx: ml.qx,
                    qy: ml.qy,
                    qz: ml.qz,
                },
            )
        })
        .collect();

    Ok(EditModel::from_parts(
        nodes,
        members,
        supports,
        nodal_loads,
        member_uniform_loads,
        Vec::new(),
        dto.next_node_id,
        dto.next_member_id,
    ))
}

// ── DTO conversion: V3 DTO → EditModel (with validation) ─────────────

fn dto_v3_to_model(dto: &ModelFileV3) -> Result<EditModel, PersistenceError> {
    let v1 = ModelFileV1 {
        next_node_id: dto.next_node_id,
        next_member_id: dto.next_member_id,
        nodes: dto.nodes.clone(),
        members: dto.members.clone(),
        supports: dto.supports.clone(),
        nodal_loads: dto.nodal_loads.clone(),
    };
    validate_dto(&v1)?;

    use PersistenceError::Validation as V;

    let member_exists = |id: u32| dto.members.iter().any(|m| m.id == id);

    let mut seen_member_loads = std::collections::HashSet::new();
    for ml in &dto.member_uniform_loads {
        if !member_exists(ml.member_id) {
            return Err(V(format!(
                "member uniform load references nonexistent member {}",
                ml.member_id
            )));
        }
        if !seen_member_loads.insert(ml.member_id) {
            return Err(V(format!(
                "duplicate member uniform load for member {}",
                ml.member_id
            )));
        }
        if !ml.qx.is_finite() || !ml.qy.is_finite() || !ml.qz.is_finite() {
            return Err(V(format!(
                "member uniform load for member {} has non-finite values",
                ml.member_id
            )));
        }
    }

    for pl in &dto.member_point_loads {
        if !member_exists(pl.member_id) {
            return Err(V(format!(
                "member point load references nonexistent member {}",
                pl.member_id
            )));
        }
        if !pl.xi.is_finite() || !pl.fx.is_finite() || !pl.fy.is_finite() || !pl.fz.is_finite() {
            return Err(V(format!(
                "member point load for member {} has non-finite values",
                pl.member_id
            )));
        }
        if !(0.0..=1.0).contains(&pl.xi) {
            return Err(V(format!(
                "member point load for member {} has xi = {} outside [0, 1]",
                pl.member_id, pl.xi
            )));
        }
    }

    let nodes: Vec<EditNode> = dto
        .nodes
        .iter()
        .map(|n| EditNode {
            id: NodeId(n.id),
            x: n.x,
            y: n.y,
            z: n.z,
        })
        .collect();

    let members: Vec<EditMember> = dto
        .members
        .iter()
        .map(|m| EditMember {
            id: MemberId(m.id),
            node_i: NodeId(m.node_i),
            node_j: NodeId(m.node_j),
            ref_vec: m.ref_vec,
            section: FrameSection3D::new(m.e, m.g, m.area, m.iy, m.iz, m.j)
                .expect("validated above"),
        })
        .collect();

    let supports: Vec<(NodeId, SupportSpec)> = dto
        .supports
        .iter()
        .map(|s| (NodeId(s.node_id), SupportSpec::custom(s.dofs)))
        .collect();

    let nodal_loads: Vec<(NodeId, NodalLoadSpec)> = dto
        .nodal_loads
        .iter()
        .map(|l| {
            (
                NodeId(l.node_id),
                NodalLoadSpec {
                    fx: l.fx,
                    fy: l.fy,
                    fz: l.fz,
                    mx: l.mx,
                    my: l.my,
                    mz: l.mz,
                },
            )
        })
        .collect();

    let member_uniform_loads: Vec<(MemberId, MemberUniformLoad3D)> = dto
        .member_uniform_loads
        .iter()
        .map(|ml| {
            (
                MemberId(ml.member_id),
                MemberUniformLoad3D {
                    qx: ml.qx,
                    qy: ml.qy,
                    qz: ml.qz,
                },
            )
        })
        .collect();

    let member_point_loads: Vec<(MemberId, MemberPointLoad3D)> = dto
        .member_point_loads
        .iter()
        .map(|pl| {
            (
                MemberId(pl.member_id),
                MemberPointLoad3D {
                    xi: pl.xi,
                    fx: pl.fx,
                    fy: pl.fy,
                    fz: pl.fz,
                },
            )
        })
        .collect();

    Ok(EditModel::from_parts(
        nodes,
        members,
        supports,
        nodal_loads,
        member_uniform_loads,
        member_point_loads,
        dto.next_node_id,
        dto.next_member_id,
    ))
}

// ── Validation ───────────────────────────────────────────────────────

/// Validate a DTO before converting to EditModel.
fn validate_dto(dto: &ModelFileV1) -> Result<(), PersistenceError> {
    use PersistenceError::Validation as V;

    // ── Unique node IDs ──
    let mut seen_nodes = std::collections::HashSet::new();
    for n in &dto.nodes {
        if !seen_nodes.insert(n.id) {
            return Err(V(format!("duplicate node ID {}", n.id)));
        }
    }

    // ── Unique member IDs ──
    let mut seen_members = std::collections::HashSet::new();
    for m in &dto.members {
        if !seen_members.insert(m.id) {
            return Err(V(format!("duplicate member ID {}", m.id)));
        }
    }

    // ── Finite node coordinates ──
    for n in &dto.nodes {
        if !n.x.is_finite() || !n.y.is_finite() || !n.z.is_finite() {
            return Err(V(format!("node {} has non-finite coordinates", n.id)));
        }
    }

    // ── Valid member endpoints, no self-connection, no zero-length ──
    let node_exists = |id: u32| seen_nodes.contains(&id);
    for m in &dto.members {
        if m.node_i == m.node_j {
            return Err(V(format!(
                "member {} is self-connected (node_i == node_j == {})",
                m.id, m.node_i
            )));
        }
        if !node_exists(m.node_i) {
            return Err(V(format!(
                "member {} references nonexistent node_i {}",
                m.id, m.node_i
            )));
        }
        if !node_exists(m.node_j) {
            return Err(V(format!(
                "member {} references nonexistent node_j {}",
                m.id, m.node_j
            )));
        }
        let ni = dto.nodes.iter().find(|n| n.id == m.node_i).unwrap();
        let nj = dto.nodes.iter().find(|n| n.id == m.node_j).unwrap();
        let dx = nj.x - ni.x;
        let dy = nj.y - ni.y;
        let dz = nj.z - ni.z;
        let len = (dx * dx + dy * dy + dz * dz).sqrt();
        if len < 1e-12 {
            return Err(V(format!("member {} has zero length", m.id)));
        }
    }

    // ── Valid section parameters ──
    for m in &dto.members {
        for (label, val) in [
            ("E", m.e),
            ("G", m.g),
            ("area", m.area),
            ("Iy", m.iy),
            ("Iz", m.iz),
            ("J", m.j),
        ] {
            if !val.is_finite() {
                return Err(V(format!(
                    "member {} has non-finite section {}",
                    m.id, label
                )));
            }
            if val <= 0.0 {
                return Err(V(format!(
                    "member {} has non-positive section {} = {}",
                    m.id, label, val
                )));
            }
        }
    }

    // ── Valid reference vectors ──
    for m in &dto.members {
        if !m.ref_vec.iter().all(|v| v.is_finite()) {
            return Err(V(format!(
                "member {} has non-finite reference vector",
                m.id
            )));
        }
        let norm = (m.ref_vec[0] * m.ref_vec[0]
            + m.ref_vec[1] * m.ref_vec[1]
            + m.ref_vec[2] * m.ref_vec[2])
            .sqrt();
        if norm < 1e-12 {
            return Err(V(format!("member {} has zero reference vector", m.id)));
        }
        // Check not parallel to member axis.
        let ni = dto.nodes.iter().find(|n| n.id == m.node_i).unwrap();
        let nj = dto.nodes.iter().find(|n| n.id == m.node_j).unwrap();
        let axis = [nj.x - ni.x, nj.y - ni.y, nj.z - ni.z];
        let axis_norm = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        let axis_unit = [
            axis[0] / axis_norm,
            axis[1] / axis_norm,
            axis[2] / axis_norm,
        ];
        let ref_unit = [
            m.ref_vec[0] / norm,
            m.ref_vec[1] / norm,
            m.ref_vec[2] / norm,
        ];
        let cross = [
            axis_unit[1] * ref_unit[2] - axis_unit[2] * ref_unit[1],
            axis_unit[2] * ref_unit[0] - axis_unit[0] * ref_unit[2],
            axis_unit[0] * ref_unit[1] - axis_unit[1] * ref_unit[0],
        ];
        let sin_angle = (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt();
        if sin_angle < 1e-8 {
            return Err(V(format!(
                "member {} has reference vector parallel to member axis",
                m.id
            )));
        }
    }

    // ── Valid support references, no duplicates ──
    let mut seen_supports = std::collections::HashSet::new();
    for s in &dto.supports {
        if !node_exists(s.node_id) {
            return Err(V(format!(
                "support references nonexistent node {}",
                s.node_id
            )));
        }
        if !seen_supports.insert(s.node_id) {
            return Err(V(format!("duplicate support for node {}", s.node_id)));
        }
        for (i, dof) in s.dofs.iter().enumerate() {
            if let Some(v) = dof
                && !v.is_finite()
            {
                return Err(V(format!(
                    "support for node {} has non-finite DOF {}",
                    s.node_id, i
                )));
            }
        }
    }

    // ── Valid nodal loads, no duplicates ──
    let mut seen_loads = std::collections::HashSet::new();
    for l in &dto.nodal_loads {
        if !node_exists(l.node_id) {
            return Err(V(format!(
                "nodal load references nonexistent node {}",
                l.node_id
            )));
        }
        if !seen_loads.insert(l.node_id) {
            return Err(V(format!("duplicate nodal load for node {}", l.node_id)));
        }
        let load = NodalLoadSpec {
            fx: l.fx,
            fy: l.fy,
            fz: l.fz,
            mx: l.mx,
            my: l.my,
            mz: l.mz,
        };
        if !load.is_finite() {
            return Err(V(format!(
                "nodal load for node {} has non-finite values",
                l.node_id
            )));
        }
    }

    // ── ID counters must exceed all existing IDs ──
    for n in &dto.nodes {
        if dto.next_node_id <= n.id {
            return Err(V(format!(
                "next_node_id {} is not greater than existing node ID {}",
                dto.next_node_id, n.id
            )));
        }
    }
    for m in &dto.members {
        if dto.next_member_id <= m.id {
            return Err(V(format!(
                "next_member_id {} is not greater than existing member ID {}",
                dto.next_member_id, m.id
            )));
        }
    }

    Ok(())
}

// ── Serialization API ───────────────────────────────────────────────

/// A project loaded from disk: the structural model plus the optional
/// Section-module state stored alongside it.
///
/// `section` is `None` for project files written before the section state was
/// persisted (or when the writing app had no section to store).
pub struct LoadedProject {
    pub model: EditModel,
    pub section: Option<SectionDef>,
}

/// Serialize an EditModel to a JSON string (no section state).
pub fn serialize_project(model: &EditModel) -> Result<String, PersistenceError> {
    serialize_project_with_section(model, None)
}

/// Serialize an EditModel plus optional Section-module state to JSON.
///
/// The section is an **additive, optional** top-level field, so a call with
/// `None` produces exactly the same bytes as [`serialize_project`].
pub fn serialize_project_with_section(
    model: &EditModel,
    section: Option<&SectionDef>,
) -> Result<String, PersistenceError> {
    let file = ProjectFileV3 {
        format_version: CURRENT_FORMAT_VERSION,
        model: model_to_dto_v3(model),
        section: section.cloned(),
    };
    serde_json::to_string_pretty(&file).map_err(|e| PersistenceError::JsonError(e.to_string()))
}

/// Deserialize a JSON string to an EditModel (dropping any section state).
///
/// Supports version 1 (legacy, no member loads), version 2 (member
/// uniform loads only), and version 3 (current, with member uniform
/// and point loads plus optional Section-module state).
pub fn deserialize_project(json: &str) -> Result<EditModel, PersistenceError> {
    Ok(deserialize_loaded_project(json)?.model)
}

/// Deserialize a JSON string into a [`LoadedProject`] (model + section state).
///
/// Version 1 and 2 files carry no section state; version 3 files may or may
/// not (the field is optional), so both old-style and new-style payloads load.
pub fn deserialize_loaded_project(json: &str) -> Result<LoadedProject, PersistenceError> {
    #[derive(Deserialize)]
    struct VersionProbe {
        format_version: u32,
    }

    let probe: VersionProbe =
        serde_json::from_str(json).map_err(|e| PersistenceError::JsonError(e.to_string()))?;

    match probe.format_version {
        1 => {
            let file: ProjectFileV1 = serde_json::from_str(json)
                .map_err(|e| PersistenceError::JsonError(e.to_string()))?;
            Ok(LoadedProject {
                model: dto_to_model(&file.model)?,
                section: None,
            })
        }
        2 => {
            let file: ProjectFileV2 = serde_json::from_str(json)
                .map_err(|e| PersistenceError::JsonError(e.to_string()))?;
            Ok(LoadedProject {
                model: dto_v2_to_model(&file.model)?,
                section: None,
            })
        }
        3 => {
            let file: ProjectFileV3 = serde_json::from_str(json)
                .map_err(|e| PersistenceError::JsonError(e.to_string()))?;
            Ok(LoadedProject {
                model: dto_v3_to_model(&file.model)?,
                section: file.section,
            })
        }
        found => Err(PersistenceError::UnsupportedVersion {
            found,
            supported: CURRENT_FORMAT_VERSION,
        }),
    }
}

/// Write `contents` to `path` via a temporary file, then rename atomically.
///
/// The parent directory is created if necessary.  On Windows, rename-over-
/// existing is not guaranteed atomic, but is the best available strategy
/// without requiring filesystem locks.
fn write_atomic(path: &Path, contents: &str) -> Result<(), PersistenceError> {
    if let Some(parent) = path.parent()
        && !parent.as_os_str().is_empty()
    {
        std::fs::create_dir_all(parent).map_err(|e| PersistenceError::IoError(e.to_string()))?;
    }

    let mut tmp_path = path.to_path_buf();
    let file_name = path
        .file_name()
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| "tmp".to_string());
    tmp_path.set_file_name(format!(".{file_name}.tmp"));

    std::fs::write(&tmp_path, contents.as_bytes())
        .map_err(|e| PersistenceError::IoError(e.to_string()))?;

    std::fs::rename(&tmp_path, path).map_err(|e| {
        let _ = std::fs::remove_file(&tmp_path);
        PersistenceError::IoError(e.to_string())
    })?;

    Ok(())
}

/// Save an EditModel to a `.saproject` file (no section state).
pub fn save_project(path: &Path, model: &EditModel) -> Result<(), PersistenceError> {
    save_project_with_section(path, model, None)
}

/// Save an EditModel plus optional Section-module state to a `.saproject` file.
///
/// Writes to a temporary file first, then renames atomically.
pub fn save_project_with_section(
    path: &Path,
    model: &EditModel,
    section: Option<&SectionDef>,
) -> Result<(), PersistenceError> {
    let json = serialize_project_with_section(model, section)?;
    write_atomic(path, &json)
}

/// Load an EditModel from a `.saproject` file (dropping any section state).
pub fn load_project(path: &Path) -> Result<EditModel, PersistenceError> {
    Ok(load_project_with_section(path)?.model)
}

/// Load a [`LoadedProject`] (model + section state) from a `.saproject` file.
///
/// Reads and validates the file fully before returning.  Does not modify any
/// existing model — the caller is responsible for transactional replacement.
pub fn load_project_with_section(path: &Path) -> Result<LoadedProject, PersistenceError> {
    let contents =
        std::fs::read_to_string(path).map_err(|e| PersistenceError::IoError(e.to_string()))?;
    deserialize_loaded_project(&contents)
}

// ── App settings ─────────────────────────────────────────────────────

/// Window geometry persisted between sessions (in egui points).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct WindowPrefs {
    pub width: f32,
    pub height: f32,
}

/// Application settings persisted outside of any project.
///
/// Kept deliberately small: the chrome theme plus the cheaply captured window
/// size.  The **layout preset** (panel widths / dock height / collapsed
/// flags) is stored alongside these fields in the same file under the
/// `"layout"` key — see [`LayoutPrefs`], [`save_layout`] and
/// [`load_layout_or_default`].  It is kept as a separate top-level key (not a
/// field of this struct) so that both writers cooperate without either
/// clobbering the other's keys.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AppSettings {
    /// egui chrome theme (`true` = dark).
    #[serde(default = "default_dark_mode")]
    pub dark_mode: bool,
    /// Last window size, if known.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub window: Option<WindowPrefs>,
}

/// Dark chrome is the historical default.
fn default_dark_mode() -> bool {
    true
}

impl Default for AppSettings {
    fn default() -> Self {
        Self {
            dark_mode: default_dark_mode(),
            window: None,
        }
    }
}

/// Persisted panel layout preset (additive extension of the settings file).
///
/// Stored under the `"layout"` key of the settings file, next to
/// [`AppSettings`].  Every field is `#[serde(default)]`, so an older settings
/// file without the key (or without individual fields) loads cleanly with
/// defaults.  Widths / heights are `Option`: `None` means "never captured",
/// and the panels fall back to their window-fraction defaults.
///
/// The **theme** is deliberately *not* duplicated here — it already lives in
/// [`AppSettings::dark_mode`], which stays the single source of truth (the
/// theme is written by the settings save path on every toggle).
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct LayoutPrefs {
    /// Width of the left model-tree panel (points).
    #[serde(default)]
    pub tree_width: Option<f32>,
    /// Width of the right inspector panel (points).
    #[serde(default)]
    pub inspector_width: Option<f32>,
    /// Height of the bottom Messages / Results dock (points).
    #[serde(default)]
    pub dock_height: Option<f32>,
    /// Whether the inspector panel is collapsed.
    #[serde(default)]
    pub inspector_collapsed: bool,
    /// Whether the bottom dock is collapsed.
    #[serde(default)]
    pub dock_collapsed: bool,
}

/// The per-user settings file path.
///
/// Resolved without any extra dependency:
///
/// * Windows: `%APPDATA%\section-properties\settings.json`
/// * otherwise: `$XDG_CONFIG_HOME/section-properties/settings.json`, or
///   `$HOME/.config/section-properties/settings.json`
/// * fallback (neither env var set): `section-properties-settings.json` in the
///   current working directory.
pub fn settings_path() -> PathBuf {
    let base = if cfg!(windows) {
        std::env::var_os("APPDATA").map(PathBuf::from)
    } else {
        std::env::var_os("XDG_CONFIG_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))
    };
    match base {
        Some(dir) => dir.join("section-properties").join("settings.json"),
        None => PathBuf::from("section-properties-settings.json"),
    }
}

/// Load settings from `path`, falling back to [`AppSettings::default`].
///
/// A *missing* file is not an error (first run) and produces no warning.  A
/// file that exists but cannot be read or parsed also yields defaults, but the
/// problem is returned as the second tuple element so the caller can surface
/// it (e.g. in the message dock) rather than silently swallowing it.
pub fn load_settings_or_default(path: &Path) -> (AppSettings, Option<String>) {
    match std::fs::read_to_string(path) {
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => (AppSettings::default(), None),
        Err(e) => (
            AppSettings::default(),
            Some(format!("could not read {} ({e})", path.display())),
        ),
        Ok(text) => match serde_json::from_str::<AppSettings>(&text) {
            Ok(settings) => (settings, None),
            Err(e) => (
                AppSettings::default(),
                Some(format!("settings file {} is corrupt ({e})", path.display())),
            ),
        },
    }
}

/// Save settings to `path` (creating the parent directory if needed).
///
/// The additive `"layout"` key written by [`save_layout`] is preserved: this
/// function serializes the [`AppSettings`] fields and re-attaches any
/// existing `"layout"` value, so the two writers never clobber each other.
pub fn save_settings(path: &Path, settings: &AppSettings) -> Result<(), PersistenceError> {
    let mut json = serde_json::to_value(settings)
        .map_err(|e| PersistenceError::JsonError(e.to_string()))?;
    if let Some(layout) = read_layout_value(path)
        && let Some(obj) = json.as_object_mut()
    {
        obj.insert("layout".to_string(), layout);
    }
    let text = serde_json::to_string_pretty(&json)
        .map_err(|e| PersistenceError::JsonError(e.to_string()))?;
    write_atomic(path, &text)
}

/// Load the persisted layout preset from the settings file at `path`.
///
/// A **missing** `"layout"` key (every settings file written before layout
/// presets existed) is not an error and yields [`LayoutPrefs::default`].  A
/// key that is present but malformed also yields defaults, with the problem
/// returned as the second tuple element so the caller can surface it rather
/// than silently swallow it.
pub fn load_layout_or_default(path: &Path) -> (LayoutPrefs, Option<String>) {
    match read_layout_value(path) {
        None => (LayoutPrefs::default(), None),
        Some(value) => match serde_json::from_value::<LayoutPrefs>(value) {
            Ok(layout) => (layout, None),
            Err(e) => (
                LayoutPrefs::default(),
                Some(format!(
                    "settings file {} has an invalid layout ({e})",
                    path.display()
                )),
            ),
        },
    }
}

/// Merge `layout` into the settings file at `path` (read-modify-write,
/// atomic).
///
/// The existing `dark_mode` / `window` keys are preserved; a file that is
/// missing or does not parse as a JSON object starts fresh with just the
/// layout key.
pub fn save_layout(path: &Path, layout: &LayoutPrefs) -> Result<(), PersistenceError> {
    let mut root = match std::fs::read_to_string(path) {
        Ok(text) => match serde_json::from_str::<serde_json::Value>(&text) {
            Ok(serde_json::Value::Object(map)) => serde_json::Value::Object(map),
            _ => serde_json::Value::Object(serde_json::Map::new()),
        },
        Err(_) => serde_json::Value::Object(serde_json::Map::new()),
    };
    let value =
        serde_json::to_value(layout).map_err(|e| PersistenceError::JsonError(e.to_string()))?;
    if let Some(obj) = root.as_object_mut() {
        obj.insert("layout".to_string(), value);
    }
    let text = serde_json::to_string_pretty(&root)
        .map_err(|e| PersistenceError::JsonError(e.to_string()))?;
    write_atomic(path, &text)
}

/// Read the raw `"layout"` JSON value out of the settings file, if the file
/// exists, parses as an object and carries the key.
fn read_layout_value(path: &Path) -> Option<serde_json::Value> {
    let text = std::fs::read_to_string(path).ok()?;
    let value: serde_json::Value = serde_json::from_str(&text).ok()?;
    value.as_object()?.get("layout").cloned()
}

// ── Tests ────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::edit_model::{EditModel, MemberId, NodalLoadSpec, SupportSpec};
    use crate::section_def::{MaterialPreset, SectionKind, SectionMaterial, SectionParams};

    fn sample_model() -> EditModel {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        m.add_member(a, b).unwrap();
        m.set_member_section(MemberId(0), 200e9, 80e9, 1e-3, 2e-6, 1e-6, 3e-6)
            .unwrap();
        m.set_support(a, SupportSpec::fixed()).unwrap();
        m.set_support(b, SupportSpec::pinned()).unwrap();
        m.set_nodal_load(
            b,
            NodalLoadSpec {
                fx: 0.0,
                fy: 0.0,
                fz: -1000.0,
                mx: 0.0,
                my: 0.0,
                mz: 0.0,
            },
        )
        .unwrap();
        m
    }

    #[test]
    fn round_trip_empty() {
        let m = EditModel::new();
        let json = serialize_project(&m).unwrap();
        let m2 = deserialize_project(&json).unwrap();
        assert_eq!(m2.nodes().len(), 0);
        assert_eq!(m2.members().len(), 0);
        assert_eq!(m2.next_node_id(), 0);
        assert_eq!(m2.next_member_id(), 0);
    }

    #[test]
    fn round_trip_sample() {
        let m = sample_model();
        let json = serialize_project(&m).unwrap();
        let m2 = deserialize_project(&json).unwrap();

        assert_eq!(m2.nodes().len(), 2);
        assert_eq!(m2.members().len(), 1);
        assert_eq!(m2.supports().len(), 2);
        assert_eq!(m2.nodal_loads().len(), 1);

        assert_eq!(m2.nodes()[0].id, m.nodes()[0].id);
        assert_eq!(m2.nodes()[1].id, m.nodes()[1].id);
        assert_eq!(m2.members()[0].id, m.members()[0].id);
        assert_eq!(m2.next_node_id(), m.next_node_id());
        assert_eq!(m2.next_member_id(), m.next_member_id());
    }

    #[test]
    fn stable_id_preservation() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(1.0, 0.0, 0.0).unwrap();
        let c = m.add_node(2.0, 0.0, 0.0).unwrap();
        m.delete_node(b).unwrap();
        let d = m.add_node(3.0, 0.0, 0.0).unwrap();

        let json = serialize_project(&m).unwrap();
        let m2 = deserialize_project(&json).unwrap();

        let ids: Vec<u32> = m2.nodes().iter().map(|n| n.id.0).collect();
        assert!(ids.contains(&a.0));
        assert!(!ids.contains(&b.0));
        assert!(ids.contains(&c.0));
        assert!(ids.contains(&d.0));
        assert_eq!(m2.next_node_id(), m.next_node_id());
    }

    #[test]
    fn deterministic_serialization() {
        let m = sample_model();
        let j1 = serialize_project(&m).unwrap();
        let j2 = serialize_project(&m).unwrap();
        assert_eq!(j1, j2);
    }

    #[test]
    fn unsupported_version_rejected() {
        let json = r#"{"format_version":99,"model":{"next_node_id":0,"next_member_id":0,"nodes":[],"members":[],"supports":[],"nodal_loads":[]}}"#;
        let err = deserialize_project(json).unwrap_err();
        assert!(matches!(err, PersistenceError::UnsupportedVersion { .. }));
    }

    #[test]
    fn malformed_json_rejected() {
        let json = r#"{"format_version":1,"model":invalid}"#;
        let err = deserialize_project(json).unwrap_err();
        assert!(matches!(err, PersistenceError::JsonError(_)));
    }

    #[test]
    fn duplicate_node_id_rejected() {
        let json = r#"{"format_version":1,"model":{"next_node_id":2,"next_member_id":0,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":0,"x":1.0,"y":0.0,"z":0.0}],"members":[],"supports":[],"nodal_loads":[]}}"#;
        let err = deserialize_project(json).unwrap_err();
        assert!(matches!(err, PersistenceError::Validation(_)));
    }

    #[test]
    fn missing_member_endpoint_rejected() {
        let json = r#"{"format_version":1,"model":{"next_node_id":1,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":5,"ref_vec":[0.0,0.0,1.0],"e":200e9,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[]}}"#;
        let err = deserialize_project(json).unwrap_err();
        assert!(matches!(err, PersistenceError::Validation(_)));
    }

    #[test]
    fn invalid_section_rejected() {
        let json = r#"{"format_version":1,"model":{"next_node_id":2,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":1.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":1,"ref_vec":[0.0,0.0,1.0],"e":-1.0,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[]}}"#;
        let err = deserialize_project(json).unwrap_err();
        assert!(matches!(err, PersistenceError::Validation(_)));
    }

    #[test]
    fn invalid_ref_vec_rejected() {
        let json = r#"{"format_version":1,"model":{"next_node_id":2,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":1.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":1,"ref_vec":[0.0,0.0,0.0],"e":200e9,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[]}}"#;
        let err = deserialize_project(json).unwrap_err();
        assert!(matches!(err, PersistenceError::Validation(_)));
    }

    #[test]
    fn invalid_support_reference_rejected() {
        let json = r#"{"format_version":1,"model":{"next_node_id":1,"next_member_id":0,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0}],"members":[],"supports":[{"node_id":5,"dofs":[null,null,null,null,null,null]}],"nodal_loads":[]}}"#;
        let err = deserialize_project(json).unwrap_err();
        assert!(matches!(err, PersistenceError::Validation(_)));
    }

    #[test]
    fn duplicate_support_rejected() {
        let json = r#"{"format_version":1,"model":{"next_node_id":1,"next_member_id":0,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0}],"members":[],"supports":[{"node_id":0,"dofs":[0.0,0.0,0.0,0.0,0.0,0.0]},{"node_id":0,"dofs":[0.0,0.0,0.0,null,null,null]}],"nodal_loads":[]}}"#;
        let err = deserialize_project(json).unwrap_err();
        assert!(matches!(err, PersistenceError::Validation(_)));
    }

    #[test]
    fn next_id_below_existing_rejected() {
        let json = r#"{"format_version":1,"model":{"next_node_id":1,"next_member_id":0,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":2,"x":1.0,"y":0.0,"z":0.0}],"members":[],"supports":[],"nodal_loads":[]}}"#;
        let err = deserialize_project(json).unwrap_err();
        assert!(matches!(err, PersistenceError::Validation(_)));
    }

    #[test]
    fn filesystem_round_trip() {
        let dir = std::env::temp_dir();
        let path = dir.join("test_saproject_phase150.saproject");

        let m = sample_model();
        save_project(&path, &m).unwrap();
        let m2 = load_project(&path).unwrap();

        assert_eq!(m2.nodes().len(), 2);
        assert_eq!(m2.members().len(), 1);
        assert_eq!(m2.nodes()[0].id, m.nodes()[0].id);
        assert_eq!(m2.members()[0].id, m.members()[0].id);

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn failed_load_preserves_existing_file() {
        let dir = std::env::temp_dir();
        let path_good = dir.join("test_good_phase150.saproject");
        let path_bad = dir.join("test_bad_phase150.saproject");

        let m = sample_model();
        save_project(&path_good, &m).unwrap();
        std::fs::write(&path_bad, b"not valid json").unwrap();

        assert!(load_project(&path_bad).is_err());
        let m2 = load_project(&path_good).unwrap();
        assert_eq!(m2.nodes().len(), 2);

        let _ = std::fs::remove_file(&path_good);
        let _ = std::fs::remove_file(&path_bad);
    }

    // ── App settings ─────────────────────────────────────────────────

    #[test]
    fn settings_round_trip() {
        let path = std::env::temp_dir().join("test_settings_round_trip.json");
        let _ = std::fs::remove_file(&path);
        let settings = AppSettings {
            dark_mode: false,
            window: Some(WindowPrefs {
                width: 1280.0,
                height: 720.0,
            }),
        };
        save_settings(&path, &settings).unwrap();
        let (loaded, warning) = load_settings_or_default(&path);
        assert!(warning.is_none());
        assert_eq!(loaded, settings);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn settings_default_when_missing() {
        let path = std::env::temp_dir().join("test_settings_missing_xyz.json");
        let _ = std::fs::remove_file(&path);
        let (loaded, warning) = load_settings_or_default(&path);
        assert!(warning.is_none(), "missing file must not warn");
        assert_eq!(loaded, AppSettings::default());
        assert!(loaded.dark_mode, "default theme is dark");
        assert!(loaded.window.is_none());
    }

    #[test]
    fn settings_corrupt_file_falls_back_with_warning() {
        let path = std::env::temp_dir().join("test_settings_corrupt.json");
        std::fs::write(&path, b"{ this is not json").unwrap();
        let (loaded, warning) = load_settings_or_default(&path);
        assert_eq!(loaded, AppSettings::default());
        assert!(warning.is_some(), "corrupt file must be reported, not masked");
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn settings_partial_json_uses_field_defaults() {
        // Only `window` present → `dark_mode` falls back to the dark default.
        let path = std::env::temp_dir().join("test_settings_partial.json");
        std::fs::write(&path, br#"{"window":{"width":800.0,"height":600.0}}"#).unwrap();
        let (loaded, warning) = load_settings_or_default(&path);
        assert!(warning.is_none());
        assert!(loaded.dark_mode);
        assert_eq!(
            loaded.window,
            Some(WindowPrefs {
                width: 800.0,
                height: 600.0
            })
        );
        let _ = std::fs::remove_file(&path);
    }

    // ── Layout preset (additive extension of the settings file) ─────────

    fn sample_layout() -> LayoutPrefs {
        LayoutPrefs {
            tree_width: Some(240.0),
            inspector_width: Some(320.0),
            dock_height: Some(140.0),
            inspector_collapsed: true,
            dock_collapsed: false,
        }
    }

    #[test]
    fn layout_round_trip_through_the_settings_file() {
        let path = std::env::temp_dir().join("test_layout_round_trip.json");
        let _ = std::fs::remove_file(&path);

        save_layout(&path, &sample_layout()).unwrap();
        let (loaded, warning) = load_layout_or_default(&path);
        assert!(warning.is_none());
        assert_eq!(loaded, sample_layout());

        // The layout shares the settings file: the settings loader still
        // works and ignores the extra key.
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(text.contains("\"layout\""), "{text}");
        let (settings, warning) = load_settings_or_default(&path);
        assert!(warning.is_none());
        assert_eq!(settings, AppSettings::default());

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn save_settings_preserves_the_layout_key() {
        let path = std::env::temp_dir().join("test_layout_preserved_by_settings.json");
        let _ = std::fs::remove_file(&path);

        save_layout(&path, &sample_layout()).unwrap();
        // The theme/window writer must not clobber the layout preset.
        save_settings(
            &path,
            &AppSettings {
                dark_mode: false,
                window: Some(WindowPrefs {
                    width: 1000.0,
                    height: 700.0,
                }),
            },
        )
        .unwrap();

        let (layout, warning) = load_layout_or_default(&path);
        assert!(warning.is_none());
        assert_eq!(layout, sample_layout());
        let (settings, _) = load_settings_or_default(&path);
        assert!(!settings.dark_mode);
        assert_eq!(
            settings.window,
            Some(WindowPrefs {
                width: 1000.0,
                height: 700.0
            })
        );

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn legacy_settings_without_layout_yield_defaults() {
        // A settings file written before layout presets existed.
        let path = std::env::temp_dir().join("test_layout_legacy.json");
        std::fs::write(
            &path,
            br#"{"dark_mode":false,"window":{"width":900.0,"height":600.0}}"#,
        )
        .unwrap();
        let (layout, warning) = load_layout_or_default(&path);
        assert!(warning.is_none());
        assert_eq!(layout, LayoutPrefs::default());

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn missing_settings_file_yields_default_layout_without_warning() {
        let path = std::env::temp_dir().join("test_layout_missing_xyz.json");
        let _ = std::fs::remove_file(&path);
        let (layout, warning) = load_layout_or_default(&path);
        assert!(warning.is_none(), "missing file must not warn");
        assert_eq!(layout, LayoutPrefs::default());
    }

    #[test]
    fn layout_partial_json_uses_field_defaults() {
        let path = std::env::temp_dir().join("test_layout_partial.json");
        let _ = std::fs::remove_file(&path);
        std::fs::write(&path, br#"{"layout":{"tree_width":250.0}}"#).unwrap();
        let (layout, warning) = load_layout_or_default(&path);
        assert!(warning.is_none());
        assert_eq!(layout.tree_width, Some(250.0));
        assert_eq!(layout.inspector_width, None);
        assert_eq!(layout.dock_height, None);
        assert!(!layout.inspector_collapsed);
        assert!(!layout.dock_collapsed);

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn corrupt_layout_falls_back_with_warning() {
        // A failed computation is always surfaced, never a silent zero.
        let path = std::env::temp_dir().join("test_layout_corrupt.json");
        std::fs::write(&path, br#"{"layout":42}"#).unwrap();
        let (layout, warning) = load_layout_or_default(&path);
        assert_eq!(layout, LayoutPrefs::default());
        assert!(warning.is_some(), "corrupt layout must be reported");

        let _ = std::fs::remove_file(&path);
    }

    // ── Section state in the project file ─────────────────────────────

    fn sample_section() -> SectionDef {
        SectionDef {
            kind: SectionKind::IBeam,
            params: SectionParams {
                height: 300.0,
                width: 150.0,
                web_thickness: 7.0,
                thickness: 10.0,
                radius: 12.0,
                ..SectionKind::IBeam.default_params()
            },
            material: SectionMaterial {
                preset: MaterialPreset::Steel,
                custom_e: 210e9,
                custom_nu: 0.3,
            },
        }
    }

    #[test]
    fn section_def_in_project_round_trip() {
        let model = sample_model();
        let section = sample_section();
        let json = serialize_project_with_section(&model, Some(&section)).unwrap();
        let loaded = deserialize_loaded_project(&json).unwrap();
        assert_eq!(loaded.model.nodes().len(), model.nodes().len());
        assert_eq!(loaded.section, Some(section));
    }

    #[test]
    fn section_def_filesystem_round_trip() {
        let path = std::env::temp_dir().join("test_section_state_round_trip.saproject");
        let model = sample_model();
        let section = sample_section();
        save_project_with_section(&path, &model, Some(&section)).unwrap();
        let loaded = load_project_with_section(&path).unwrap();
        assert_eq!(loaded.section, Some(section));
        assert_eq!(loaded.model.members().len(), 1);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn serialized_project_without_section_omits_field() {
        // Backward compatibility: the legacy API must not emit the new field,
        // so files written before this feature are byte-for-byte unchanged.
        let json = serialize_project(&sample_model()).unwrap();
        assert!(!json.contains("\"section\""), "{json}");
    }

    #[test]
    fn old_style_v3_payload_loads_without_section() {
        // A version-3 file with no `section` key (as written before this feature).
        let json = r#"{"format_version":3,"model":{"next_node_id":2,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":3.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":1,"ref_vec":[0.0,0.0,1.0],"e":200e9,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[],"member_uniform_loads":[],"member_point_loads":[]}}"#;
        let loaded = deserialize_loaded_project(json).unwrap();
        assert!(loaded.section.is_none());
        assert_eq!(loaded.model.n_members(), 1);
    }

    #[test]
    fn old_style_v1_payload_loads_without_section() {
        let json = r#"{"format_version":1,"model":{"next_node_id":2,"next_member_id":1,"nodes":[{"id":0,"x":0.0,"y":0.0,"z":0.0},{"id":1,"x":3.0,"y":0.0,"z":0.0}],"members":[{"id":0,"node_i":0,"node_j":1,"ref_vec":[0.0,0.0,1.0],"e":200e9,"g":80e9,"area":1e-3,"iy":2e-6,"iz":1e-6,"j":3e-6}],"supports":[],"nodal_loads":[]}}"#;
        let loaded = deserialize_loaded_project(json).unwrap();
        assert!(loaded.section.is_none());
        assert_eq!(loaded.model.n_members(), 1);
    }

    #[test]
    fn unknown_fields_in_project_file_are_ignored() {
        // Forward compatibility: a future writer that adds fields must not
        // break this reader.
        let json = r#"{"format_version":3,"future_extra":42,"model":{"next_node_id":0,"next_member_id":0,"nodes":[],"members":[],"supports":[],"nodal_loads":[],"member_uniform_loads":[],"member_point_loads":[]}}"#;
        let loaded = deserialize_loaded_project(json).unwrap();
        assert!(loaded.section.is_none());
        assert_eq!(loaded.model.n_nodes(), 0);
    }
}
