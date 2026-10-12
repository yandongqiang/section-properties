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

use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::edit_model::{
    EditMember, EditModel, EditNode, MemberId, MemberUniformLoad3D, NodalLoadSpec, NodeId,
    SupportSpec,
};
use structural_analysis::FrameSection3D;

// ── File format version ───────────────────────────────────────────────

/// Currently supported file format version.
pub const CURRENT_FORMAT_VERSION: u32 = 2;

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

fn model_to_dto_v2(model: &EditModel) -> ModelFileV2 {
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
    ModelFileV2 {
        next_node_id: v1.next_node_id,
        next_member_id: v1.next_member_id,
        nodes: v1.nodes,
        members: v1.members,
        supports: v1.supports,
        nodal_loads: v1.nodal_loads,
        member_uniform_loads,
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

/// Serialize an EditModel to a JSON string.
pub fn serialize_project(model: &EditModel) -> Result<String, PersistenceError> {
    let file = ProjectFileV2 {
        format_version: CURRENT_FORMAT_VERSION,
        model: model_to_dto_v2(model),
    };
    serde_json::to_string_pretty(&file).map_err(|e| PersistenceError::JsonError(e.to_string()))
}

/// Deserialize a JSON string to an EditModel.
///
/// Supports both version 1 (legacy, no member uniform loads) and
/// version 2 (current, with member uniform loads) files.
pub fn deserialize_project(json: &str) -> Result<EditModel, PersistenceError> {
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
            dto_to_model(&file.model)
        }
        2 => {
            let file: ProjectFileV2 = serde_json::from_str(json)
                .map_err(|e| PersistenceError::JsonError(e.to_string()))?;
            dto_v2_to_model(&file.model)
        }
        found => Err(PersistenceError::UnsupportedVersion {
            found,
            supported: CURRENT_FORMAT_VERSION,
        }),
    }
}

/// Save an EditModel to a `.saproject` file.
///
/// Writes to a temporary file first, then renames atomically.
/// On Windows, rename-over-existing is not guaranteed atomic, but
/// is the best available strategy without requiring filesystem locks.
pub fn save_project(path: &Path, model: &EditModel) -> Result<(), PersistenceError> {
    let json = serialize_project(model)?;

    let mut tmp_path = path.to_path_buf();
    let stem = path
        .file_stem()
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| "tmp".to_string());
    tmp_path.set_file_name(format!(".{stem}.saproject.tmp"));

    std::fs::write(&tmp_path, json.as_bytes())
        .map_err(|e| PersistenceError::IoError(e.to_string()))?;

    std::fs::rename(&tmp_path, path).map_err(|e| {
        let _ = std::fs::remove_file(&tmp_path);
        PersistenceError::IoError(e.to_string())
    })?;

    Ok(())
}

/// Load an EditModel from a `.saproject` file.
///
/// Reads and validates the file fully before returning.
/// Does not modify any existing model — the caller is responsible
/// for transactional replacement.
pub fn load_project(path: &Path) -> Result<EditModel, PersistenceError> {
    let contents =
        std::fs::read_to_string(path).map_err(|e| PersistenceError::IoError(e.to_string()))?;
    deserialize_project(&contents)
}

// ── Tests ────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::edit_model::{EditModel, MemberId, NodalLoadSpec, SupportSpec};

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
}
