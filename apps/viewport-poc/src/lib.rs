//! Testable viewport modules — no GUI dependencies.
//!
//! The [`camera`], [`scene`], [`selection`], [`display`], [`model`], and
//! [`project`] modules contain all logic that can be unit-tested without a
//! display.  The binary (`main.rs`) wires these into an `eframe` app.

pub mod app_state;
pub mod camera;
pub mod commands;
pub mod display;
pub mod edit_history;
pub mod edit_model;
pub mod model;
pub mod persistence;
pub mod project;
pub mod scene;
pub mod section_def;
pub mod selection;

pub use app_state::{AppMessage, MessageLevel, Project};
pub use camera::Camera3D;
pub use display::ResultDisplayState;
pub use edit_history::EditHistory;
pub use edit_model::{
    EditError, EditMember, EditModel, EditNode, MemberId, MemberPointLoad3D, MemberUniformLoad3D,
    NodalLoadSpec, NodeId, SupportSpec,
};
pub use scene::ViewportScene;
pub use section_def::{
    GeometryProperties, MaterialPreset, SectionDef, SectionKind, SectionMaterial, SectionParams,
    SectionResult, TorsionProperties,
};
pub use selection::SelectionState;
