//! Testable viewport modules — no GUI dependencies.
//!
//! The [`camera`], [`scene`], [`selection`], [`display`], [`model`], and
//! [`project`] modules contain all logic that can be unit-tested without a
//! display.  The binary (`main.rs`) wires these into an `eframe` app.

pub mod app_state;
pub mod camera;
pub mod display;
pub mod model;
pub mod project;
pub mod scene;
pub mod selection;

pub use app_state::{AppMessage, MessageLevel, Project};
pub use camera::Camera3D;
pub use display::ResultDisplayState;
pub use scene::ViewportScene;
pub use selection::SelectionState;
