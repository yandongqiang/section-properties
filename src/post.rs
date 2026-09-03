//! Post-processing utilities mirroring Python `sectionproperties.post`.
pub mod fibre;
pub use fibre::{Fiber, to_fibre_from_mesh, to_fibre_section, total_area};
