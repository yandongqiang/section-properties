mod boolean;
mod boundary;
mod compound;
mod point;
mod polygon;

pub use boolean::{
    BoolOp, BooleanError, polygon_boolean, polygon_boolean_checked, polygon_boolean_sampled_checked,
    section_difference, section_intersection, section_union, check_boolean_bounds, union_voids,
};
pub use boundary::BoundaryExtrema;
pub use compound::{Axis, CompoundError, CompoundGeometry, Geometry, Transform, segments_interact};
pub use crate::section::Section;
pub use polygon::{JoinStyle, PolygonError};
pub use point::Point;
pub use polygon::Polygon;
