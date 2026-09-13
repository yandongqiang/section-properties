#!/usr/bin/env python3
"""Generate deterministic Python `sectionproperties` reference data for the
Rust/Python cross-validation suite.

Writes one JSON file per case plus an index into
`tests/reference/cross_validation/`.

Design notes
------------
* The **exact geometry coordinates** are exported (outer ring + hole rings), so
  the Rust side can rebuild a byte-identical polygon rather than relying on a
  second, possibly divergent, parameterisation of the same section. This makes
  the comparison a true differential test of the property pipelines.
* Geometry is deterministic: no randomness, no timestamps in the JSON payload.
* Environment versions are recorded so a reference set is never compared
  against a different implementation without noticing.

Usage:
    python generate_cross_validation_reference.py
"""

import json
import os
import sys
from typing import Any, Dict, List, Optional

import importlib.metadata as md
import numpy as np
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.affinity import rotate as shapely_rotate
from shapely.affinity import scale as shapely_scale

from sectionproperties.pre import Geometry
from sectionproperties.pre.library import (
    angle_section,
    channel_section,
    circular_section,
    i_section,
    rectangular_hollow_section,
    rectangular_section,
    tee_section,
    zed_section,
)
from sectionproperties.analysis import Section

OUT_DIR = os.path.join("tests", "reference", "cross_validation")


def env_metadata() -> Dict[str, str]:
    meta = {
        "python": sys.version.split()[0],
        "generator": "generate_cross_validation_reference.py",
    }
    for pkg in ("sectionproperties", "numpy", "scipy", "shapely"):
        try:
            meta[pkg] = md.version(pkg)
        except Exception:
            meta[pkg] = "unknown"
    return meta


def _f(x: Any) -> Optional[float]:
    """Convert numpy scalars / None to plain floats for JSON."""
    if x is None:
        return None
    return float(x)


def rings_of(geom: Geometry) -> Dict[str, Any]:
    """Extract outer ring and hole rings from a sectionproperties Geometry."""
    shp = geom.geom
    outer = [(float(x), float(y)) for x, y in list(shp.exterior.coords)]
    # drop the duplicated closing point if present
    if len(outer) > 1 and outer[0] == outer[-1]:
        outer = outer[:-1]
    holes = []
    for interior in shp.interiors:
        ring = [(float(x), float(y)) for x, y in list(interior.coords)]
        if len(ring) > 1 and ring[0] == ring[-1]:
            ring = ring[:-1]
        holes.append(ring)
    return {"outer": outer, "holes": holes}


def compute_reference(geom: Geometry, mesh_size: float, with_warping: bool) -> Dict[str, Any]:
    geom.create_mesh(mesh_sizes=[mesh_size])
    sec = Section(geom)
    sec.calculate_geometric_properties()

    area = _f(sec.get_area())
    cx, cy = sec.get_c()
    q = sec.get_q()
    ixx, iyy, ixy = sec.get_ic()
    i11, i22 = sec.get_ip()
    phi = _f(sec.get_phi())
    zxx_p, zxx_m, zyy_p, zyy_m = sec.get_z()
    rx, ry = sec.get_rc()

    out: Dict[str, Any] = {
        "area": area,
        "centroid": [_f(cx), _f(cy)],
        "q": [_f(q[0]), _f(q[1])],
        "ixx": _f(ixx),
        "iyy": _f(iyy),
        "ixy": _f(ixy),
        "i11": _f(i11),
        "i22": _f(i22),
        "phi": phi,
        "zxx_plus": _f(zxx_p),
        "zxx_minus": _f(zxx_m),
        "zyy_plus": _f(zyy_p),
        "zyy_minus": _f(zyy_m),
        "rx": _f(rx),
        "ry": _f(ry),
    }

    out["warping"] = None
    out["warping_error"] = None
    if with_warping:
        try:
            sec.calculate_warping_properties()
            sc = sec.get_sc()
            out["warping"] = {
                "j": _f(sec.get_j()),
                "iw": _f(sec.get_gamma()),
                "shear_centre": [_f(sc[0]), _f(sc[1])],
            }
        except Exception as exc:  # reference-side failure, recorded not fatal
            out["warping_error"] = f"{type(exc).__name__}: {exc}"
            print(f"      [warning] warping failed: {type(exc).__name__}: {exc}")
    return out


def emit(name: str, description: str, geom: Geometry, mesh_size: float,
         with_warping: bool, index: List[Dict[str, Any]]) -> None:
    rings = rings_of(geom)
    props = compute_reference(geom, mesh_size, with_warping)
    payload = {
        "case": name,
        "description": description,
        "environment": env_metadata(),
        "mesh_size": mesh_size,
        "geometry": rings,
        "properties": props,
    }
    path = os.path.join(OUT_DIR, f"{name}.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2, sort_keys=True)
    index.append({
        "case": name,
        "description": description,
        "mesh_size": mesh_size,
        "warping": props["warping"] is not None,
        "warping_error": props["warping_error"],
        "file": f"{name}.json",
    })
    print(f"  wrote {path}  (A={props['area']:.6g}, warping={'yes' if with_warping else 'no'})")


def rect_with_hole(w: float, h: float, hole_r: float, hole_c: tuple, n: int = 48) -> Geometry:
    shell = [(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)]
    cx, cy = hole_c
    hole = [
        (cx + hole_r * np.cos(2 * np.pi * k / n), cy + hole_r * np.sin(2 * np.pi * k / n))
        for k in range(n)
    ]
    return Geometry(ShapelyPolygon(shell, [hole]))


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    index: List[Dict[str, Any]] = []
    print("Generating Python reference corpus...")

    # ---- basic shapes -------------------------------------------------------
    emit("rect_100x50", "Rectangle 100 (depth) x 50 (width), no holes",
         rectangular_section(d=100, b=50), 5.0, True, index)
    emit("square_100", "Square 100 x 100",
         rectangular_section(d=100, b=100), 5.0, True, index)
    emit("circle_r50", "Circle radius 50 as a 64-segment polygon",
         circular_section(d=100, n=64), 5.0, True, index)
    emit("triangle_right", "Right triangle (0,0)-(100,0)-(0,80)",
         Geometry(ShapelyPolygon([(0.0, 0.0), (100.0, 0.0), (0.0, 80.0)])), 5.0, True, index)

    # ---- structural sections ------------------------------------------------
    emit("i_300x150", "I section 300 deep, 150 wide, tf=12, tw=8",
         i_section(d=300, b=150, t_f=12, t_w=8, r=0, n_r=0), 10.0, True, index)
    emit("channel_200x75", "Channel 200 deep, 75 wide, tf=10, tw=8",
         channel_section(d=200, b=75, t_f=10, t_w=8, r=0, n_r=0), 6.0, True, index)
    emit("angle_100x100", "Equal angle 100 x 100 x 8 (asymmetric)",
         angle_section(d=100, b=100, t=8, r_r=0, r_t=0, n_r=0), 4.0, True, index)
    emit("tee_200x100", "Tee section 200 deep, 100 wide, tf=12, tw=8",
         tee_section(d=200, b=100, t_f=12, t_w=8, r=0, n_r=0), 6.0, True, index)
    emit("rhs_200x100", "Rectangular hollow section 200 x 100 x 8",
         rectangular_hollow_section(d=200, b=100, t=8, r_out=0, n_r=0), 5.0, True, index)
    emit("zed_200x100", "Zed section 200 x 100, t=8 (asymmetric, mono-symmetric axis)",
         zed_section(d=200, b_l=60, b_r=60, l=100, t=8, r_out=0, n_r=0), 6.0, True, index)

    # ---- hole cases ---------------------------------------------------------
    emit("rect_hole_centered", "200 x 100 rectangle with centred circular hole r=25",
         rect_with_hole(200, 100, 25, (100, 50)), 6.0, True, index)
    emit("rect_hole_eccentric", "200 x 100 rectangle with eccentric circular hole r=20 at (60,30)",
         rect_with_hole(200, 100, 20, (60, 30)), 6.0, True, index)
    emit("rhs_thin_wall", "Thin-walled RHS 200 x 100 x 3",
         rectangular_hollow_section(d=200, b=100, t=3, r_out=0, n_r=0), 3.0, True, index)

    # ---- rotation sub-corpus (asymmetric section) ---------------------------
    base = angle_section(d=100, b=100, t=8, r_r=0, r_t=0, n_r=0)
    for ang in (45.0, 90.0, -45.0, 135.0):
        tag = f"angle_rot{int(ang) if ang > 0 else 'm' + str(abs(int(ang)))}"
        rotated = Geometry(shapely_rotate(base.geom, ang, origin=(0.0, 0.0)))
        emit(tag, f"Angle 100x100x8 rotated {ang} degrees about the origin",
             rotated, 4.0, True, index)

    # ---- scaling sub-corpus -------------------------------------------------
    for alpha, tag in ((1e-3, "m3"), (1e3, "p3")):
        srect = Geometry(
            shapely_scale(rectangular_section(d=100, b=50).geom, alpha, alpha, origin=(0.0, 0.0))
        )
        emit(f"rect_scale_{tag}",
             f"Rectangle 100x50 geometrically scaled by alpha={alpha:g}",
             srect, 5.0 * alpha, True, index)
        si = Geometry(
            shapely_scale(i_section(d=300, b=150, t_f=12, t_w=8, r=0, n_r=0).geom,
                          alpha, alpha, origin=(0.0, 0.0))
        )
        emit(f"i_scale_{tag}",
             f"I section 300x150 geometrically scaled by alpha={alpha:g}",
             si, 10.0 * alpha, True, index)

    meta = {
        "environment": env_metadata(),
        "notes": (
            "Deterministic Python sectionproperties reference data for the "
            "Rust/Python differential cross-validation suite. Geometry "
            "coordinates are exported so the Rust side rebuilds an identical "
            "polygon; do not compare polygon vertex ordering."
        ),
        "cases": index,
    }
    with open(os.path.join(OUT_DIR, "index.json"), "w", encoding="utf-8") as fh:
        json.dump(meta, fh, indent=2, sort_keys=True)
    print(f"\nWrote index with {len(index)} cases to {os.path.join(OUT_DIR, 'index.json')}")


if __name__ == "__main__":
    main()
