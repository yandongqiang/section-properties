//! Section work module — cross-section builder and property calculator.
//!
//! Renders into the `Section` arm of the inspector.  The user picks a shape,
//! edits its dimensions (mm) and material, sees a live preview and the
//! computed properties, and can bind the result to the currently selected
//! member.  All numerical work lives in [`viewport_poc::section_def`]; this
//! file is rendering plus a thin cache.
//!
//! Two computation tiers keep the UI responsive:
//!
//! * **Geometry** (area, second moments, centroid) is recomputed automatically
//!   whenever the definition changes — it is cheap polygon integration.
//! * **Torsion** (`J`) is computed on demand from a button.  Circles and tubes
//!   use an exact closed form and return immediately; the polygonal/thin-walled
//!   shapes need the crate's warping FEM, whose cost explodes with the boundary
//!   vertex count (a smooth circle — modelled as a 64-gon — would take minutes).
//!   That FEM solve therefore runs on a **background thread** and reports back
//!   through a channel, so the UI never freezes.

use std::sync::mpsc::{self, Receiver, TryRecvError};

use egui;

use viewport_poc::app_state::AppMessage;
use viewport_poc::edit_model::MemberId;
use viewport_poc::section_def::{
    GeometryProperties, MaterialPreset, SectionDef, SectionKind, SectionMaterial,
    TorsionProperties, outline,
};
use viewport_poc::selection::SelectionState;

use super::theme::{TextPalette, ViewportPalette};
use crate::ViewportApp;

/// Persistent Section-module state (shape definition + cached results).
#[derive(Default)]
pub struct SectionEditorState {
    /// Current definition being edited.
    pub def: SectionDef,
    /// Cached fast geometry.
    geometry: Option<GeometryCache>,
    /// Error from the last geometry computation, if any.
    geometry_error: Option<String>,
    /// Cached torsion result and the definition it belongs to.
    torsion: Option<TorsionProperties>,
    torsion_for: Option<SectionDef>,
    /// Error from the last torsion computation, if any.
    torsion_error: Option<String>,
    /// Last geometry error pushed to the message log (avoids spamming).
    reported_error: Option<String>,
    /// A background torsion FEM solve currently in flight, if any.
    pending: Option<TorsionPending>,
    /// Monotonic tag identifying the active solve; stale results are dropped.
    torsion_generation: u64,
}

/// Result sent back from a background torsion worker.
type TorsionReply = (u64, SectionDef, Result<TorsionProperties, String>);

/// Handles for one in-flight background torsion solve.
struct TorsionPending {
    /// Definition the job is solving (used to detect stale results).
    def: SectionDef,
    /// Channel the worker reports on.
    rx: Receiver<TorsionReply>,
}

/// Cached geometry + preview outline for one definition.
struct GeometryCache {
    for_def: SectionDef,
    loops: Vec<Vec<[f64; 2]>>,
    min: [f64; 2],
    max: [f64; 2],
    props: GeometryProperties,
}

impl SectionEditorState {
    /// Discard cached torsion results (called when the definition changes).
    fn invalidate_torsion(&mut self) {
        if self.torsion_for.as_ref() != Some(&self.def) {
            self.torsion = None;
            self.torsion_for = None;
            self.torsion_error = None;
        }
        // A solve in flight for a now-superseded definition is stale: drop the
        // receiver and bump the generation so its result can never be applied.
        if self.pending.as_ref().is_some_and(|p| p.def != self.def) {
            self.pending = None;
            self.torsion_generation += 1;
        }
    }

    /// True while a background torsion solve is in flight.
    pub fn torsion_computing(&self) -> bool {
        self.pending.is_some()
    }

    /// Begin a torsion computation for the current definition.
    ///
    /// Circle/tube use an exact closed form and complete immediately on the
    /// calling thread; every other shape is solved by the crate's warping FEM
    /// on a background thread.  Only one job runs at a time.
    fn start_torsion(&mut self, ctx: egui::Context) {
        if self.pending.is_some() {
            return;
        }
        // Fast analytical path (instant, no thread).
        if self.def.kind.has_analytical_torsion() {
            self.apply_torsion_result(self.def.clone(), self.def.compute_torsion());
            return;
        }
        // FEM: hand off to a worker so the UI stays responsive.
        self.torsion_generation += 1;
        let generation = self.torsion_generation;
        let def = self.def.clone();
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let result = def.compute_torsion();
            // A dropped receiver just means the job was superseded; ignore.
            let _ = tx.send((generation, def, result));
            ctx.request_repaint();
        });
        self.torsion = None;
        self.torsion_for = None;
        self.torsion_error = None;
        self.pending = Some(TorsionPending {
            def: self.def.clone(),
            rx,
        });
    }

    /// Poll the background solve, applying its result if still current.
    ///
    /// Returns a log message when the solve failed (surfaced in the bottom
    /// dock as well as the panel); `None` otherwise.  Results for a definition
    /// that has since changed are silently discarded.
    fn poll_torsion(&mut self) -> Option<AppMessage> {
        let reply: Option<TorsionReply> = {
            let p = self.pending.as_ref()?;
            match p.rx.try_recv() {
                Ok(reply) => Some(reply),
                Err(TryRecvError::Empty) => return None,
                Err(TryRecvError::Disconnected) => None,
            }
        };
        self.pending = None; // the job is finished either way

        let Some((generation, def, result)) = reply else {
            // Worker vanished without reporting a result — surface, never mask.
            let msg = "torsion worker terminated unexpectedly";
            self.torsion = None;
            self.torsion_for = None;
            self.torsion_error = Some(msg.to_string());
            return Some(AppMessage::error(format!("Section torsion: {msg}")));
        };
        if generation != self.torsion_generation || def != self.def {
            return None; // stale result for a superseded definition
        }
        let failed = self.apply_torsion_result(def, result);
        failed.map(|e| AppMessage::error(format!("Section torsion: {e}")))
    }

    /// Store a torsion result (or error) for `def`.
    ///
    /// Returns the error message when the solve failed, for logging.
    fn apply_torsion_result(
        &mut self,
        def: SectionDef,
        result: Result<TorsionProperties, String>,
    ) -> Option<String> {
        match result {
            Ok(t) => {
                self.torsion = Some(t);
                self.torsion_for = Some(def);
                self.torsion_error = None;
                None
            }
            Err(e) => {
                self.torsion = None;
                self.torsion_for = None;
                self.torsion_error = Some(e.clone());
                Some(e)
            }
        }
    }

    /// Recompute the geometry cache if the definition changed.
    fn sync_geometry(&mut self) {
        if self.geometry.as_ref().map(|g| &g.for_def) == Some(&self.def) {
            return;
        }
        match self.def.compute_geometry() {
            Ok((section, props)) => {
                let loops = outline(&section);
                self.geometry = Some(GeometryCache {
                    for_def: self.def.clone(),
                    min: outline_min(&loops),
                    max: outline_max(&loops),
                    loops,
                    props,
                });
                self.geometry_error = None;
            }
            Err(e) => {
                self.geometry = None;
                self.geometry_error = Some(e);
            }
        }
    }

    /// Cached torsion result, only if it matches the current definition.
    fn current_torsion(&self) -> Option<&TorsionProperties> {
        if self.torsion_for.as_ref() == Some(&self.def) {
            self.torsion.as_ref()
        } else {
            None
        }
    }

    /// Replace the definition wholesale (e.g. when a project is opened).
    ///
    /// Clears every cache so the restored definition is recomputed on the next
    /// frame, and discards any in-flight torsion solve for the previous
    /// definition (bumping the generation so its result can never be applied).
    pub fn set_def(&mut self, def: SectionDef) {
        self.def = def;
        self.geometry = None;
        self.geometry_error = None;
        self.torsion = None;
        self.torsion_for = None;
        self.torsion_error = None;
        self.reported_error = None;
        self.pending = None;
        self.torsion_generation += 1;
    }
}

// ── Panel ───────────────────────────────────────────────────────────────

/// Draw the Section module into the inspector panel.
pub fn show_section_module(app: &mut ViewportApp, ui: &mut egui::Ui) {
    // Theme-dependent colours (Phase P7-3): the preview canvas follows the
    // viewport palette and the panel text follows the chrome text palette,
    // so the Section module reads correctly in light mode too.
    let pal = ViewportPalette::for_theme(app.dark_mode);
    let text = TextPalette::for_theme(app.dark_mode);

    ui.heading("Section");
    ui.label(
        egui::RichText::new("Cross-section library \u{2014} compute and bind to a member")
            .color(text.help),
    );
    ui.separator();

    // ── Shape + dimensions ──────────────────────────────────────────────
    shape_controls(&mut app.section_editor, ui, text);

    // ── Material ────────────────────────────────────────────────────────
    material_controls(&mut app.section_editor, ui);

    // Refresh the fast cache against whatever the controls just produced.
    app.section_editor.invalidate_torsion();
    app.section_editor.sync_geometry();

    // Collect any torsion result the background worker has finished.
    if let Some(msg) = app.section_editor.poll_torsion() {
        app.messages.push(msg);
    }

    // Surface geometry errors once, in the message log.
    if let Some(err) = app.section_editor.geometry_error.clone() {
        if app.section_editor.reported_error.as_ref() != Some(&err) {
            app.messages
                .push(AppMessage::error(format!("Section: {err}")));
            app.section_editor.reported_error = Some(err);
        }
    } else {
        app.section_editor.reported_error = None;
    }

    ui.separator();

    // ── Preview ─────────────────────────────────────────────────────────
    ui.label(egui::RichText::new("Preview").strong());
    match app.section_editor.geometry.as_ref() {
        Some(cache) => draw_preview(ui, cache, pal),
        None => {
            egui::Frame::group(ui.style()).show(ui, |ui| {
                ui.label(egui::RichText::new("(no valid geometry to preview)").color(text.error));
            });
        }
    }

    ui.separator();

    // ── Properties ──────────────────────────────────────────────────────
    property_table(&mut app.section_editor, ui, text);

    ui.separator();

    // ── Bind to member ──────────────────────────────────────────────────
    bind_controls(app, ui, text);
}

fn shape_controls(state: &mut SectionEditorState, ui: &mut egui::Ui, text: TextPalette) {
    egui::CollapsingHeader::new("Shape")
        .default_open(true)
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Type:");
                let before = state.def.kind;
                egui::ComboBox::from_id_salt("section_kind")
                    .selected_text(before.label())
                    .show_ui(ui, |ui| {
                        for kind in SectionKind::ALL {
                            ui.selectable_value(&mut state.def.kind, kind, kind.label());
                        }
                    });
                if state.def.kind != before {
                    // Reset dimensions to the new shape's defaults.
                    state.def.params = state.def.kind.default_params();
                }
            });

            ui.label(format!("Designation: {}", state.def.designation()));
            ui.add_space(4.0);

            let p = &mut state.def.params;
            ui.horizontal_wrapped(|ui| match state.def.kind {
                SectionKind::Rectangle | SectionKind::Triangle => {
                    dim(ui, "Width", &mut p.width);
                    dim(ui, "Height", &mut p.height);
                }
                SectionKind::Circle => {
                    dim(ui, "Diameter", &mut p.diameter);
                }
                SectionKind::Tube => {
                    dim(ui, "Diameter", &mut p.diameter);
                    dim(ui, "Wall t", &mut p.thickness);
                }
                SectionKind::Box => {
                    dim(ui, "Width", &mut p.width);
                    dim(ui, "Height", &mut p.height);
                    dim(ui, "Wall t", &mut p.thickness);
                }
                SectionKind::IBeam => {
                    dim(ui, "Depth h", &mut p.height);
                    dim(ui, "Flange b", &mut p.width);
                    dim(ui, "Web tw", &mut p.web_thickness);
                    dim(ui, "Flange tf", &mut p.thickness);
                    dim(ui, "Root r", &mut p.radius);
                }
            });
            ui.label(
                egui::RichText::new("Dimensions in mm; properties in SI (m², m⁴).")
                    .color(text.help),
            );
        });
}

/// One labelled dimension field (mm).
fn dim(ui: &mut egui::Ui, label: &str, value: &mut f64) {
    ui.label(format!("{label}:"));
    ui.add(
        egui::DragValue::new(value)
            .speed(1.0)
            .range(0.0..=100_000.0)
            .suffix(" mm"),
    );
}

fn material_controls(state: &mut SectionEditorState, ui: &mut egui::Ui) {
    egui::CollapsingHeader::new("Material")
        .default_open(true)
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Preset:");
                let before = state.def.material.preset;
                egui::ComboBox::from_id_salt("section_material")
                    .selected_text(before.label())
                    .show_ui(ui, |ui| {
                        for preset in MaterialPreset::ALL {
                            ui.selectable_value(
                                &mut state.def.material.preset,
                                preset,
                                preset.label(),
                            );
                        }
                    });
                if state.def.material.preset != before
                    && state.def.material.preset != MaterialPreset::Custom
                {
                    // Adopt the preset's numbers into the custom slots so a
                    // later switch to Custom starts from sensible values.
                    let m = SectionMaterial {
                        preset: state.def.material.preset,
                        ..state.def.material
                    }
                    .material();
                    state.def.material.custom_e = m.youngs_modulus;
                    state.def.material.custom_nu = m.poissons_ratio;
                }
            });

            if state.def.material.preset == MaterialPreset::Custom {
                ui.horizontal(|ui| {
                    ui.label("E:");
                    ui.add(
                        egui::DragValue::new(&mut state.def.material.custom_e)
                            .speed(1e8)
                            .range(1.0..=1e13)
                            .custom_formatter(|v, _| format!("{:.3} GPa", v * 1e-9)),
                    );
                    ui.label("ν:");
                    ui.add(
                        egui::DragValue::new(&mut state.def.material.custom_nu)
                            .speed(0.01)
                            .range(-0.99..=0.49)
                            .fixed_decimals(3),
                    );
                });
            }

            let props = state.geometry.as_ref().map(|g| &g.props);
            let (e, g, nu) = match props {
                Some(p) => (p.youngs_modulus, p.shear_modulus, p.poissons_ratio),
                None => {
                    let m = state.def.material.material();
                    (m.youngs_modulus, m.shear_modulus, m.poissons_ratio)
                }
            };
            ui.label(format!("E = {:.3} GPa", e * 1e-9));
            ui.label(format!("G = {:.3} GPa", g * 1e-9));
            ui.label(format!("ν = {nu:.3}"));
        });
}

fn property_table(state: &mut SectionEditorState, ui: &mut egui::Ui, text: TextPalette) {
    ui.label(egui::RichText::new("Properties").strong());

    if let Some(err) = state.geometry_error.clone() {
        ui.label(egui::RichText::new(format!("\u{26a0} {err}")).color(text.error));
        return;
    }
    let Some(cache) = state.geometry.as_ref() else {
        return;
    };
    let p = cache.props;

    egui::Grid::new("section_props_grid")
        .num_columns(3)
        .spacing([10.0, 3.0])
        .striped(true)
        .show(ui, |ui| {
            ui.strong("Property");
            ui.strong("SI");
            ui.strong("Other");
            ui.end_row();

            row(
                ui,
                "A",
                format!("{:.6e} m²", p.area),
                format!("{:.1} mm²", p.area * 1e6),
            );
            row(
                ui,
                "Iy (local y)",
                format!("{:.6e} m⁴", p.iy),
                format!("{:.1} mm⁴", p.iy * 1e12),
            );
            row(
                ui,
                "Iz (local z)",
                format!("{:.6e} m⁴", p.iz),
                format!("{:.1} mm⁴", p.iz * 1e12),
            );
            row(ui, "Ixy", format!("{:.6e} m⁴", p.ixy), String::new());
            row(
                ui,
                "I11 (major)",
                format!("{:.6e} m⁴", p.i11),
                String::new(),
            );
            row(
                ui,
                "I22 (minor)",
                format!("{:.6e} m⁴", p.i22),
                String::new(),
            );
            row(
                ui,
                "φ",
                format!("{:.4} rad", p.phi),
                format!("{:.3}°", p.phi.to_degrees()),
            );
            row(
                ui,
                "Centroid",
                format!("({:.4e}, {:.4e}) m", p.cx, p.cy),
                format!("({:.2}, {:.2}) mm", p.cx * 1e3, p.cy * 1e3),
            );
            row(ui, "E·A", format!("{:.4e} N", p.ea), String::new());
            row(ui, "E·Iy", format!("{:.4e} N·m²", p.eiy), String::new());
            row(ui, "E·Iz", format!("{:.4e} N·m²", p.eiz), String::new());
        });

    // Torsion (J): closed-form for circles/tubes, background FEM otherwise.
    ui.add_space(4.0);
    let computing = state.torsion_computing();
    if computing {
        ui.horizontal(|ui| {
            ui.spinner();
            ui.label(egui::RichText::new("Computing torsion (J)\u{2026}").color(text.help));
        });
    } else {
        match state.current_torsion().copied() {
            Some(t) => {
                egui::Grid::new("section_torsion_grid")
                    .num_columns(3)
                    .spacing([10.0, 3.0])
                    .striped(true)
                    .show(ui, |ui| {
                        row(
                            ui,
                            "J (torsion)",
                            format!("{:.6e} m⁴", t.j),
                            format!("{:.1} mm⁴", t.j * 1e12),
                        );
                        row(
                            ui,
                            "Iw (warping)",
                            format!("{:.6e} m⁶", t.iw),
                            String::new(),
                        );
                        row(ui, "G·J", format!("{:.4e} N·m²", t.gj), String::new());
                    });
            }
            None => {
                ui.label(egui::RichText::new("J (torsion): not computed").color(text.stale));
            }
        }
        if let Some(err) = state.torsion_error.clone() {
            ui.label(egui::RichText::new(format!("\u{26a0} {err}")).color(text.error));
        }
    }

    let has_result = state.current_torsion().is_some();
    let label = if computing {
        "Computing\u{2026}"
    } else if has_result {
        "Recompute torsion (J)"
    } else {
        "Compute torsion (J)"
    };
    let enabled = !computing && state.geometry.is_some();
    if ui.add_enabled(enabled, egui::Button::new(label)).clicked() {
        let ctx = ui.ctx().clone();
        state.start_torsion(ctx);
    }
    if !computing {
        let note = if state.def.kind.has_analytical_torsion() {
            "Closed-form J \u{2014} computed instantly (no FEM)."
        } else {
            "Solved by the warping FEM in the background; the UI stays responsive."
        };
        ui.label(egui::RichText::new(note).color(text.help));
    }
}

fn row(ui: &mut egui::Ui, name: &str, si: String, other: String) {
    ui.label(name);
    ui.monospace(si);
    ui.monospace(other);
    ui.end_row();
}

// ── Binding ─────────────────────────────────────────────────────────────

fn bind_controls(app: &mut ViewportApp, ui: &mut egui::Ui, text: TextPalette) {
    ui.label(egui::RichText::new("Bind to member").strong());

    let Some(props) = app.section_editor.geometry.as_ref().map(|g| g.props) else {
        ui.label(egui::RichText::new("(fix the geometry error first)").color(text.error));
        return;
    };
    let torsion = app.section_editor.current_torsion().copied();

    let target = match app.selection {
        SelectionState::Member(id) => Some(id),
        _ => None,
    };

    let ready = target.is_some() && torsion.is_some();
    let response = ui.add_enabled(ready, egui::Button::new("Apply to selected member"));

    if target.is_none() {
        response
            .clone()
            .on_disabled_hover_text("Select a member in the viewport or tree first.");
    } else if torsion.is_none() {
        response.clone().on_disabled_hover_text(
            "Press \u{201c}Compute torsion (J)\u{201d} first \u{2014} the member needs a positive J.",
        );
    }

    if let (true, Some(id), Some(t)) = (response.clicked(), target, torsion) {
        apply_to_member(app, id, &props, t.j);
    }

    ui.label(
        egui::RichText::new(
            "Writes E, G, A, Iy, Iz, J into the member's section (visible/tweakable on the Model tab).",
        )
        .color(text.help),
    );
}

/// Fill the member edit buffer with the computed values and apply it.
///
/// Reuses the existing member-edit flow ([`ViewportApp::apply_member_edit`])
/// so there is a single write path to the model.
fn apply_to_member(app: &mut ViewportApp, id: MemberId, props: &GeometryProperties, j: f64) {
    app.sync_member_edit_buffer(id);
    app.member_edit.e = props.youngs_modulus;
    app.member_edit.g = props.shear_modulus;
    app.member_edit.area = props.area;
    app.member_edit.iy = props.iy;
    app.member_edit.iz = props.iz;
    app.member_edit.j = j;
    app.apply_member_edit(id);
}

// ── Preview painting ────────────────────────────────────────────────────

/// Paint the 2-D section preview.
///
/// The preview canvas uses the active theme's [`ViewportPalette`] (the same
/// background / member / node colours as the 3-D viewport), so it no longer
/// stays dark-tuned in light mode (Phase P7-3).
fn draw_preview(ui: &mut egui::Ui, cache: &GeometryCache, pal: ViewportPalette) {
    let width = ui.available_width().clamp(120.0, 320.0);
    let (rect, _resp) = ui.allocate_exact_size(egui::vec2(width, 200.0), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 3.0, pal.bg);

    let span_x = (cache.max[0] - cache.min[0]).max(1e-9);
    let span_y = (cache.max[1] - cache.min[1]).max(1e-9);
    let scale = (rect.width() * 0.85 / span_x as f32).min(rect.height() * 0.85 / span_y as f32);
    let cx = (cache.min[0] + cache.max[0]) / 2.0;
    let cy = (cache.min[1] + cache.max[1]) / 2.0;
    let center = rect.center();

    let to_screen = |p: [f64; 2]| -> egui::Pos2 {
        egui::pos2(
            center.x + (p[0] - cx) as f32 * scale,
            center.y - (p[1] - cy) as f32 * scale,
        )
    };

    for (i, loop_pts) in cache.loops.iter().enumerate() {
        if loop_pts.len() < 2 {
            continue;
        }
        let pts: Vec<egui::Pos2> = loop_pts.iter().map(|p| to_screen(*p)).collect();
        let color = if i == 0 { pal.member } else { pal.help };
        painter.add(egui::Shape::closed_line(
            pts,
            egui::Stroke::new(1.5_f32, color),
        ));
    }

    // Centroid crosshair.
    let c = to_screen([cache.props.cx, cache.props.cy]);
    let stroke = egui::Stroke::new(1.0_f32, pal.node);
    painter.line_segment(
        [egui::pos2(c.x - 4.0, c.y), egui::pos2(c.x + 4.0, c.y)],
        stroke,
    );
    painter.line_segment(
        [egui::pos2(c.x, c.y - 4.0), egui::pos2(c.x, c.y + 4.0)],
        stroke,
    );
}

fn outline_min(loops: &[Vec<[f64; 2]>]) -> [f64; 2] {
    let mut m = [f64::INFINITY; 2];
    for lp in loops {
        for p in lp {
            m[0] = m[0].min(p[0]);
            m[1] = m[1].min(p[1]);
        }
    }
    if m[0].is_finite() { m } else { [0.0, 0.0] }
}

fn outline_max(loops: &[Vec<[f64; 2]>]) -> [f64; 2] {
    let mut m = [f64::NEG_INFINITY; 2];
    for lp in loops {
        for p in lp {
            m[0] = m[0].max(p[0]);
            m[1] = m[1].max(p[1]);
        }
    }
    if m[0].is_finite() { m } else { [0.0, 0.0] }
}
