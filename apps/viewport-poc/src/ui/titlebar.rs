//! Compact title bar plus the custom window chrome for the undecorated window.
//!
//! The OS window decoration is disabled (`with_decorations(false)` in
//! `main.rs`), so this module draws everything the native title bar used to
//! provide:
//!
//! * the **project name**, with a dirty marker (`●`) when there are unsaved
//!   changes (an empty project reads "Untitled");
//! * **panel visibility toggles** for the left Project panel, the right panel
//!   and the bottom Output dock, each highlighting the panel's current state;
//! * **window controls** at the far right — minimize, maximize/restore and
//!   close — drawn with the same line-art glyphs as the rest of the chrome;
//! * the **title-bar drag surface**: pressing the empty area moves the window
//!   (`ViewportCommand::StartDrag`) and double-clicking it toggles maximize;
//! * the **custom resize grips** ([`show_resize_grips`]) — a few thin hit
//!   zones along the window's outer edges and corners, which start a native
//!   resize (`ViewportCommand::BeginResize`).  The north (top) grip is
//!   **split around the title-bar controls** so a press on any control's top
//!   pixels still reaches the control instead of starting a resize.
//!
//! All of this is pure view state: toggling a panel, moving the window or
//! resizing it never touches the model, the selection or the analysis.  The
//! command palette is **not** triggered from here — it stays reachable by its
//! `Ctrl+Shift+P` shortcut (handled in `main.rs`) and from the Help menu.

use egui;

use super::icons;
use super::theme::{self, TextPalette};
use crate::ViewportApp;

/// Thickness of the custom resize-grip band at the window edge (points).
///
/// Kept to the outermost few pixels so the grips never steal clicks from the
/// viewport or the panels, which sit inside this band.  Matches the 4–6 px
/// range expected of a custom resize border.
const RESIZE_BAND: f32 = 5.0;

/// Draw the title bar.
pub fn show_title_bar(app: &mut ViewportApp, ctx: &egui::Context) {
    // Rects of every interactive control drawn on the bar this frame.  They are
    // published *after* the panel is laid out so [`show_resize_grips`] can keep
    // the north (top) resize band off them — a press on a control's top pixels
    // must press the control, never start a resize.
    let mut controls: Vec<egui::Rect> = Vec::new();
    egui::TopBottomPanel::top("title_bar").show(ctx, |ui| {
        // The whole bar is a drag / double-click region.  It is registered
        // *before* the controls so the buttons (added later) win the hit-test
        // wherever they overlap: egui resolves overlapping widgets to the
        // top-most (last-added) one, and the buttons sense clicks, so a press
        // on a button never routes through this drag region.
        let bar_rect = ui.max_rect();
        let drag = ui.interact(
            bar_rect,
            ui.id().with("title_bar_drag"),
            egui::Sense::click(),
        );
        if drag.double_clicked() {
            toggle_maximize(ctx);
        }
        if drag.is_pointer_button_down_on() {
            // Hand the press to the OS so the whole window follows the pointer
            // until release — the standard title-bar move.
            ctx.send_viewport_cmd(egui::ViewportCommand::StartDrag);
        }

        ui.horizontal(|ui| {
            project_title(app, ui);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                // Added first in the right-to-left layout → right-most, so the
                // controls sit at the far right, after the panel toggles.
                controls.extend(window_controls(ctx, ui));
                controls.extend(panel_toggles(app, ui));
            });
        });
    });
    store_control_rects(ctx, controls);
}

/// Project name plus a dirty marker when the model has unsaved changes.
fn project_title(app: &ViewportApp, ui: &mut egui::Ui) {
    let text = TextPalette::for_theme(app.dark_mode);
    let name = if app.project.is_empty() {
        "Untitled".to_string()
    } else {
        app.project.name.clone()
    };
    ui.label(theme::header(name));
    if app.is_dirty() {
        ui.label(theme::header("\u{25cf}").color(text.stale))
            .on_hover_text("Unsaved changes");
    }
}

/// Small icon toggles for the three collapsible panels.
///
/// Each button is highlighted while its panel is visible; clicking flips the
/// panel's collapsed flag (pure view state).  Returns the rects of the toggles
/// so the north resize band can avoid them.
fn panel_toggles(app: &mut ViewportApp, ui: &mut egui::Ui) -> Vec<egui::Rect> {
    let mut rects = Vec::with_capacity(3);
    let response = icons::icon_toggle(
        ui,
        icons::panel_bottom,
        !app.bottom_dock_collapsed,
        "Show / hide the Output dock (Ctrl+M)",
    );
    if response.clicked() {
        app.bottom_dock_collapsed = !app.bottom_dock_collapsed;
    }
    rects.push(response.rect);

    let response = icons::icon_toggle(
        ui,
        icons::panel_right,
        !app.inspector_collapsed,
        "Show / hide the right panel (Ctrl+I)",
    );
    if response.clicked() {
        app.inspector_collapsed = !app.inspector_collapsed;
    }
    rects.push(response.rect);

    let response = icons::icon_toggle(
        ui,
        icons::panel_left,
        !app.left_panel_collapsed,
        "Show / hide the Project panel",
    );
    if response.clicked() {
        app.left_panel_collapsed = !app.left_panel_collapsed;
    }
    rects.push(response.rect);

    rects
}

/// The three window controls at the far right of the title bar.
///
/// Added right-to-left (this runs inside a `right_to_left` layout), so the
/// visual order across the bar is minimize · maximize/restore · close, matching
/// the platform convention.  Each sends a [`egui::ViewportCommand`]; none of
/// them touches application state.  Returns the rects of the buttons so the
/// north resize band can avoid them.
fn window_controls(ctx: &egui::Context, ui: &mut egui::Ui) -> Vec<egui::Rect> {
    let mut rects = Vec::with_capacity(3);
    let response = icons::window_control_button(ui, icons::window_close, "Close", true);
    if response.clicked() {
        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
    }
    rects.push(response.rect);

    let maximized = is_maximized(ctx);
    let (icon, tip) = if maximized {
        (maximize_icon(true), "Restore")
    } else {
        (maximize_icon(false), "Maximize")
    };
    let response = icons::window_control_button(ui, icon, tip, false);
    if response.clicked() {
        ctx.send_viewport_cmd(egui::ViewportCommand::Maximized(!maximized));
    }
    rects.push(response.rect);

    let response = icons::window_control_button(ui, icons::window_minimize, "Minimize", false);
    if response.clicked() {
        ctx.send_viewport_cmd(egui::ViewportCommand::Minimized(true));
    }
    rects.push(response.rect);

    rects
}

/// Whether the window is currently maximized (`None`/unknown → not maximized).
fn is_maximized(ctx: &egui::Context) -> bool {
    ctx.input(|i| i.viewport().maximized).unwrap_or(false)
}

/// Flip the maximized state (used by the double-click-to-maximize gesture).
fn toggle_maximize(ctx: &egui::Context) {
    ctx.send_viewport_cmd(egui::ViewportCommand::Maximized(!is_maximized(ctx)));
}

/// The glyph for the maximize/restore control: a "restore" glyph while the
/// window is maximized (clicking it restores), a "maximize" square otherwise.
pub fn maximize_icon(maximized: bool) -> icons::IconFn {
    if maximized {
        icons::window_restore
    } else {
        icons::window_maximize
    }
}

// ── Title-bar control rects (published for the resize grips) ─────────────

/// egui data-map key under which the title-bar control rects are stashed for
/// the current frame.
///
/// The title bar ([`show_title_bar`]) and the resize grips
/// ([`show_resize_grips`]) are drawn as two separate passes; this carries the
/// control geometry between them without widening [`ViewportApp`] (whose shape
/// is frozen by `ui/actions.rs`).  Kept in egui memory only — never persisted.
fn control_rects_id() -> egui::Id {
    egui::Id::new("title_bar_control_rects")
}

/// Record the title-bar control rects drawn this frame.
fn store_control_rects(ctx: &egui::Context, rects: Vec<egui::Rect>) {
    ctx.data_mut(|d| d.insert_temp(control_rects_id(), rects));
}

/// The title-bar control rects drawn this frame (empty before the bar shows).
fn control_rects(ctx: &egui::Context) -> Vec<egui::Rect> {
    ctx.data(|d| d.get_temp(control_rects_id()).unwrap_or_default())
}

// ── Custom resize grips (undecorated window) ─────────────────────────────

/// The eight resize zones of `screen`: four corners plus the four edge strips
/// between them (so the zones do not overlap).
///
/// Kept a pure function of the screen rect and band width so the point →
/// direction hit-test below — and its tests — share exactly one definition of
/// where the zones are.
pub fn resize_zones(screen: egui::Rect, band: f32) -> [(egui::ResizeDirection, egui::Rect); 8] {
    use egui::ResizeDirection as D;
    let (l, r, t, b) = (screen.left(), screen.right(), screen.top(), screen.bottom());
    let x0 = l + band;
    let x1 = r - band;
    let y0 = t + band;
    let y1 = b - band;
    let rect = |a: egui::Pos2, c: egui::Pos2| egui::Rect::from_min_max(a, c);
    [
        (D::NorthWest, rect(egui::pos2(l, t), egui::pos2(x0, y0))),
        (D::NorthEast, rect(egui::pos2(x1, t), egui::pos2(r, y0))),
        (D::SouthWest, rect(egui::pos2(l, y1), egui::pos2(x0, b))),
        (D::SouthEast, rect(egui::pos2(x1, y1), egui::pos2(r, b))),
        (D::North, rect(egui::pos2(x0, t), egui::pos2(x1, y0))),
        (D::South, rect(egui::pos2(x0, y1), egui::pos2(x1, b))),
        (D::West, rect(egui::pos2(l, y0), egui::pos2(x0, y1))),
        (D::East, rect(egui::pos2(x1, y0), egui::pos2(r, y1))),
    ]
}

/// The resizable rects of `screen`, with the north (top) edge **split around**
/// the `occupied` title-bar control rects.
///
/// `occupied` are the interactive title-bar controls (panel toggles, window
/// buttons) drawn this frame.  The plain [`resize_zones`] north strip is a
/// single full-width band, whose top pixels overlap the controls and would
/// steal their clicks; here the north band is reduced to the horizontal spans
/// *between* the controls, so every control keeps all of its pixels while the
/// empty stretches of the bar stay resizable.  The other seven zones are
/// returned unchanged.
pub fn resize_grips(
    screen: egui::Rect,
    band: f32,
    occupied: &[egui::Rect],
) -> Vec<(egui::ResizeDirection, egui::Rect)> {
    use egui::ResizeDirection as D;
    let mut grips: Vec<(D, egui::Rect)> = Vec::with_capacity(8);
    let mut north: Option<egui::Rect> = None;
    for (dir, rect) in resize_zones(screen, band) {
        if dir == D::North {
            north = Some(rect);
        } else {
            grips.push((dir, rect));
        }
    }
    if let Some(strip) = north {
        for piece in split_horizontal(strip, occupied) {
            grips.push((D::North, piece));
        }
    }
    grips
}

/// The sub-rects of the horizontal `strip` whose x-span is **not** covered by
/// any of `blockers` (blockers only count where they vertically overlap the
/// strip).
///
/// Used to carve the north resize band around the title-bar controls: the
/// blocked x-intervals are merged, then the gaps between them become the north
/// grip pieces.  Pure arithmetic — no egui state — so it is directly testable.
fn split_horizontal(strip: egui::Rect, blockers: &[egui::Rect]) -> Vec<egui::Rect> {
    // Clipped x-intervals of the blockers that overlap the strip vertically.
    let mut blocked: Vec<(f32, f32)> = blockers
        .iter()
        .filter(|b| b.bottom() > strip.top() && b.top() < strip.bottom())
        .map(|b| (b.left().max(strip.left()), b.right().min(strip.right())))
        .filter(|(l, r)| r > l)
        .collect();
    blocked.sort_by(|a, b| a.0.total_cmp(&b.0));

    // Merge overlapping / touching intervals so gaps are disjoint.
    let mut merged: Vec<(f32, f32)> = Vec::new();
    for (l, r) in blocked {
        match merged.last_mut() {
            Some(last) if l <= last.1 => last.1 = last.1.max(r),
            _ => merged.push((l, r)),
        }
    }

    // Emit the gaps left between the merged blockers.
    let mut out = Vec::new();
    let mut cursor = strip.left();
    for (l, r) in merged {
        if l > cursor {
            out.push(egui::Rect::from_min_max(
                egui::pos2(cursor, strip.top()),
                egui::pos2(l, strip.bottom()),
            ));
        }
        cursor = cursor.max(r);
    }
    if cursor < strip.right() {
        out.push(egui::Rect::from_min_max(
            egui::pos2(cursor, strip.top()),
            egui::pos2(strip.right(), strip.bottom()),
        ));
    }
    out
}

/// Hit-test a pointer position against the window's resize grips.
///
/// Returns the [`egui::ResizeDirection`] for the edge / corner zone the point
/// lies in, or `None` for the interior.  A maximized window offers **no**
/// zones at all — it cannot be resized, so the grips are suppressed entirely.
///
/// `occupied` are the title-bar control rects drawn this frame.  A point that
/// falls inside one of them never resizes — the control owns every pixel of
/// its rect, including the shared boundary — which is the exact guarantee that
/// a press on a panel toggle / window button is never hijacked by the north
/// band.  Elsewhere the north edge is tested against [`resize_grips`]; a point
/// on the top edge in an *unoccupied* span yields
/// [`egui::ResizeDirection::North`].
pub fn resize_zone(
    pos: egui::Pos2,
    screen: egui::Rect,
    band: f32,
    maximized: bool,
    occupied: &[egui::Rect],
) -> Option<egui::ResizeDirection> {
    if maximized {
        return None;
    }
    // A title-bar control wins its own pixels outright (belt to the split-zone
    // braces below): a control's boundary pixel is part of the control, not the
    // grip that happens to abut it.
    if occupied.iter().any(|r| r.contains(pos)) {
        return None;
    }
    resize_grips(screen, band, occupied)
        .into_iter()
        .find(|(_, rect)| rect.contains(pos))
        .map(|(dir, _)| dir)
}

/// The pointer shape for a resize zone.
fn cursor_for(dir: egui::ResizeDirection) -> egui::CursorIcon {
    use egui::ResizeDirection as D;
    match dir {
        D::North => egui::CursorIcon::ResizeNorth,
        D::South => egui::CursorIcon::ResizeSouth,
        D::East => egui::CursorIcon::ResizeEast,
        D::West => egui::CursorIcon::ResizeWest,
        D::NorthEast => egui::CursorIcon::ResizeNorthEast,
        D::SouthEast => egui::CursorIcon::ResizeSouthEast,
        D::NorthWest => egui::CursorIcon::ResizeNorthWest,
        D::SouthWest => egui::CursorIcon::ResizeSouthWest,
    }
}

/// Draw the custom resize grips for the undecorated window.
///
/// With `decorations(false)` the OS resize borders are gone, so the app
/// provides its own: eight thin hit zones along the outermost pixels, each
/// starting a native resize via [`egui::ViewportCommand::BeginResize`] when it
/// is pressed.  They are drawn invisibly — only a faint accent highlight
/// appears under the pointer — and are kept to the outermost [`RESIZE_BAND`]
/// pixels so they never swallow clicks meant for the viewport or the panels.
///
/// The rects come from [`resize_grips`], so the north edge is already split
/// around the title-bar controls published by [`show_title_bar`]: no grip
/// widget covers a control, so a control keeps every pixel of its top edge.
/// A maximized window creates no grips at all.
pub fn show_resize_grips(ctx: &egui::Context, dark_mode: bool) {
    if is_maximized(ctx) {
        return;
    }
    let screen = ctx.screen_rect();
    let occupied = control_rects(ctx);
    // The grips (north edge split around the controls) are the single source of
    // truth for both the hit-test and the widgets that own the press.
    let grips = resize_grips(screen, RESIZE_BAND, &occupied);
    let hovered = ctx
        .input(|i| i.pointer.hover_pos())
        .and_then(|pos| resize_zone(pos, screen, RESIZE_BAND, false, &occupied));
    if let Some(dir) = hovered {
        ctx.set_cursor_icon(cursor_for(dir));
    }
    egui::Area::new(egui::Id::new("window_resize_grips"))
        .order(egui::Order::Foreground)
        .fixed_pos(screen.min)
        .movable(false)
        .interactable(true)
        .show(ctx, |ui| {
            for (index, (dir, rect)) in grips.into_iter().enumerate() {
                let response = ui.interact(
                    rect,
                    egui::Id::new(("window_resize_grip", index)),
                    egui::Sense::click(),
                );
                if hovered == Some(dir) && response.hovered() {
                    ui.painter()
                        .rect_filled(rect, 0.0, theme::accent_alpha(dark_mode, 70));
                }
                if response.is_pointer_button_down_on() {
                    ui.ctx()
                        .send_viewport_cmd(egui::ViewportCommand::BeginResize(dir));
                }
            }
        });
}

// ── Native rounded window corners (Change 1) ────────────────────────────

/// Ask Windows 11 to round the (undecorated) window's frame.
///
/// `DwmSetWindowAttribute(hwnd, DWMWA_WINDOW_CORNER_PREFERENCE, DWMWCP_ROUND)`
/// is the native route: DWM rounds the frame of a **borderless** window too,
/// and — unlike the `with_transparent(true)` egui fallback — it needs no
/// transparency, so the window keeps its opaque, snapped content and there are
/// no black corners to composite.
///
/// The HWND is looked up by the app's own window title (`FindWindowW`), which
/// is already set when eframe calls the app creator; if that lookup misses
/// (e.g. the title has not been applied yet) it falls back to the foreground
/// window.  DWM's corner rounding predates the process being the foreground
/// app, so either handle is the right window.
///
/// Returns `Ok(())` once DWM accepted the preference.  On Windows 10 the
/// attribute is unknown and DWM returns `E_INVALIDARG`; on a missing window the
/// lookup yields a null handle.  Both are reported to the caller as an `Err`
/// string — the window then simply keeps its square corners (the app never
/// depends on the rounding).
#[cfg(windows)]
pub fn apply_rounded_corners(window_title: &str) -> Result<(), String> {
    use windows_sys::Win32::Foundation::HWND;
    use windows_sys::Win32::Graphics::Dwm::{
        DWMWA_WINDOW_CORNER_PREFERENCE, DWMWCP_ROUND, DwmSetWindowAttribute,
    };
    use windows_sys::Win32::UI::WindowsAndMessaging::{FindWindowW, GetForegroundWindow};

    /// Locate the app window: exact title match first, foreground window second.
    fn find_hwnd(title: &str) -> HWND {
        let wide: Vec<u16> = title.encode_utf16().chain(std::iter::once(0)).collect();
        // Safety: `wide` is a NUL-terminated UTF-16 string; a null class name
        // asks FindWindowW to match on the title alone.
        let by_title = unsafe { FindWindowW(std::ptr::null(), wide.as_ptr()) };
        if !by_title.is_null() {
            return by_title;
        }
        // Safety: returns the foreground window or a null handle; both are fine.
        unsafe { GetForegroundWindow() }
    }

    let hwnd = find_hwnd(window_title);
    if hwnd.is_null() {
        return Err("could not locate the application window".to_string());
    }

    // `DWMWCP_ROUND` is DWM's "round all corners" preference (2).
    let preference: i32 = DWMWCP_ROUND;
    // Safety: `preference` is a live `i32` whose size matches `cbattribute`.
    let hr = unsafe {
        DwmSetWindowAttribute(
            hwnd,
            DWMWA_WINDOW_CORNER_PREFERENCE as u32,
            (&preference as *const i32).cast(),
            std::mem::size_of::<i32>() as u32,
        )
    };
    if hr < 0 {
        return Err(format!(
            "DwmSetWindowAttribute(DWMWA_WINDOW_CORNER_PREFERENCE) failed (HRESULT 0x{:08X})",
            hr as u32
        ));
    }
    Ok(())
}

/// Non-Windows stub: rounded corners are a Windows 11 (DWM) feature only.
#[cfg(not(windows))]
pub fn apply_rounded_corners(_window_title: &str) -> Result<(), String> {
    Err("rounded window corners are only implemented on Windows".to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn screen() -> egui::Rect {
        egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(800.0, 600.0))
    }

    /// A title-bar control rect: a 26×24 button starting at the window's top
    /// edge over the horizontal span `left..right` (the toggles / window
    /// buttons all begin at the very top of the bar).
    fn control(left: f32, right: f32) -> egui::Rect {
        egui::Rect::from_min_max(egui::pos2(left, 0.0), egui::pos2(right, 24.0))
    }

    /// Every corner and edge of the window maps to the matching resize
    /// direction, and the interior maps to nothing.
    #[test]
    fn resize_zone_maps_corners_edges_and_interior() {
        let s = screen();
        let b = RESIZE_BAND;
        use egui::ResizeDirection as D;

        // Corners.
        assert_eq!(
            resize_zone(egui::pos2(2.0, 2.0), s, b, false, &[]),
            Some(D::NorthWest)
        );
        assert_eq!(
            resize_zone(egui::pos2(798.0, 2.0), s, b, false, &[]),
            Some(D::NorthEast)
        );
        assert_eq!(
            resize_zone(egui::pos2(2.0, 598.0), s, b, false, &[]),
            Some(D::SouthWest)
        );
        assert_eq!(
            resize_zone(egui::pos2(798.0, 598.0), s, b, false, &[]),
            Some(D::SouthEast)
        );

        // Edge mid-spans.
        assert_eq!(
            resize_zone(egui::pos2(400.0, 2.0), s, b, false, &[]),
            Some(D::North)
        );
        assert_eq!(
            resize_zone(egui::pos2(400.0, 598.0), s, b, false, &[]),
            Some(D::South)
        );
        assert_eq!(
            resize_zone(egui::pos2(2.0, 300.0), s, b, false, &[]),
            Some(D::West)
        );
        assert_eq!(
            resize_zone(egui::pos2(798.0, 300.0), s, b, false, &[]),
            Some(D::East)
        );

        // Just inside the band on each edge is *not* a grip, and neither is the
        // interior — so normal clicks never become resizes.
        assert_eq!(resize_zone(egui::pos2(400.0, 6.0), s, b, false, &[]), None);
        assert_eq!(resize_zone(egui::pos2(6.0, 300.0), s, b, false, &[]), None);
        assert_eq!(
            resize_zone(egui::pos2(400.0, 300.0), s, b, false, &[]),
            None
        );
        assert_eq!(resize_zone(egui::pos2(20.0, 300.0), s, b, false, &[]), None);
    }

    /// A press inside a title-bar control on the top edge is *not* a resize:
    /// the north band is carved around the control, so the control keeps every
    /// one of its top pixels — while the empty spans of the bar still resize.
    #[test]
    fn north_grip_yields_to_title_bar_controls() {
        let s = screen();
        let b = RESIZE_BAND;
        use egui::ResizeDirection as D;
        // A window-control-sized button over x 700..760 at the top of the bar.
        let occ = [control(700.0, 760.0)];

        // Inside the control's span, within the north band → no resize.
        assert_eq!(resize_zone(egui::pos2(730.0, 2.0), s, b, false, &occ), None);
        // Every pixel of the control's span yields, including its edges.
        assert_eq!(resize_zone(egui::pos2(700.0, 1.0), s, b, false, &occ), None);
        assert_eq!(resize_zone(egui::pos2(760.0, 4.0), s, b, false, &occ), None);

        // The *unoccupied* stretches of the top edge still resize.
        assert_eq!(
            resize_zone(egui::pos2(400.0, 2.0), s, b, false, &occ),
            Some(D::North)
        );
        assert_eq!(
            resize_zone(egui::pos2(690.0, 2.0), s, b, false, &occ),
            Some(D::North)
        );
        assert_eq!(
            resize_zone(egui::pos2(770.0, 2.0), s, b, false, &occ),
            Some(D::North)
        );

        // The context corners are away from the control and stay resizable.
        assert_eq!(
            resize_zone(egui::pos2(2.0, 2.0), s, b, false, &occ),
            Some(D::NorthWest)
        );
        assert_eq!(
            resize_zone(egui::pos2(798.0, 2.0), s, b, false, &occ),
            Some(D::NorthEast)
        );
    }

    /// Several controls (three panel toggles + three window buttons) all yield
    /// to the north grip, and the gaps *between* them still resize.
    #[test]
    fn north_grip_yields_to_each_control_and_keeps_the_gaps() {
        let s = screen();
        let b = RESIZE_BAND;
        use egui::ResizeDirection as D;
        let occ = [
            control(600.0, 626.0),
            control(632.0, 658.0),
            control(664.0, 690.0),
            control(700.0, 726.0),
            control(728.0, 754.0),
            control(756.0, 782.0),
        ];
        // A press over any control yields.
        for x in [610.0, 645.0, 677.0, 713.0, 741.0, 769.0] {
            assert_eq!(
                resize_zone(egui::pos2(x, 2.0), s, b, false, &occ),
                None,
                "x={x}"
            );
        }
        // The narrow gaps between the buttons, and the wide empty left side,
        // still resize.
        assert_eq!(
            resize_zone(egui::pos2(629.0, 2.0), s, b, false, &occ),
            Some(D::North)
        );
        assert_eq!(
            resize_zone(egui::pos2(100.0, 2.0), s, b, false, &occ),
            Some(D::North)
        );
        assert_eq!(
            resize_zone(egui::pos2(790.0, 2.0), s, b, false, &occ),
            Some(D::North)
        );
    }

    /// Only the north edge consults `occupied`; the other seven zones map
    /// exactly as before even when a control rect sits near their edge.
    #[test]
    fn non_north_zones_ignore_control_rects() {
        let s = screen();
        let b = RESIZE_BAND;
        use egui::ResizeDirection as D;
        // A control spanning the top-right corner: the East edge below it must
        // still be a grip.
        let occ = [control(780.0, 800.0)];
        assert_eq!(
            resize_zone(egui::pos2(798.0, 300.0), s, b, false, &occ),
            Some(D::East)
        );
        assert_eq!(
            resize_zone(egui::pos2(400.0, 598.0), s, b, false, &occ),
            Some(D::South)
        );
        assert_eq!(
            resize_zone(egui::pos2(2.0, 300.0), s, b, false, &occ),
            Some(D::West)
        );
        assert_eq!(
            resize_zone(egui::pos2(2.0, 598.0), s, b, false, &occ),
            Some(D::SouthWest)
        );
        assert_eq!(
            resize_zone(egui::pos2(798.0, 598.0), s, b, false, &occ),
            Some(D::SouthEast)
        );
        assert_eq!(
            resize_zone(egui::pos2(2.0, 2.0), s, b, false, &occ),
            Some(D::NorthWest)
        );
    }

    /// A maximized window offers no resize zones anywhere — not even at the
    /// corners or edges, and regardless of any control rects.
    #[test]
    fn resize_zone_is_empty_when_maximized() {
        let s = screen();
        let b = RESIZE_BAND;
        let occ = [control(700.0, 760.0)];
        for pos in [
            egui::pos2(2.0, 2.0),
            egui::pos2(798.0, 2.0),
            egui::pos2(2.0, 598.0),
            egui::pos2(798.0, 598.0),
            egui::pos2(400.0, 2.0),
            egui::pos2(730.0, 2.0),
            egui::pos2(400.0, 598.0),
            egui::pos2(2.0, 300.0),
            egui::pos2(798.0, 300.0),
            egui::pos2(400.0, 300.0),
        ] {
            assert_eq!(
                resize_zone(pos, s, b, true, &occ),
                None,
                "maximized at {pos:?}"
            );
        }
    }

    /// `resize_grips` splits the north band around the controls, keeps every
    /// north piece inside the band and clear of the controls, and leaves the
    /// other seven directions intact.
    #[test]
    fn resize_grips_split_north_around_controls() {
        use egui::ResizeDirection as D;
        let s = screen();
        let occ = [control(700.0, 760.0)];
        let grips = resize_grips(s, RESIZE_BAND, &occ);

        let north: Vec<egui::Rect> = grips
            .iter()
            .filter(|(d, _)| *d == D::North)
            .map(|(_, r)| *r)
            .collect();
        assert!(north.len() >= 2, "the north band is split by the control");
        for r in &north {
            assert!(
                (r.top() - s.top()).abs() < f32::EPSILON,
                "north piece off-band"
            );
            assert!(
                (r.height() - RESIZE_BAND).abs() < f32::EPSILON,
                "north piece thickness"
            );
            assert!(
                r.right() <= 700.0 + f32::EPSILON || r.left() >= 760.0 - f32::EPSILON,
                "north piece overlaps the control: {r:?}"
            );
        }
        // The seven non-north directions survive exactly once.
        for want in [
            D::South,
            D::East,
            D::West,
            D::NorthEast,
            D::NorthWest,
            D::SouthEast,
            D::SouthWest,
        ] {
            assert_eq!(
                grips.iter().filter(|(d, _)| *d == want).count(),
                1,
                "{want:?} missing / duplicated"
            );
        }
    }

    /// With no controls the north band is a single full-width strip, i.e. the
    /// split is a no-op outside the title-bar row.
    #[test]
    fn resize_grips_without_controls_keep_one_north_strip() {
        use egui::ResizeDirection as D;
        let s = screen();
        let north: Vec<egui::Rect> = resize_grips(s, RESIZE_BAND, &[])
            .into_iter()
            .filter(|(d, _)| *d == D::North)
            .map(|(_, r)| r)
            .collect();
        assert_eq!(north.len(), 1);
        assert_eq!(north[0], resize_zones(s, RESIZE_BAND)[4].1);
    }

    /// The zone rects cover all eight directions exactly once and stay inside
    /// the screen.
    #[test]
    fn resize_zones_describe_all_eight_directions() {
        use egui::ResizeDirection as D;
        let s = screen();
        let zones = resize_zones(s, RESIZE_BAND);
        assert_eq!(zones.len(), 8);

        let dirs: Vec<D> = zones.iter().map(|(d, _)| *d).collect();
        for want in [
            D::North,
            D::South,
            D::East,
            D::West,
            D::NorthEast,
            D::NorthWest,
            D::SouthEast,
            D::SouthWest,
        ] {
            assert_eq!(dirs.iter().filter(|d| **d == want).count(), 1, "{want:?}");
        }

        // Every zone lies within the screen and is non-empty.
        for (dir, rect) in zones {
            assert!(!rect.is_negative(), "{dir:?} is empty/inverted");
            assert!(s.contains_rect(rect), "{dir:?} escapes the screen");
        }
    }

    /// The maximize control reflects the window state: restore while
    /// maximized, the maximize square otherwise.
    #[test]
    fn maximize_icon_reflects_window_state() {
        assert_eq!(
            maximize_icon(false),
            icons::window_maximize as icons::IconFn
        );
        assert_eq!(maximize_icon(true), icons::window_restore as icons::IconFn);
        assert_ne!(maximize_icon(false), maximize_icon(true));
    }

    /// Each zone maps to the cursor of its direction.
    #[test]
    fn cursor_matches_each_direction() {
        use egui::ResizeDirection as D;
        assert_eq!(cursor_for(D::North), egui::CursorIcon::ResizeNorth);
        assert_eq!(cursor_for(D::South), egui::CursorIcon::ResizeSouth);
        assert_eq!(cursor_for(D::East), egui::CursorIcon::ResizeEast);
        assert_eq!(cursor_for(D::West), egui::CursorIcon::ResizeWest);
        assert_eq!(cursor_for(D::NorthEast), egui::CursorIcon::ResizeNorthEast);
        assert_eq!(cursor_for(D::NorthWest), egui::CursorIcon::ResizeNorthWest);
        assert_eq!(cursor_for(D::SouthEast), egui::CursorIcon::ResizeSouthEast);
        assert_eq!(cursor_for(D::SouthWest), egui::CursorIcon::ResizeSouthWest);
    }
}
