//! Command Palette (Phase 159-A) — opened with **Ctrl+Shift+P**.
//!
//! A modal overlay with a single search field: fuzzy/substring matching over
//! each [`Command`]'s label + category + id (see [`commands::search`]), results
//! ordered by match quality.
//!
//! # Keyboard model
//!
//! * `\u{2191}` / `\u{2193}` (with or without Ctrl) — move the highlight
//! * `Tab` / `Shift+Tab` — next / previous result
//! * `Enter` — execute the highlighted command and close the palette
//! * `Esc` — close without running anything
//! * Mouse: hover highlights, click executes (enabled rows only)
//!
//! The search field keeps egui's widget focus for as long as the palette is
//! open (a focus-lock [`EventFilter`](egui::EventFilter) stops Tab / arrows /
//! Escape from moving focus away), which also suppresses the app's global
//! shortcuts — the palette is fully modal.
//!
//! Disabled commands are **listed**, dimmed and suffixed `(unavailable)`, with
//! the reason from
//! [`command_disabled_reason`](ViewportApp::command_disabled_reason) on hover;
//! they cannot be executed.

use egui;

use super::commands::{self, Command};
use super::theme;
use crate::ViewportApp;

/// Row height of one result (points).
const ROW_HEIGHT: f32 = 24.0;
/// Maximum number of results shown before scrolling.
const MAX_VISIBLE_ROWS: usize = 10;
/// Height of the search field row (points).
const SEARCH_HEIGHT: f32 = 30.0;
/// Distance of the palette box from the top of the window (points).
const TOP_OFFSET: f32 = 96.0;

/// Palette UI state — kept in memory only (never persisted).
#[derive(Debug, Clone, Default)]
pub struct PaletteState {
    /// Whether the palette is open.
    open: bool,
    /// Current search query.
    query: String,
    /// Index of the highlighted result (into `commands::search(&query)`).
    selected: usize,
    /// Focus the search field on the next drawn frame (set when opening).
    focus_pending: bool,
    /// Id of the search widget from the last drawn frame (for focus release).
    search_id: Option<egui::Id>,
}

impl PaletteState {
    /// Whether the palette is currently open.
    pub fn is_open(&self) -> bool {
        self.open
    }
}

/// Open the palette, or close it when already open (resets query + highlight).
pub fn toggle(app: &mut ViewportApp) {
    let opening = !app.palette.is_open();
    app.palette.open = opening;
    app.palette.query.clear();
    app.palette.selected = 0;
    app.palette.focus_pending = opening;
}

/// Close the palette and release the search field's focus.
fn close(app: &mut ViewportApp, ctx: &egui::Context) {
    let search_id = app.palette.search_id.take();
    app.palette.open = false;
    app.palette.query.clear();
    app.palette.selected = 0;
    app.palette.focus_pending = false;
    if let Some(id) = search_id {
        ctx.memory_mut(|m| m.surrender_focus(id));
    }
}

/// Move the highlight by `delta`, wrapping around the result list.
fn move_selection(app: &mut ViewportApp, len: usize, delta: isize) {
    if len == 0 {
        app.palette.selected = 0;
        return;
    }
    let next = (app.palette.selected as isize + delta).rem_euclid(len as isize);
    app.palette.selected = next as usize;
}

/// Draw the palette for one frame.  No-op (and zero cost) when closed.
pub fn show(app: &mut ViewportApp, ctx: &egui::Context) {
    if !app.palette.open {
        return;
    }

    // ── Ranked results for the current query ───────────────────────────
    let results = commands::search(&app.palette.query);
    if app.palette.selected >= results.len() {
        app.palette.selected = results.len().saturating_sub(1);
    }

    // ── Keyboard (handled before drawing so the frame shows the move) ───
    let mut close_requested = false;
    let mut enter_command: Option<Command> = None;
    ctx.input(|i| {
        if i.key_pressed(egui::Key::Escape) {
            close_requested = true;
        }
        if i.key_pressed(egui::Key::ArrowDown) {
            move_selection(app, results.len(), 1);
        }
        if i.key_pressed(egui::Key::ArrowUp) {
            move_selection(app, results.len(), -1);
        }
        if i.key_pressed(egui::Key::Tab) {
            move_selection(app, results.len(), if i.modifiers.shift { -1 } else { 1 });
        }
        if i.key_pressed(egui::Key::Enter) {
            enter_command = results.get(app.palette.selected).copied();
        }
    });

    // ── Layout: one full-screen layer holding the dim + the palette box ─
    let screen = ctx.screen_rect();
    let width = (screen.width() * 0.5).clamp(420.0, 640.0);
    let rows = results.len().min(MAX_VISIBLE_ROWS);
    let list_h = rows as f32 * ROW_HEIGHT + 8.0;
    let box_h = SEARCH_HEIGHT + list_h + 16.0;
    let box_rect = egui::Rect::from_min_size(
        egui::pos2(
            screen.center().x - width * 0.5,
            (screen.min.y + TOP_OFFSET).min(screen.max.y - box_h),
        ),
        egui::vec2(width, box_h),
    );

    let mut search_id: Option<egui::Id> = None;
    let mut row_clicked: Option<Command> = None;
    let area = egui::Area::new("command_palette".into())
        .order(egui::Order::Foreground)
        .fixed_pos(screen.min)
        .show(ctx, |ui| {
            // Dim the whole window; a click on the dim area closes the palette.
            let backdrop = ui.allocate_response(screen.size(), egui::Sense::click());
            ui.painter()
                .rect_filled(screen, 0.0, theme::scrim(app.dark_mode));

            ui.scope_builder(egui::UiBuilder::new().max_rect(box_rect), |ui| {
                egui::Frame::window(ui.style()).show(ui, |ui| {
                    ui.set_width(width);
                    // ── Search field ──
                    let resp = ui.add(
                        egui::TextEdit::singleline(&mut app.palette.query)
                            .hint_text("Type a command\u{2026}")
                            .frame(false),
                    );
                    if app.palette.focus_pending {
                        resp.request_focus();
                        app.palette.focus_pending = false;
                    }
                    search_id = Some(resp.id);
                    // Modal focus: Tab / arrows / Escape act on the palette
                    // instead of moving egui's widget focus away.
                    let filter = egui::EventFilter {
                        tab: true,
                        horizontal_arrows: false,
                        vertical_arrows: true,
                        escape: true,
                    };
                    ctx.memory_mut(|m| m.set_focus_lock_filter(resp.id, filter));
                    ui.separator();
                    // ── Results ──
                    egui::ScrollArea::vertical()
                        .max_height(list_h)
                        .show(ui, |ui| {
                            ui.set_width(width);
                            if results.is_empty() {
                                ui.label(egui::RichText::new("No matching commands").weak());
                            }
                            for (i, &cmd) in results.iter().enumerate() {
                                if let Some(clicked) = row(ui, app, i, cmd) {
                                    row_clicked = Some(clicked);
                                }
                            }
                        });
                });
            });
            backdrop
        });
    app.palette.search_id = search_id;

    // A click on the dimmed area outside the palette box closes it (clicks on
    // the box itself are handled by its widgets and never reach the backdrop).
    let outside_click = area.inner.clicked()
        && ctx
            .input(|i| i.pointer.interact_pos())
            .is_none_or(|p| !box_rect.contains(p));

    // Enter (or a row click) executes the highlighted command — only when it
    // is enabled; disabled rows are inert by design.
    let mut did_run = false;
    if let Some(cmd) = enter_command.or(row_clicked) {
        if app.command_is_enabled(cmd) {
            app.execute_command(cmd, ctx);
            did_run = true;
        }
    }

    if close_requested || did_run || outside_click {
        close(app, ctx);
    }
}

/// Draw one result row; returns the command when it was clicked (enabled rows
/// only — disabled rows are inert).
fn row(ui: &mut egui::Ui, app: &mut ViewportApp, index: usize, cmd: Command) -> Option<Command> {
    let enabled = app.command_is_enabled(cmd);
    let reason = app.command_disabled_reason(cmd);
    let selected = index == app.palette.selected;

    let (rect, mut resp) = ui.allocate_exact_size(
        egui::vec2(ui.available_width(), ROW_HEIGHT),
        egui::Sense::click(),
    );
    if resp.hovered() {
        app.palette.selected = index;
    }

    // Highlight the keyboard selection, or the hovered row.
    let visuals = ui.visuals().clone();
    let bg = if selected {
        visuals.selection.bg_fill
    } else if resp.hovered() {
        visuals.widgets.hovered.bg_fill
    } else {
        egui::Color32::TRANSPARENT
    };
    if bg != egui::Color32::TRANSPARENT {
        ui.painter()
            .rect_filled(rect, egui::Rounding::same(4.0), bg);
    }

    // Label (dimmed + marked when unavailable) with the category · shortcut
    // right-aligned.
    let label_color = if enabled {
        visuals.text_color()
    } else {
        visuals.weak_text_color()
    };
    let meta_color = visuals.weak_text_color().gamma_multiply(0.8);
    let inner = rect.shrink2(egui::vec2(8.0, 0.0));
    let mut child = ui.new_child(
        egui::UiBuilder::new()
            .max_rect(inner)
            .layout(egui::Layout::left_to_right(egui::Align::Center)),
    );
    let mut text = cmd.label();
    if !enabled {
        text.push_str("  (unavailable)");
    }
    child.label(egui::RichText::new(text).color(label_color));
    child.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
        let mut meta = cmd.category().label().to_string();
        if let Some(shortcut) = cmd.shortcut() {
            meta.push_str(" \u{b7} ");
            meta.push_str(shortcut);
        }
        ui.label(theme::small(meta).color(meta_color));
    });

    // Explanation for disabled rows; a compact hint for enabled ones.
    resp = match reason {
        Some(reason) => resp.on_hover_text(format!("Unavailable \u{2014} {reason}")),
        None => resp.on_hover_text(format!("{} \u{b7} {}", cmd.category().label(), cmd.id())),
    };

    if resp.clicked() && enabled {
        return Some(cmd);
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use viewport_poc::display::ResultViewMode;
    use viewport_poc::edit_model::EditModel;
    use viewport_poc::persistence;

    /// An app with no model (for the disabled-command paths).
    fn empty_app() -> ViewportApp {
        ViewportApp::new(
            persistence::AppSettings::default(),
            std::path::PathBuf::from("unused-settings.json"),
            None,
        )
    }

    /// An app holding one 4 m beam with a live scene.
    fn beam_app() -> ViewportApp {
        let mut app = empty_app();
        let mut m = EditModel::new();
        m.add_node(0.0, 0.0, 0.0).unwrap();
        m.add_node(4.0, 0.0, 0.0).unwrap();
        app.project.load_model("test".to_string(), m);
        app.rebuild_scene();
        app
    }

    fn key(key: egui::Key, modifiers: egui::Modifiers) -> egui::Event {
        egui::Event::Key {
            key,
            physical_key: Some(key),
            pressed: true,
            repeat: false,
            modifiers,
        }
    }

    fn text(s: &str) -> egui::Event {
        egui::Event::Text(s.to_owned())
    }

    /// Run one headless frame of the palette with the given input events.
    fn frame(app: &mut ViewportApp, ctx: &egui::Context, events: Vec<egui::Event>) {
        frame_mod(app, ctx, events, egui::Modifiers::default())
    }

    /// Like [`frame`], with an explicit integration-level modifier state
    /// (egui derives `i.modifiers` from `RawInput.modifiers`, exactly like
    /// the real windowing integration does).
    fn frame_mod(
        app: &mut ViewportApp,
        ctx: &egui::Context,
        events: Vec<egui::Event>,
        modifiers: egui::Modifiers,
    ) {
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::pos2(0.0, 0.0),
                egui::vec2(1200.0, 800.0),
            )),
            events,
            modifiers,
            ..Default::default()
        };
        let _ = ctx.run(input, |ctx| {
            show(app, ctx);
        });
    }

    #[test]
    fn palette_opens_via_command_and_draws_headlessly() {
        let mut app = beam_app();
        let ctx = egui::Context::default();
        app.execute_command(Command::TogglePalette, &ctx);
        assert!(app.palette.is_open());
        // Two frames: the first requests focus, the second exercises the
        // focus-lock filter path.  Must not panic.
        frame(&mut app, &ctx, vec![]);
        frame(&mut app, &ctx, vec![]);
        assert!(app.palette.is_open());
    }

    #[test]
    fn palette_typing_narrows_results_and_enter_executes() {
        let mut app = beam_app();
        let ctx = egui::Context::default();
        app.execute_command(Command::TogglePalette, &ctx);
        frame(&mut app, &ctx, vec![]);
        // Type "tog": the ranked list starts with "Toggle Cut-Face Cap" and
        // "Toggle Deformed Shape" (label-prefix matches, then label order).
        frame(&mut app, &ctx, vec![text("tog")]);
        frame(&mut app, &ctx, vec![]);
        let results = commands::search("tog");
        assert_eq!(results[0].label(), "Toggle Cut-Face Cap");
        assert_eq!(results[1].label(), "Toggle Deformed Shape");
        assert_eq!(app.palette.selected, 0);
        // Down moves the highlight; Enter executes it and closes the palette.
        frame(
            &mut app,
            &ctx,
            vec![key(egui::Key::ArrowDown, egui::Modifiers::default())],
        );
        assert_eq!(app.palette.selected, 1);
        frame(
            &mut app,
            &ctx,
            vec![key(egui::Key::Enter, egui::Modifiers::default())],
        );
        assert!(!app.palette.is_open(), "executing closes the palette");
        assert_eq!(
            app.display.mode,
            ResultViewMode::Deformed,
            "the highlighted command actually ran"
        );
    }

    #[test]
    fn palette_tab_and_shift_tab_move_the_highlight() {
        let mut app = beam_app();
        let ctx = egui::Context::default();
        app.execute_command(Command::TogglePalette, &ctx);
        frame(&mut app, &ctx, vec![]);
        frame(&mut app, &ctx, vec![text("tog")]);
        frame(&mut app, &ctx, vec![]);
        frame(
            &mut app,
            &ctx,
            vec![key(egui::Key::Tab, egui::Modifiers::default())],
        );
        assert_eq!(app.palette.selected, 1);
        frame_mod(
            &mut app,
            &ctx,
            vec![key(egui::Key::Tab, egui::Modifiers::SHIFT)],
            egui::Modifiers::SHIFT,
        );
        assert_eq!(app.palette.selected, 0);
        assert!(app.palette.is_open(), "navigation never executes");
    }

    #[test]
    fn palette_escape_closes_without_running() {
        let mut app = beam_app();
        let ctx = egui::Context::default();
        app.execute_command(Command::TogglePalette, &ctx);
        frame(&mut app, &ctx, vec![]);
        let mode = app.display.mode;
        frame(
            &mut app,
            &ctx,
            vec![key(egui::Key::Escape, egui::Modifiers::default())],
        );
        assert!(!app.palette.is_open());
        assert_eq!(app.display.mode, mode, "nothing was executed");
    }

    #[test]
    fn palette_outside_click_closes() {
        let mut app = beam_app();
        let ctx = egui::Context::default();
        app.execute_command(Command::TogglePalette, &ctx);
        // Two warm-up frames: the first opens (and focuses), the second makes
        // the palette's widget rects available for hit-testing the click.
        frame(&mut app, &ctx, vec![]);
        frame(&mut app, &ctx, vec![]);
        // A click near the top-left corner is on the dimmed backdrop, well
        // outside the centred palette box.
        let pos = egui::pos2(10.0, 10.0);
        frame(
            &mut app,
            &ctx,
            vec![
                egui::Event::PointerMoved(pos),
                egui::Event::PointerButton {
                    pos,
                    button: egui::PointerButton::Primary,
                    pressed: true,
                    modifiers: egui::Modifiers::default(),
                },
                egui::Event::PointerButton {
                    pos,
                    button: egui::PointerButton::Primary,
                    pressed: false,
                    modifiers: egui::Modifiers::default(),
                },
            ],
        );
        assert!(!app.palette.is_open());
    }

    #[test]
    fn palette_lists_disabled_commands_but_refuses_to_run_them() {
        let mut app = empty_app();
        let ctx = egui::Context::default();
        app.execute_command(Command::TogglePalette, &ctx);
        frame(&mut app, &ctx, vec![]);
        frame(&mut app, &ctx, vec![text("run analysis")]);
        frame(&mut app, &ctx, vec![]);
        // "Run Analysis" is listed (no model loaded) …
        assert_eq!(commands::search("run analysis")[0], Command::RunAnalysis);
        assert!(!app.command_is_enabled(Command::RunAnalysis));
        // … but Enter on it is refused: the palette stays open, nothing runs.
        frame(
            &mut app,
            &ctx,
            vec![key(egui::Key::Enter, egui::Modifiers::default())],
        );
        assert!(app.palette.is_open(), "disabled commands cannot execute");
        assert!(app.project.model.is_none());
        assert!(app.analysis_result().is_none());
    }

    #[test]
    fn palette_search_always_lists_disabled_commands() {
        // An empty query lists the whole registry, disabled entries included.
        let all = commands::search("");
        assert!(all.contains(&Command::Undo));
        assert!(all.contains(&Command::RunAnalysis));
        assert!(all.contains(&Command::ShowAll));
        assert_eq!(all.len(), Command::ALL.len());
    }
}
