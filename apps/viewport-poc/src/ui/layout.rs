//! Layout preset capture / restore (Phase P7-3, C2).
//!
//! The persisted [`LayoutPrefs`](viewport_poc::persistence::LayoutPrefs)
//! (panel widths, dock height, collapsed flags) live in the settings file;
//! this module is the runtime bridge between the panels and that file:
//!
//! * the resizable panels publish their live size here every frame
//!   ([`publish_tree_width`], [`publish_inspector_width`],
//!   [`publish_dock_height`]) and read the restored size back when computing
//!   their default width / height ([`restored_tree_width`] etc.);
//! * the application loop composes the current layout with [`capture`],
//!   compares it against [`last_saved`] and persists it on change / exit.
//!
//! The transport is egui's own data map (`Context::data`), keyed under
//! `"layout/…"`, so no extra state has to be threaded through
//! [`ViewportApp`](crate::ViewportApp).

use viewport_poc::persistence::LayoutPrefs;

/// egui data key holding the layout restored from disk.
const KEY_RESTORED: &str = "layout/restored";
/// egui data key holding the layout last written to disk.
const KEY_LAST_SAVED: &str = "layout/last_saved";
/// egui data keys holding the live panel sizes published each frame.
const KEY_TREE_W: &str = "layout/tree_w";
const KEY_INSPECTOR_W: &str = "layout/inspector_w";
const KEY_DOCK_H: &str = "layout/dock_h";

/// Store the layout restored from disk so the panels can size themselves
/// from it on the first frames.
pub fn install_restored(ctx: &egui::Context, layout: LayoutPrefs) {
    ctx.data_mut(|d| {
        d.insert_persisted(egui::Id::new(KEY_RESTORED), layout);
        // Seed the "last saved" snapshot so an unchanged layout produces no
        // write on startup.
        d.insert_persisted(egui::Id::new(KEY_LAST_SAVED), layout);
    });
}

/// Publish the live width of the left model-tree panel (called every frame).
pub fn publish_tree_width(ctx: &egui::Context, width: f32) {
    ctx.data_mut(|d| d.insert_persisted(egui::Id::new(KEY_TREE_W), width));
}

/// Publish the live width of the right inspector panel (called every frame).
pub fn publish_inspector_width(ctx: &egui::Context, width: f32) {
    ctx.data_mut(|d| d.insert_persisted(egui::Id::new(KEY_INSPECTOR_W), width));
}

/// Publish the live height of the bottom Messages / Results dock.
pub fn publish_dock_height(ctx: &egui::Context, height: f32) {
    ctx.data_mut(|d| d.insert_persisted(egui::Id::new(KEY_DOCK_H), height));
}

/// Restored width of the model-tree panel, if one was persisted.
pub fn restored_tree_width(ctx: &egui::Context) -> Option<f32> {
    restored(ctx).and_then(|l| l.tree_width)
}

/// Restored width of the inspector panel, if one was persisted.
pub fn restored_inspector_width(ctx: &egui::Context) -> Option<f32> {
    restored(ctx).and_then(|l| l.inspector_width)
}

/// Restored height of the bottom dock, if one was persisted.
pub fn restored_dock_height(ctx: &egui::Context) -> Option<f32> {
    restored(ctx).and_then(|l| l.dock_height)
}

/// Compose the current layout: the live panel sizes published this frame
/// (falling back to the restored values for a collapsed panel, which
/// publishes nothing), plus the live collapsed flags.
pub fn capture(
    ctx: &egui::Context,
    inspector_collapsed: bool,
    dock_collapsed: bool,
) -> LayoutPrefs {
    let restored = restored(ctx);
    let published = |key: &str| ctx.data_mut(|d| d.get_persisted::<f32>(egui::Id::new(key)));
    LayoutPrefs {
        tree_width: published(KEY_TREE_W)
            .or_else(|| restored.and_then(|l| l.tree_width)),
        inspector_width: published(KEY_INSPECTOR_W)
            .or_else(|| restored.and_then(|l| l.inspector_width)),
        dock_height: published(KEY_DOCK_H).or_else(|| restored.and_then(|l| l.dock_height)),
        inspector_collapsed,
        dock_collapsed,
    }
}

/// The layout last written to disk, if any.
pub fn last_saved(ctx: &egui::Context) -> Option<LayoutPrefs> {
    ctx.data_mut(|d| d.get_persisted(egui::Id::new(KEY_LAST_SAVED)))
}

/// Record `layout` as the last one written to disk.
pub fn set_last_saved(ctx: &egui::Context, layout: LayoutPrefs) {
    ctx.data_mut(|d| d.insert_persisted(egui::Id::new(KEY_LAST_SAVED), layout));
}

/// The layout restored from disk, if any.
fn restored(ctx: &egui::Context) -> Option<LayoutPrefs> {
    ctx.data_mut(|d| d.get_persisted(egui::Id::new(KEY_RESTORED)))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The panel geometry must survive a publish → capture round-trip through
    /// egui's data map, and the collapsed flags must flow through untouched.
    #[test]
    fn capture_reflects_published_sizes_and_flags() {
        egui::__run_test_ctx(|ctx| {
            install_restored(ctx, LayoutPrefs::default());
            assert_eq!(
                capture(ctx, false, false),
                LayoutPrefs::default(),
                "nothing published yet → capture equals the restored (empty) preset"
            );

            publish_tree_width(ctx, 240.0);
            publish_inspector_width(ctx, 300.0);
            publish_dock_height(ctx, 130.0);
            let captured = capture(ctx, true, false);
            assert_eq!(captured.tree_width, Some(240.0));
            assert_eq!(captured.inspector_width, Some(300.0));
            assert_eq!(captured.dock_height, Some(130.0));
            assert!(captured.inspector_collapsed);
            assert!(!captured.dock_collapsed);

            set_last_saved(ctx, captured);
            assert_eq!(last_saved(ctx), Some(captured));
        });
    }

    /// A collapsed panel publishes nothing, so `capture` must fall back to
    /// the restored value instead of dropping the size.
    #[test]
    fn capture_falls_back_to_restored_when_panel_hidden() {
        egui::__run_test_ctx(|ctx| {
            let restored_layout = LayoutPrefs {
                tree_width: Some(220.0),
                inspector_width: Some(310.0),
                dock_height: Some(120.0),
                inspector_collapsed: true,
                dock_collapsed: true,
            };
            install_restored(ctx, restored_layout);

            // Only the tree publishes (the inspector / dock are collapsed).
            publish_tree_width(ctx, 260.0);
            let captured = capture(ctx, true, true);
            assert_eq!(captured.tree_width, Some(260.0), "live value wins");
            assert_eq!(
                captured.inspector_width,
                Some(310.0),
                "collapsed panel keeps its restored width"
            );
            assert_eq!(captured.dock_height, Some(120.0));
        });
    }

    /// The restored sizes must be readable individually — this is what the
    /// panels call to size themselves.
    #[test]
    fn restored_accessors_read_the_installed_preset() {
        egui::__run_test_ctx(|ctx| {
            install_restored(
                ctx,
                LayoutPrefs {
                    tree_width: Some(200.0),
                    inspector_width: None,
                    dock_height: Some(150.0),
                    inspector_collapsed: false,
                    dock_collapsed: false,
                },
            );
            assert_eq!(restored_tree_width(ctx), Some(200.0));
            assert_eq!(restored_inspector_width(ctx), None);
            assert_eq!(restored_dock_height(ctx), Some(150.0));
        });
    }
}
