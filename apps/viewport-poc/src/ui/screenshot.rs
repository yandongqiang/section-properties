//! Viewport screenshot export (Phase P7-3, C1) — no new dependencies.
//!
//! # How the capture works (renderer-agnostic)
//!
//! [`request`] sends `egui::ViewportCommand::Screenshot` to the viewport
//! integration.  eframe (both the glow and wgpu backends) handles it **after
//! painting the frame** by reading back the composited window framebuffer
//! (`Painter::read_screen_rgba`) and pushing an
//! [`egui::Event::Screenshot`](egui::Event::Screenshot) carrying an
//! [`egui::ColorImage`] into the input events of a later frame.
//!
//! Because the read-back happens on the *composited window*, the capture is
//! **renderer-agnostic**: the Software renderer draws the viewport with the
//! egui painter, and the GPU renderer draws it through a paint callback into
//! the same framebuffer — either way the final image contains the viewport,
//! panels, menu and ribbon.  The application loop (see `main.rs`) drains the
//! event, encodes the image and reports the saved path in the Messages dock.
//!
//! # File format
//!
//! Uncompressed 24-bit BMP (`BI_RGB`), written bottom-up in BGR order with
//! 4-byte row padding — a ~40-line encoder with no dependency on an image
//! crate.  Alpha is dropped (the captured window is opaque).

use std::path::{Path, PathBuf};

/// Ask egui/eframe to capture the composited window.
///
/// The reply arrives as [`egui::Event::Screenshot`] on a later frame; the
/// application loop handles it (see `main.rs`).
pub fn request(ctx: &egui::Context) {
    ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot);
}

/// Encode an [`egui::ColorImage`] as an uncompressed 24-bit BMP.
///
/// `ColorImage` pixels are RGBA, row by row, **top to bottom**; BMP stores
/// rows **bottom-up** as BGR triples padded to a 4-byte boundary.
pub fn encode_bmp(image: &egui::ColorImage) -> Vec<u8> {
    let [w, h] = image.size;
    let row_bytes = w * 3;
    let pad = (4 - (row_bytes % 4)) % 4;
    let row_size = row_bytes + pad;
    let pixel_size = row_size * h;
    let file_size = 54 + pixel_size;

    let mut out = Vec::with_capacity(file_size);

    // ── BITMAPFILEHEADER (14 bytes) ──
    out.extend_from_slice(b"BM");
    out.extend_from_slice(&(file_size as u32).to_le_bytes());
    out.extend_from_slice(&0u16.to_le_bytes()); // reserved
    out.extend_from_slice(&0u16.to_le_bytes()); // reserved
    out.extend_from_slice(&54u32.to_le_bytes()); // pixel data offset

    // ── BITMAPINFOHEADER (40 bytes) ──
    out.extend_from_slice(&40u32.to_le_bytes()); // header size
    out.extend_from_slice(&(w as i32).to_le_bytes()); // width
    out.extend_from_slice(&(h as i32).to_le_bytes()); // height (positive = bottom-up)
    out.extend_from_slice(&1u16.to_le_bytes()); // planes
    out.extend_from_slice(&24u16.to_le_bytes()); // bits per pixel
    out.extend_from_slice(&0u32.to_le_bytes()); // compression = BI_RGB
    out.extend_from_slice(&(pixel_size as u32).to_le_bytes()); // image size
    out.extend_from_slice(&2835i32.to_le_bytes()); // 72 DPI, x
    out.extend_from_slice(&2835i32.to_le_bytes()); // 72 DPI, y
    out.extend_from_slice(&0u32.to_le_bytes()); // colours used
    out.extend_from_slice(&0u32.to_le_bytes()); // important colours
    debug_assert_eq!(out.len(), 54);

    // ── Pixel data: bottom-up rows, BGR order, 4-byte row padding ──
    for y in (0..h).rev() {
        for x in 0..w {
            let c = image.pixels[y * w + x];
            out.push(c.b());
            out.push(c.g());
            out.push(c.r());
        }
        out.extend(std::iter::repeat_n(0u8, pad));
    }
    debug_assert_eq!(out.len(), file_size);

    out
}

/// The default capture directory.
///
/// * Windows: `%USERPROFILE%\Pictures\section-properties`
/// * otherwise: `$HOME/Pictures/section-properties`
/// * fallback (no home env var): the current directory
pub fn capture_dir() -> PathBuf {
    let home = if cfg!(windows) {
        std::env::var_os("USERPROFILE").map(PathBuf::from)
    } else {
        std::env::var_os("HOME").map(PathBuf::from)
    };
    match home {
        Some(home) => home.join("Pictures").join("section-properties"),
        None => PathBuf::from("."),
    }
}

/// The next capture index in `dir`: one past the highest existing
/// `capture-<n>.bmp` (1 in an empty / unreadable directory).
fn next_capture_index(dir: &Path) -> u32 {
    let mut max = 0u32;
    if let Ok(entries) = std::fs::read_dir(dir) {
        for entry in entries.flatten() {
            let name = entry.file_name();
            let name = name.to_string_lossy();
            if let Some(digits) = name
                .strip_prefix("capture-")
                .and_then(|s| s.strip_suffix(".bmp"))
                && let Ok(n) = digits.parse::<u32>()
            {
                max = max.max(n);
            }
        }
    }
    max + 1
}

/// Save `image` into `dir` as `capture-<n>.bmp`, returning the full path.
pub fn save_to(dir: &Path, image: &egui::ColorImage) -> Result<PathBuf, String> {
    std::fs::create_dir_all(dir)
        .map_err(|e| format!("could not create {} ({e})", dir.display()))?;
    let path = dir.join(format!("capture-{}.bmp", next_capture_index(dir)));
    let bytes = encode_bmp(image);
    std::fs::write(&path, &bytes)
        .map_err(|e| format!("could not write {} ({e})", path.display()))?;
    Ok(path)
}

/// Save `image` as `capture-<n>.bmp` in the default [`capture_dir`], falling
/// back to the current directory when that location is unavailable.
pub fn save(image: &egui::ColorImage) -> Result<PathBuf, String> {
    let preferred = capture_dir();
    match save_to(&preferred, image) {
        Ok(path) => Ok(path),
        Err(first) => {
            let fallback = PathBuf::from(".");
            save_to(&fallback, image).map_err(|second| {
                format!(
                    "could not write to {} ({first}) or the current directory ({second})",
                    preferred.display()
                )
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tiny image with a known colour per pixel (row-major, top-down).
    fn tiny() -> egui::ColorImage {
        // 3x2 — width 3 exercises the BMP 4-byte row padding (9 → 12).
        let pixels = vec![
            egui::Color32::from_rgb(10, 20, 30),
            egui::Color32::from_rgb(40, 50, 60),
            egui::Color32::from_rgb(70, 80, 90),
            egui::Color32::from_rgb(100, 110, 120),
            egui::Color32::from_rgb(130, 140, 150),
            egui::Color32::from_rgb(160, 170, 180),
        ];
        egui::ColorImage {
            size: [3, 2],
            pixels,
        }
    }

    #[test]
    fn bmp_header_encodes_dimensions_and_format() {
        let bytes = encode_bmp(&tiny());
        // Row: 3 px * 3 B = 9 bytes, padded to 12; 2 rows → 24; + 54 header.
        assert_eq!(bytes.len(), 54 + 12 * 2);
        assert_eq!(&bytes[0..2], b"BM");
        assert_eq!(u32::from_le_bytes(bytes[2..6].try_into().unwrap()), 78); // file size
        assert_eq!(u32::from_le_bytes(bytes[10..14].try_into().unwrap()), 54); // pixel offset
        assert_eq!(u32::from_le_bytes(bytes[14..18].try_into().unwrap()), 40); // info size
        assert_eq!(i32::from_le_bytes(bytes[18..22].try_into().unwrap()), 3); // width
        assert_eq!(i32::from_le_bytes(bytes[22..26].try_into().unwrap()), 2); // height
        assert_eq!(u16::from_le_bytes(bytes[26..28].try_into().unwrap()), 1); // planes
        assert_eq!(u16::from_le_bytes(bytes[28..30].try_into().unwrap()), 24); // bpp
        assert_eq!(u32::from_le_bytes(bytes[30..34].try_into().unwrap()), 0); // BI_RGB
        assert_eq!(u32::from_le_bytes(bytes[34..38].try_into().unwrap()), 24); // image size
    }

    #[test]
    fn bmp_pixels_are_bgr_and_bottom_up() {
        let bytes = encode_bmp(&tiny());
        let row_size = 12usize;
        // Image top-left (10,20,30) lives in the LAST BMP row, column 0.
        let off = 54 + row_size; // second (bottom) BMP row
        assert_eq!(&bytes[off..off + 3], &[30, 20, 10]); // B,G,R
        // Image bottom-right (160,170,180) lives in the FIRST BMP row.
        let off = 54 + 2 * 3; // column 2 of the top BMP row
        assert_eq!(&bytes[off..off + 3], &[180, 170, 160]);
        // The 3 padding bytes of the first row are zero.
        let off = 54 + 9;
        assert_eq!(&bytes[off..off + 3], &[0, 0, 0]);
    }

    #[test]
    fn next_capture_index_is_max_plus_one() {
        let dir = std::env::temp_dir().join("viewport_poc_screenshot_index_test");
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();

        // Empty directory → 1.
        assert_eq!(next_capture_index(&dir), 1);

        for name in ["capture-1.bmp", "capture-7.bmp", "capture-x.bmp", "notes.txt"] {
            std::fs::write(dir.join(name), b"junk").unwrap();
        }
        assert_eq!(next_capture_index(&dir), 8);

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn save_to_writes_sequential_capture_files() {
        let dir = std::env::temp_dir().join("viewport_poc_screenshot_save_test");
        let _ = std::fs::remove_dir_all(&dir);

        let first = save_to(&dir, &tiny()).unwrap();
        assert_eq!(first, dir.join("capture-1.bmp"));
        let bytes = std::fs::read(&first).unwrap();
        assert_eq!(&bytes[0..2], b"BM");
        assert_eq!(bytes.len(), 54 + 12 * 2);

        let second = save_to(&dir, &tiny()).unwrap();
        assert_eq!(second, dir.join("capture-2.bmp"));

        let _ = std::fs::remove_dir_all(&dir);
    }
}
