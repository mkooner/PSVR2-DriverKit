// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//! PSVR2-DriverKit's look, from the PlayStation VR2 store page the user picked: a navy-to-blue gradient background,
//! translucent tile-blue sections, white text, white for the focused (primary) action as on the PS5, and the devices as
//! picture tiles whose ring lights up blue like the Sense controllers' light bars when connected.

use eframe::egui;
use egui::{Color32, CornerRadius, FontFamily, FontId, Margin, Stroke, TextStyle};

// Sampled from the picture (region averages).
pub const NAVY_TOP: Color32 = Color32::from_rgb(0x03, 0x04, 0x36); // background, top
pub const NAVY_GLOW: Color32 = Color32::from_rgb(0x04, 0x18, 0x54); // background, behind the headset
pub const NAVY_LOW: Color32 = Color32::from_rgb(0x05, 0x26, 0x52); // background, bottom
pub const TILE: Color32 = Color32::from_rgb(0x14, 0x46, 0x96); // store tiles: secondary buttons
pub const NAVY: Color32 = Color32::from_rgb(0x07, 0x1A, 0x4F); // text on white
pub const PANEL: Color32 = Color32::from_rgba_premultiplied(0x07, 0x1C, 0x40, 0xB0); // sections (translucent tile)
pub const VISOR: Color32 = Color32::from_rgb(0x1E, 0x40, 0x86); // borders
pub const FIELD: Color32 = Color32::from_rgb(0x05, 0x14, 0x3E); // text fields, commands
pub const GRAPHITE: Color32 = Color32::from_rgb(0xF4, 0xF6, 0xFF); // text (white)
pub const SLATE: Color32 = Color32::from_rgb(0x9D, 0xB0, 0xD8); // secondary text
pub const PS_BLUE: Color32 = Color32::from_rgb(0x2F, 0x7B, 0xFF); // progress, selection
pub const LIT: Color32 = Color32::from_rgb(0x5A, 0xB8, 0xFF); // connected: the controllers' light-bar blue
pub const FAULT: Color32 = Color32::from_rgb(0xFF, 0x7A, 0x7A); // errors

/// The first font file fontconfig resolves for one of these families (it falls back to any font, so the resolved
/// family has to match).
fn font_file(families: &[&str]) -> Option<Vec<u8>> {
    for fam in families {
        let out = std::process::Command::new("fc-match").args(["-f", "%{family}|%{file}", fam]).output().ok()?;
        let text = String::from_utf8_lossy(&out.stdout).to_string();
        let Some((found, file)) = text.split_once('|') else { continue };
        let want = fam.split(':').next().unwrap_or(fam).to_lowercase();
        if found.to_lowercase().contains(&want) && (file.ends_with(".ttf") || file.ends_with(".otf")) {
            if let Ok(bytes) = std::fs::read(file) {
                return Some(bytes);
            }
        }
    }
    None
}

fn fonts() -> egui::FontDefinitions {
    let mut f = egui::FontDefinitions::default();
    let mut add = |name: &str, families: &[&str], family: FontFamily| {
        if let Some(bytes) = font_file(families) {
            f.font_data.insert(name.into(), std::sync::Arc::new(egui::FontData::from_owned(bytes)));
            f.families.entry(family).or_default().insert(0, name.into());
        }
    };
    add("ui", &["Noto Sans:weight=regular", "Cantarell", "DejaVu Sans"], FontFamily::Proportional);
    add("mono", &["JetBrains Mono:weight=regular", "Noto Sans Mono", "DejaVu Sans Mono"], FontFamily::Monospace);
    let bold = FontFamily::Name("bold".into());
    let regular = f.families.get(&FontFamily::Proportional).cloned().unwrap_or_default();
    if let Some(bytes) = font_file(&["Noto Sans:weight=bold", "Cantarell:weight=bold", "DejaVu Sans:weight=bold"]) {
        f.font_data.insert("ui-bold".into(), std::sync::Arc::new(egui::FontData::from_owned(bytes)));
        let mut fam = vec!["ui-bold".to_string()];
        fam.extend(regular);
        f.families.insert(bold, fam);
    } else {
        f.families.insert(bold, regular);
    }
    f
}

pub fn bold(size: f32) -> FontId {
    FontId::new(size, FontFamily::Name("bold".into()))
}

pub fn install(ctx: &egui::Context) {
    ctx.set_fonts(fonts());
    ctx.set_theme(egui::Theme::Dark);
    ctx.style_mut_of(egui::Theme::Dark, |style| {
        style.text_styles = [
            (TextStyle::Heading, bold(17.0)),
            (TextStyle::Body, FontId::new(14.0, FontFamily::Proportional)),
            (TextStyle::Button, FontId::new(14.0, FontFamily::Proportional)),
            (TextStyle::Small, FontId::new(12.0, FontFamily::Proportional)),
            (TextStyle::Monospace, FontId::new(13.0, FontFamily::Monospace)),
        ]
        .into();
        style.spacing.item_spacing = egui::vec2(10.0, 8.0);
        style.spacing.button_padding = egui::vec2(14.0, 6.0);
        style.spacing.interact_size.y = 30.0;
        let v = &mut style.visuals;
        v.dark_mode = true;
        v.panel_fill = Color32::TRANSPARENT;
        v.window_fill = NAVY_GLOW;
        v.extreme_bg_color = FIELD;
        v.faint_bg_color = Color32::from_rgba_unmultiplied(0x14, 0x46, 0x96, 0x38);
        v.code_bg_color = FIELD;
        v.override_text_color = Some(GRAPHITE);
        v.hyperlink_color = LIT;
        v.selection.bg_fill = PS_BLUE.gamma_multiply(0.6);
        v.selection.stroke = Stroke::new(1.0, GRAPHITE);
        v.window_stroke = Stroke::new(1.0, VISOR);
        let r = CornerRadius::same(8);
        for w in [&mut v.widgets.noninteractive, &mut v.widgets.inactive, &mut v.widgets.hovered, &mut v.widgets.active, &mut v.widgets.open] {
            w.corner_radius = r;
        }
        v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, VISOR);
        v.widgets.noninteractive.fg_stroke = Stroke::new(1.0, GRAPHITE);
        v.widgets.inactive.bg_fill = TILE;
        v.widgets.inactive.weak_bg_fill = TILE;
        v.widgets.inactive.bg_stroke = Stroke::new(1.0, Color32::from_rgb(0x2A, 0x5C, 0xB4));
        v.widgets.inactive.fg_stroke = Stroke::new(1.0, GRAPHITE);
        // Focus/hover as on the PS5: a white outline.
        v.widgets.hovered.bg_fill = Color32::from_rgb(0x1B, 0x56, 0xB0);
        v.widgets.hovered.weak_bg_fill = Color32::from_rgb(0x1B, 0x56, 0xB0);
        v.widgets.hovered.bg_stroke = Stroke::new(1.5, GRAPHITE);
        v.widgets.hovered.fg_stroke = Stroke::new(1.0, Color32::WHITE);
        v.widgets.active.bg_fill = Color32::from_rgb(0x22, 0x63, 0xC8);
        v.widgets.active.weak_bg_fill = Color32::from_rgb(0x22, 0x63, 0xC8);
        v.widgets.active.bg_stroke = Stroke::new(2.0, Color32::WHITE);
        v.widgets.active.fg_stroke = Stroke::new(1.0, Color32::WHITE);
    });
}

/// The window's background: the picture's navy-to-blue gradient with the glow behind the headset.
pub fn paint_background(ui: &egui::Ui) {
    let rect = ui.ctx().content_rect();
    let p = ui.ctx().layer_painter(egui::LayerId::background());
    let mut mesh = egui::Mesh::default();
    let mid = rect.top() + rect.height() * 0.45;
    let bands = [(rect.top(), NAVY_TOP), (mid, NAVY_GLOW), (rect.bottom(), NAVY_LOW)];
    for w in bands.windows(2) {
        let (y0, c0) = w[0];
        let (y1, c1) = w[1];
        let i = mesh.vertices.len() as u32;
        mesh.colored_vertex(egui::pos2(rect.left(), y0), c0);
        mesh.colored_vertex(egui::pos2(rect.right(), y0), c0);
        mesh.colored_vertex(egui::pos2(rect.left(), y1), c1);
        mesh.colored_vertex(egui::pos2(rect.right(), y1), c1);
        mesh.add_triangle(i, i + 1, i + 2);
        mesh.add_triangle(i + 1, i + 2, i + 3);
    }
    p.add(egui::Shape::mesh(mesh));
    // The soft blue glow, upper right, as behind the headset in the picture.
    let c = egui::pos2(rect.right() - rect.width() * 0.22, rect.top() + rect.height() * 0.28);
    for (r, a) in [(460.0, 14u8), (330.0, 16), (210.0, 18)] {
        p.circle_filled(c, r, Color32::from_rgba_unmultiplied(0x1A, 0x4C, 0xC0, a));
    }
}

/// A picture (assets/*.png) as a texture.
pub fn texture(ctx: &egui::Context, name: &str, png_bytes: &[u8]) -> Option<egui::TextureHandle> {
    let mut dec = png::Decoder::new(std::io::Cursor::new(png_bytes));
    dec.set_transformations(png::Transformations::EXPAND | png::Transformations::STRIP_16);
    let mut reader = dec.read_info().ok()?;
    let mut buf = vec![0; reader.output_buffer_size()?];
    let info = reader.next_frame(&mut buf).ok()?;
    let (w, h) = (info.width as usize, info.height as usize);
    let rgba: Vec<u8> = match info.color_type {
        png::ColorType::Rgba => buf[..w * h * 4].to_vec(),
        png::ColorType::Rgb => buf[..w * h * 3].chunks(3).flat_map(|c| [c[0], c[1], c[2], 255]).collect(),
        _ => return None,
    };
    let img = egui::ColorImage::from_rgba_unmultiplied([w, h], &rgba);
    Some(ctx.load_texture(name, img, egui::TextureOptions::LINEAR))
}

/// A section: a translucent tile-blue panel, its title, then its contents.
pub fn section<R>(ui: &mut egui::Ui, title: &str, add: impl FnOnce(&mut egui::Ui) -> R) -> R {
    let r = egui::Frame::new()
        .fill(PANEL)
        .stroke(Stroke::new(1.0, VISOR))
        .corner_radius(CornerRadius::same(12))
        .inner_margin(Margin::symmetric(18, 14))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.label(egui::RichText::new(title).font(bold(16.0)).color(GRAPHITE));
            ui.add_space(4.0);
            add(ui)
        })
        .inner;
    ui.add_space(12.0);
    r
}

/// The section's one primary action: white with navy text, as a focused item on the PS5.
pub fn primary(text: &str) -> egui::Button<'static> {
    egui::Button::new(egui::RichText::new(text.to_string()).color(NAVY).font(bold(14.0))).fill(Color32::WHITE).stroke(Stroke::NONE)
}

/// Secondary text: what a section or control does.
pub fn note(ui: &mut egui::Ui, text: &str) {
    ui.label(egui::RichText::new(text).color(SLATE).size(13.0));
}

/// One device: its picture above its name and state. The picture's ring glows light-bar blue when connected.
pub fn device(ui: &mut egui::Ui, picture: Option<&egui::TextureHandle>, on: Option<bool>, label: &str) {
    let lit = on == Some(true);
    let state = match on {
        Some(true) => "Connected",
        Some(false) => "Not connected",
        None => "Not paired",
    };
    egui::Frame::new()
        .fill(if lit { Color32::from_rgba_premultiplied(0x10, 0x3C, 0x80, 0xC0) } else { PANEL })
        .stroke(Stroke::new(1.0, if lit { LIT.gamma_multiply(0.8) } else { VISOR }))
        .corner_radius(CornerRadius::same(12))
        .inner_margin(Margin::symmetric(14, 10))
        .show(ui, |ui| {
            ui.set_width(124.0);
            ui.vertical_centered(|ui| {
                let size = 64.0;
                let (rect, _) = ui.allocate_exact_size(egui::vec2(size + 10.0, size + 10.0), egui::Sense::hover());
                let c = rect.center();
                if lit {
                    ui.painter().circle_filled(c, size / 2.0 + 5.0, LIT.gamma_multiply(0.25));
                }
                if let Some(tex) = picture {
                    let img_rect = egui::Rect::from_center_size(c, egui::vec2(size, size));
                    egui::Image::new(tex).corner_radius(CornerRadius::same((size / 2.0) as u8)).paint_at(ui, img_rect);
                    if !lit {
                        // Dimmed while not connected.
                        ui.painter().circle_filled(c, size / 2.0, Color32::from_rgba_premultiplied(0x02, 0x05, 0x28, 0x90));
                    }
                }
                ui.painter().circle_stroke(c, size / 2.0 + 1.0, Stroke::new(2.0, if lit { LIT } else { VISOR }));
                ui.label(egui::RichText::new(label).font(bold(13.0)).color(GRAPHITE));
                ui.label(egui::RichText::new(state).size(12.0).color(if lit { LIT } else { SLATE }));
            });
        })
        .response
        .on_hover_text(format!("{label}: {state}"));
}
