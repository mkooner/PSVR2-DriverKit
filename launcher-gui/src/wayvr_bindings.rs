// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//! WayVR on monado-service: the PS VR2 Sense controllers' bindings in WayVR's own OpenXR actions file.
//!
//! WayVR (a separate program, installed by the user) reads `$XDG_CONFIG_HOME/wayvr/openxr_actions.json5` (else
//! `~/.config/wayvr/`): a JSON5 list of interaction-profile blocks whose entries replace or add to its built-in ones
//! (a block replaces the whole profile). It has no PS VR2 block of its own, so on Monado it gets the Sense
//! controllers through Monado's Touch rebinding, where show/hide is a double press of the left Triangle (Touch's Y).
//! This adds a PS VR2 block (`/interaction_profiles/mndx/pssense_controller_mndx`, XR_MNDX_psvr2_interaction, which
//! WayVR enables when the runtime has it) with every action on the same Sense button as through that rebinding,
//! except show/hide: a single press of Options (right controller) or Create (left controller, in the same place).
//!
//! Only the block between the two marker lines is ever written; the rest of the file (other profiles, comments) is
//! kept as it is. A file that doesn't parse, isn't a list, or already has the user's own PS VR2 block is left alone.

use std::fs;
use std::path::{Path, PathBuf};

pub const PROFILE: &str = "/interaction_profiles/mndx/pssense_controller_mndx";
const BEGIN: &str = "// >>> PSVR2-DriverKit: PS VR2 Sense controllers (written by the DriverKit launcher; anything outside these markers is yours)";
const END: &str = "// <<< PSVR2-DriverKit";
pub const BACKUP_SUFFIX: &str = ".driverkit-backup";

/// The block: WayVR's own Oculus Touch actions moved onto the Sense buttons as Monado's Touch -> PS VR2 rebinding
/// does (Y -> Triangle, X -> Square, A -> Cross, B -> Circle, left menu -> left PS button, squeeze value -> squeeze
/// click), show/hide on Options.
pub fn block() -> String {
    format!(
        r#"  {BEGIN}
  {{
    profile: "{PROFILE}",
    pose: {{ left: "/user/hand/left/input/aim/pose", right: "/user/hand/right/input/aim/pose" }},
    haptic: {{ left: "/user/hand/left/output/haptic", right: "/user/hand/right/output/haptic" }},
    click: {{ left: "/user/hand/left/input/trigger/value", right: "/user/hand/right/input/trigger/value" }},
    grab: {{ left: "/user/hand/left/input/squeeze/click", right: "/user/hand/right/input/squeeze/click" }},
    scroll: {{ left: "/user/hand/left/input/thumbstick", right: "/user/hand/right/input/thumbstick" }},
    // Show / hide WayVR: one press of Options (right controller) or Create (left controller).
    show_hide: {{ left: "/user/hand/left/input/share_mndx/click", right: "/user/hand/right/input/options_mndx/click" }},
    space_drag: {{ left: "/user/hand/left/input/system/click" }},
    space_reset: {{ double_click: true, left: "/user/hand/left/input/system/click" }},
    click_modifier_right: {{ left: "/user/hand/left/input/triangle_mndx/touch", right: "/user/hand/right/input/circle_mndx/touch" }},
    click_modifier_middle: {{ left: "/user/hand/left/input/square_mndx/touch", right: "/user/hand/right/input/cross_mndx/touch" }},
    move_mouse: {{ left: "/user/hand/left/input/trigger/touch", right: "/user/hand/right/input/trigger/touch" }},
  }},
  {END}
"#
    )
}

/// WayVR's config folder, as WayVR finds it (XDG config home, then `wayvr`).
pub fn config_dir() -> Option<PathBuf> {
    if let Ok(x) = std::env::var("XDG_CONFIG_HOME") {
        if !x.is_empty() {
            return Some(Path::new(&x).join("wayvr"));
        }
    }
    std::env::var("HOME").ok().map(|h| Path::new(&h).join(".config").join("wayvr"))
}

#[derive(Debug, PartialEq)]
pub enum Outcome {
    /// Written (created or updated); `backup` when the original was saved first.
    Written { created: bool, backup: Option<PathBuf> },
    /// Already up to date: nothing written.
    Unchanged,
    /// Left alone, with the reason (the user's own PS VR2 block, a file that doesn't parse or isn't a list, ...).
    LeftAlone(String),
}

fn parse(text: &str) -> Result<serde_json::Value, String> {
    json5::from_str::<serde_json::Value>(text).map_err(|e| e.to_string())
}

fn has_profile(v: &serde_json::Value) -> bool {
    v.as_array().map_or(false, |a| a.iter().any(|p| p.get("profile").and_then(|x| x.as_str()) == Some(PROFILE)))
}

/// Where the top-level list opens (the `[` after leading whitespace and comments), if the file is one.
fn list_start(text: &str) -> Option<usize> {
    let b = text.as_bytes();
    let mut i = 0;
    while i < b.len() {
        match b[i] {
            b' ' | b'\t' | b'\r' | b'\n' => i += 1,
            b'/' if i + 1 < b.len() && b[i + 1] == b'/' => {
                while i < b.len() && b[i] != b'\n' {
                    i += 1;
                }
            }
            b'/' if i + 1 < b.len() && b[i + 1] == b'*' => {
                i += 2;
                while i + 1 < b.len() && !(b[i] == b'*' && b[i + 1] == b'/') {
                    i += 1;
                }
                i += 2;
            }
            b'[' => return Some(i),
            _ => return None,
        }
    }
    None
}

/// The new text for `existing` (None: no file) with our block in it, or why the file is left alone.
pub fn updated_text(existing: Option<&str>) -> Result<Option<String>, String> {
    let ours = block();
    let Some(old) = existing else {
        return Ok(Some(format!("[\n{ours}]\n")));
    };
    let parsed = parse(old).map_err(|e| format!("it doesn't parse as JSON5 ({e}); WayVR would ignore it too"))?;
    if !parsed.is_array() {
        return Err("it isn't a list of profiles (WayVR's format)".into());
    }
    let new = if let (Some(b), Some(e)) = (old.find(BEGIN), old.find(END)) {
        if e < b {
            return Err("its DriverKit markers are out of order".into());
        }
        // Our block: from the start of the BEGIN line to the end of the END line.
        let line_start = old[..b].rfind('\n').map_or(0, |i| i + 1);
        let line_end = old[e..].find('\n').map_or(old.len(), |i| e + i + 1);
        let mut n = String::with_capacity(old.len() + ours.len());
        n.push_str(&old[..line_start]);
        n.push_str(&ours);
        n.push_str(&old[line_end..]);
        n
    } else {
        if has_profile(&parsed) {
            return Err(format!("it already has your own {PROFILE} block (kept as it is)"));
        }
        let at = list_start(old).ok_or("its list couldn't be found")?;
        let mut n = String::with_capacity(old.len() + ours.len() + 1);
        n.push_str(&old[..=at]);
        n.push('\n');
        n.push_str(&ours);
        n.push_str(&old[at + 1..]);
        n
    };
    if new == old {
        return Ok(None);
    }
    // Validate: still JSON5, still a list, our block in it, every other profile kept.
    let check = parse(&new).map_err(|e| format!("the update didn't validate ({e}); nothing written"))?;
    let others = |v: &serde_json::Value| -> Vec<String> {
        v.as_array()
            .map(|a| {
                a.iter()
                    .filter_map(|p| p.get("profile").and_then(|x| x.as_str()))
                    .filter(|p| *p != PROFILE)
                    .map(String::from)
                    .collect()
            })
            .unwrap_or_default()
    };
    if !has_profile(&check) || others(&check) != others(&parsed) {
        return Err("the update didn't keep the file's other profiles; nothing written".into());
    }
    Ok(Some(new))
}

/// Puts our block in `<dir>/openxr_actions.json5`: backs the original up once (never over an earlier backup), writes
/// through a temporary file, checks what was written.
pub fn apply(dir: &Path) -> Outcome {
    let path = dir.join("openxr_actions.json5");
    let existing = match fs::read_to_string(&path) {
        Ok(t) => Some(t),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => None,
        Err(e) => return Outcome::LeftAlone(format!("{} couldn't be read ({e})", path.display())),
    };
    let new = match updated_text(existing.as_deref()) {
        Ok(None) => return Outcome::Unchanged,
        Ok(Some(n)) => n,
        Err(why) => return Outcome::LeftAlone(format!("{}: {why}", path.display())),
    };
    if let Err(e) = fs::create_dir_all(dir) {
        return Outcome::LeftAlone(format!("{} couldn't be created ({e})", dir.display()));
    }
    let mut backup = None;
    if let Some(old) = &existing {
        let b = dir.join(format!("openxr_actions.json5{BACKUP_SUFFIX}"));
        if !b.exists() {
            if let Err(e) = fs::write(&b, old) {
                return Outcome::LeftAlone(format!("the backup {} couldn't be written ({e}); nothing changed", b.display()));
            }
            backup = Some(b);
        }
    }
    let tmp = dir.join("openxr_actions.json5.driverkit-tmp");
    if let Err(e) = fs::write(&tmp, &new).and_then(|_| fs::rename(&tmp, &path)) {
        let _ = fs::remove_file(&tmp);
        return Outcome::LeftAlone(format!("{} couldn't be written ({e}); the original is unchanged", path.display()));
    }
    // What's on disk now must be what was validated; otherwise put the original back.
    if fs::read_to_string(&path).ok().as_deref() != Some(new.as_str()) {
        if let Some(old) = &existing {
            let _ = fs::write(&path, old);
        }
        return Outcome::LeftAlone(format!("{} didn't read back as written; the original was restored", path.display()));
    }
    Outcome::Written { created: existing.is_none(), backup }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmpdir(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("driverkit-wayvr-test-{name}-{}", std::process::id()));
        let _ = fs::remove_dir_all(&d);
        fs::create_dir_all(&d).unwrap();
        d
    }

    #[test]
    fn fresh_config_gets_only_our_block() {
        let d = tmpdir("fresh");
        assert_eq!(apply(&d), Outcome::Written { created: true, backup: None });
        let v = parse(&fs::read_to_string(d.join("openxr_actions.json5")).unwrap()).unwrap();
        let a = v.as_array().unwrap();
        assert_eq!(a.len(), 1);
        assert_eq!(a[0]["profile"], PROFILE);
        assert_eq!(a[0]["show_hide"]["right"], "/user/hand/right/input/options_mndx/click");
        assert_eq!(a[0]["show_hide"]["left"], "/user/hand/left/input/share_mndx/click");
        assert!(a[0]["show_hide"].get("double_click").is_none());
        // a restart: nothing written, no backup
        assert_eq!(apply(&d), Outcome::Unchanged);
        assert!(!d.join(format!("openxr_actions.json5{BACKUP_SUFFIX}")).exists());
    }

    #[test]
    fn custom_config_keeps_everything_and_backs_up_once() {
        let d = tmpdir("custom");
        let user = "// my bindings\n[\n  /* index */\n  { profile: \"/interaction_profiles/valve/index_controller\", show_hide: { left: \"/user/hand/left/input/a/click\" } },\n]\n";
        fs::write(d.join("openxr_actions.json5"), user).unwrap();
        match apply(&d) {
            Outcome::Written { created: false, backup: Some(_) } => {}
            o => panic!("{o:?}"),
        }
        let text = fs::read_to_string(d.join("openxr_actions.json5")).unwrap();
        assert!(text.starts_with("// my bindings\n["));
        assert!(text.contains("/* index */") && text.contains("valve/index_controller"));
        let v = parse(&text).unwrap();
        assert_eq!(v.as_array().unwrap().len(), 2);
        assert_eq!(fs::read_to_string(d.join(format!("openxr_actions.json5{BACKUP_SUFFIX}"))).unwrap(), user);
        // restarts: unchanged; the backup isn't replaced
        assert_eq!(apply(&d), Outcome::Unchanged);
        assert_eq!(fs::read_to_string(d.join(format!("openxr_actions.json5{BACKUP_SUFFIX}"))).unwrap(), user);
    }

    #[test]
    fn an_older_block_of_ours_is_replaced_in_place() {
        let d = tmpdir("update");
        let old = format!("[\n  {{ profile: \"/interaction_profiles/khr/generic_controller\" }},\n  {BEGIN}\n  {{ profile: \"{PROFILE}\", show_hide: {{ double_click: true, left: \"/user/hand/left/input/triangle_mndx/click\" }} }},\n  {END}\n]\n");
        fs::write(d.join("openxr_actions.json5"), &old).unwrap();
        assert!(matches!(apply(&d), Outcome::Written { created: false, .. }));
        let text = fs::read_to_string(d.join("openxr_actions.json5")).unwrap();
        assert_eq!(text.matches(BEGIN).count(), 1);
        let v = parse(&text).unwrap();
        let a = v.as_array().unwrap();
        assert_eq!(a.len(), 2);
        assert_eq!(a[0]["profile"], "/interaction_profiles/khr/generic_controller");
        assert_eq!(a[1]["show_hide"]["right"], "/user/hand/right/input/options_mndx/click");
    }

    #[test]
    fn the_users_own_psvr2_block_and_bad_files_are_left_alone() {
        let d = tmpdir("own");
        let own = format!("[ {{ profile: \"{PROFILE}\", show_hide: {{ left: \"/user/hand/left/input/share_mndx/click\" }} }} ]\n");
        fs::write(d.join("openxr_actions.json5"), &own).unwrap();
        assert!(matches!(apply(&d), Outcome::LeftAlone(_)));
        assert_eq!(fs::read_to_string(d.join("openxr_actions.json5")).unwrap(), own);
        for bad in ["[ { profile: ", "{ \"not\": \"a list\" }"] {
            let d = tmpdir("bad");
            fs::write(d.join("openxr_actions.json5"), bad).unwrap();
            assert!(matches!(apply(&d), Outcome::LeftAlone(_)), "{bad}");
            assert_eq!(fs::read_to_string(d.join("openxr_actions.json5")).unwrap(), bad);
            assert!(!d.join(format!("openxr_actions.json5{BACKUP_SUFFIX}")).exists());
        }
    }
}
