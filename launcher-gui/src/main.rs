// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//! PSVR2-DriverKit: shows whether the headset and the Sense controllers are connected, starts and stops
//! SteamVR with our Monado plugin, monado-service, and any OpenXR app on monado-service, shows the environment an app
//! needs to use Monado, and keeps the driver settings we use (with their defaults) switchable.

mod theme;

use eframe::egui;
use serde::{Deserialize, Serialize};
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

/*
 * Settings.
 */

#[derive(Clone, Serialize, Deserialize, PartialEq)]
enum Kind {
    /// Environment variable set to `on` when enabled and to `off` (or left unset if empty) when disabled.
    Toggle { on: String, off: String },
    /// Environment variable set to an editable value when enabled, unset when disabled.
    Value { value: String },
}

/// Which launches a setting goes to: some only mean something to one runtime (e.g. XRT_COMPOSITOR_SCALE_PERCENTAGE is
/// the SteamVR driver's render size but Monado's compositor scale in monado-service).
#[derive(Clone, Copy, Serialize, Deserialize, PartialEq, Default)]
enum Target {
    #[default]
    Both,
    SteamVR,
    Monado,
}

impl Target {
    fn label(self) -> &'static str {
        match self {
            Target::Both => "SteamVR + Monado",
            Target::SteamVR => "SteamVR",
            Target::Monado => "Monado",
        }
    }
}

#[derive(Clone, Serialize, Deserialize)]
struct Setting {
    key: String,
    label: String,
    help: String,
    kind: Kind,
    enabled: bool,
    /// Always taken from this version's defaults (not the saved file).
    #[serde(default)]
    target: Target,
}

impl Setting {
    fn steamvr(mut self) -> Self {
        self.target = Target::SteamVR;
        self
    }
    fn monado(mut self) -> Self {
        self.target = Target::Monado;
        self
    }
}

fn toggle(key: &str, label: &str, help: &str, enabled: bool) -> Setting {
    Setting {
        key: key.into(),
        label: label.into(),
        help: help.into(),
        kind: Kind::Toggle { on: "1".into(), off: "0".into() },
        enabled,
        target: Target::Both,
    }
}

fn toggle_values(key: &str, label: &str, help: &str, on: &str, off: &str, enabled: bool) -> Setting {
    Setting { kind: Kind::Toggle { on: on.into(), off: off.into() }, ..toggle(key, label, help, enabled) }
}

fn value(key: &str, label: &str, help: &str, v: &str, enabled: bool) -> Setting {
    Setting {
        key: key.into(),
        label: label.into(),
        help: help.into(),
        kind: Kind::Value { value: v.into() },
        enabled,
        target: Target::Both,
    }
}

/// The headset driver's settings (Monado's psvr2 driver), one copy per runtime so each can differ: `accel` is whether
/// accelerometer-aided position is on by default (the SteamVR plugin turns it on; OpenXR mode runs without it).
fn headset_settings(target: Target, accel: bool) -> Vec<Setting> {
    let t = |s: Setting| Setting { target, ..s };
    vec![
        t(toggle("PSVR2_ACCEL_FUSION", "Accelerometer-aided head position",
                 "Head position between SLAM poses from the 2 kHz accelerometer. On by default for the SteamVR plugin (it turns it on itself), off for OpenXR (as launch-openxr.sh runs).", accel)),
        t(value("PSVR2_SMOOTH_CORRECTION_MS", "Smooth SLAM corrections (ms)",
                "Corrections ramp in smoothly and the velocity includes them. 0 turns it off.", "30", true)),
        t(value("PSVR2_POSE_BLEND_MS", "Position correction blend (ms)",
                "Spreads each position correction over this long.", "100", true)),
        t(value("PSVR2_GRAVITY_PAUSE_DEG", "Gravity estimate pause (deg/s)",
                "With accelerometer fusion: don't update the gravity estimate while turning faster than this.", "30", true)),
        t(value("PSVR2_GYRO_BIAS_TAU", "Gyro bias correction (s)",
                "Learns the headset gyro's offset with this time constant. Off by default (the driver's default).", "5", false)),
        t(value("PSVR2_REFRESH_HZ", "Display refresh (Hz)",
                "89.91 or 119.88; the matching display mode is picked.", "89.91", true)),
        t(toggle("PSVR2_SLAM_CARRY", "Carry position over SLAM resets",
                 "Keeps the head position continuous when the headset's tracking re-originates (after its first 20 s).", true)),
        t(toggle("PSVR2_SONY_LD_CONFIG", "Sony's LED detector settings",
                 "Send Sony's LED detector settings to the headset (controller tracking in bright rooms).", true)),
        t(toggle("PSVR2_BASALT", "Basalt referee",
                 "Basalt visual-inertial tracking checks the headset's position jumps in fast turns (~2 CPU cores).", false)),
        t(value("VIT_SYSTEM_LIBRARY_PATH", "Basalt library", "libbasalt.so for the Basalt referee.", "", true)),
    ]
}

/// The options we use, with their defaults (as in the drivers and the launch scripts), in two sections: the SteamVR
/// plugin (driver_monado_psvr2, launched through launch-steamvr.sh) and OpenXR (monado-service and its apps).
fn default_settings() -> Vec<Setting> {
    let mut v = vec![
        // SteamVR plugin (driver_monado_psvr2)
        toggle("ASYNC", "SteamVR async reprojection",
               "On (default): the plugin asks SteamVR for it for the session; late frames are moved to the new head pose instead of shown again. Off: SteamVR's own default (off on NVIDIA under Linux).", true).steamvr(),
        toggle("LINUX_FIXES", "SteamVRLinuxFixes layer",
               "Off (default): the plugin switches the layer off for SteamVR's compositor (on SteamVR 2.19.3 it patches the compositor with offsets from an older build: more judder). On: keep it.", false).steamvr(),
        value("XRT_COMPOSITOR_SCALE_PERCENTAGE", "Render size per eye (%)",
              "Of the panel's 2000x2040 per eye (140 = 2800x2856, the default, as Monado renders for OpenXR; 100 = the panel's size; 170 = 3400x3468 as Sony's driver, heavier).", "140", true).steamvr(),
        value("PLAY_AREA", "Default play area (m)",
              "Side of the square play area SteamVR gets until you draw your own room (Launcher tab: Set Custom Play Area opens Room Setup; Set Default Play Area goes back to this size).", "2", true).steamvr(),
        value("OVRN_DISTORTION_MESH_RES", "Distortion mesh",
              "Grid SteamVR samples the lens distortion on (80, as Sony's driver).", "80", true).steamvr(),
        toggle_values("OVRN_POSE_MODEL", "Driver-predicted head pose (ALVR model)",
                      "Off (default): the pose for now with velocities, SteamVR predicts. On: the plugin predicts and sends no velocity (worse with games that ask for the pose more than once a frame).",
                      "predicted", "", false).steamvr(),
        value("OVRN_SONY_PROPS", "Sony's extra display properties",
              "Which of Sony's headset properties to report: all (the plugin's default, as Sony's driver and PSVR2Toolkit), none, or a list of throttle, framerate, syslayer, texindex, supersample, dsc, edid, display, mask.", "all", false).steamvr(),
        toggle("RECORD_DATASET", "Record controller dataset", "Record LED tracking data (logs/).", false).steamvr(),
        toggle("RECORD_SLAM", "Record headset SLAM", "Record the headset's raw SLAM stream (logs/).", false).steamvr(),
        toggle("RECORD_CAMERA", "Record camera images", "Record camera transfers (~2 GB/min, logs/).", false).steamvr(),
    ];
    v.extend(headset_settings(Target::SteamVR, true));
    // OpenXR (monado-service)
    v.push(value("OXR_VIEWPORT_SCALE_PERCENTAGE", "Render size (%)",
                 "Recommended render size for apps on monado-service, on top of Monado's own 140% (100, the default: 2800x2856 per eye).", "100", true).monado());
    v.extend(headset_settings(Target::Monado, false));
    v
}

#[derive(Clone, Serialize, Deserialize)]
struct Config {
    test_kit: String,
    monado_build: String,
    settings: Vec<Setting>,
    /// Steam's install folder and SteamVR's (found with "Detect Steam"); empty: the launch script's defaults.
    #[serde(default)]
    steam_root: String,
    #[serde(default)]
    steamvr_dir: String,
    /// Compile Driver tab: where the driver is compiled and installed to (the source is always our GitHub repository).
    #[serde(default = "default_install_dir")]
    install_dir: String,
    /// Settings format: 2 = async reprojection off by default (SteamVR's own choice); 3 = SteamVR render scale 170;
/// 4 = smooth SLAM corrections on.
    #[serde(default)]
    version: u32,
}

const CONFIG_VERSION: u32 = 8;

impl Default for Config {
    fn default() -> Self {
        // A fresh install: everything in the Compile Driver tab's default folder (the commands and launches stay empty
        // until a compile puts a build there; driver.env then names the real folders, see load_config).
        let install = default_install_dir();
        let settings = default_settings();
        Config {
            test_kit: format!("{install}/kit"),
            monado_build: format!("{install}/build"),
            settings,
            steam_root: String::new(),
            steamvr_dir: String::new(),
            install_dir: default_install_dir(),
            version: CONFIG_VERSION,
        }
    }
}

fn default_install_dir() -> String {
    format!("{}/.local/share/PSVR2-DriverKit", std::env::var("HOME").unwrap_or_default())
}

fn config_dir() -> PathBuf {
    config_path().parent().map(Path::to_path_buf).unwrap_or_else(|| PathBuf::from("."))
}

/// What build-driver.sh recorded about the last compile (~/.config/PSVR2-DriverKit/driver.env): KEY="value" lines.
fn driver_env() -> Vec<(String, String)> {
    fs::read_to_string(config_dir().join("driver.env"))
        .unwrap_or_default()
        .lines()
        .filter(|l| !l.trim_start().starts_with('#'))
        .filter_map(|l| l.split_once('='))
        .map(|(k, v)| (k.trim().to_string(), v.trim().trim_matches('"').to_string()))
        .collect()
}

fn config_path() -> PathBuf {
    let base = std::env::var("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from(std::env::var("HOME").unwrap_or_default()).join(".config"));
    base.join("PSVR2-DriverKit").join("settings.json")
}

/// Where the settings were kept before the app was renamed to PSVR2-DriverKit (read once if the new file is missing).
fn old_config_path() -> PathBuf {
    config_path().parent().and_then(|p| p.parent()).unwrap_or(Path::new(".")).join("psvr2-monado-launcher").join("settings.json")
}

fn load_config() -> Config {
    let mut cfg: Config = fs::read_to_string(config_path())
        .or_else(|_| fs::read_to_string(old_config_path()))
        .ok()
        .and_then(|s| serde_json::from_str(&s).ok())
        .unwrap_or_default();
    let defaults = Config::default().settings;
    // The list, names, help and kinds always come from this version; saved choices (on/off, values) are kept. A setting
    // is its key and its runtime; files before version 7 had one entry per key for both runtimes: their choice goes to
    // both copies (except accelerometer fusion, which was never meant for OpenXR mode).
    let saved = std::mem::take(&mut cfg.settings);
    for d in &defaults {
        let found = saved.iter().find(|s| s.key == d.key && s.target == d.target).or_else(|| {
            if cfg.version >= 7 || (d.target == Target::Monado && d.key == "PSVR2_ACCEL_FUSION") {
                None
            } else {
                saved.iter().find(|s| s.key == d.key && (s.target == Target::Both || s.target == d.target))
            }
        });
        let mut n = d.clone();
        if let Some(s) = found {
            n.enabled = s.enabled;
            if let (Kind::Value { value: nv }, Kind::Value { value: sv }) = (&mut n.kind, &s.kind) {
                *nv = sv.clone();
            }
        }
        cfg.settings.push(n);
    }
    if cfg.version < 2 {
        // Version 1 forced async reprojection on by default; SteamVR's own default (off on NVIDIA/Linux) now.
        for s in cfg.settings.iter_mut().filter(|s| s.key == "ASYNC") {
            s.enabled = false;
        }
    }
    if cfg.version < 3 {
        // Version 2 sent 140; Sony's driver renders 170% (3400x3468 per eye).
        for s in cfg.settings.iter_mut().filter(|s| s.key == "XRT_COMPOSITOR_SCALE_PERCENTAGE") {
            if let Kind::Value { value } = &mut s.kind {
                if value.trim() == "140" {
                    *value = "170".into();
                }
            }
        }
    }
    if cfg.version < 4 {
        for s in cfg.settings.iter_mut().filter(|s| s.key == "PSVR2_SMOOTH_CORRECTION_MS") {
            s.enabled = true;
        }
    }
    if cfg.version < 5 {
        // 2026-11-18, the best run: async reprojection on, SteamVRLinuxFixes off, the panel's render size.
        for s in cfg.settings.iter_mut().filter(|s| s.key == "ASYNC") {
            s.enabled = true;
        }
        for s in cfg.settings.iter_mut().filter(|s| s.key == "LINUX_FIXES") {
            s.enabled = false;
        }
        for s in cfg.settings.iter_mut().filter(|s| s.key == "XRT_COMPOSITOR_SCALE_PERCENTAGE") {
            if let Kind::Value { value } = &mut s.kind {
                if value.trim() == "170" {
                    *value = "100".into();
                }
            }
        }
    }
    if cfg.version < 6 {
        // The gyro bias correction was on (5 s) by default, which no tested launch used: off unless it was changed.
        for s in cfg.settings.iter_mut().filter(|s| s.key == "PSVR2_GYRO_BIAS_TAU") {
            if let Kind::Value { value } = &s.kind {
                if s.enabled && value.trim() == "5" {
                    s.enabled = false;
                }
            }
        }
    }
    if cfg.version < 8 {
        // 2026-11-18: SteamVR renders 140% by default, the size Monado's compositor gives OpenXR apps (2800x2856).
        for s in cfg.settings.iter_mut().filter(|s| s.key == "XRT_COMPOSITOR_SCALE_PERCENTAGE") {
            if let Kind::Value { value } = &mut s.kind {
                if value.trim() == "100" {
                    *value = "140".into();
                }
            }
        }
    }
    cfg.version = CONFIG_VERSION;
    // The Compile Driver tab's build, if there is one: every launch from here uses it.
    if let Some((_, b)) = driver_env().into_iter().find(|(k, _)| k == "MONADO_BUILD") {
        if Path::new(&b).join("steamvr-monado-psvr2").exists() {
            cfg.monado_build = b;
        }
    }
    cfg
}

fn save_config(cfg: &Config) -> Result<(), String> {
    let p = config_path();
    if let Some(dir) = p.parent() {
        fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }
    fs::write(&p, serde_json::to_string_pretty(cfg).map_err(|e| e.to_string())?).map_err(|e| e.to_string())
}

impl Config {
    /// Environment for a launch (SteamVR or Monado) from the settings: only the settings meant for it.
    fn env(&self, launch: Target) -> Vec<(String, String)> {
        let mut out = Vec::new();
        for s in self.settings.iter().filter(|s| s.target == Target::Both || s.target == launch) {
            match (&s.kind, s.enabled) {
                (Kind::Toggle { on, .. }, true) => out.push((s.key.clone(), on.clone())),
                (Kind::Toggle { off, .. }, false) if !off.is_empty() => out.push((s.key.clone(), off.clone())),
                (Kind::Value { value }, true) if !value.trim().is_empty() => {
                    out.push((s.key.clone(), value.trim().to_string()))
                }
                _ => {}
            }
        }
        // The SteamVR launcher knows the Basalt and refresh options by their short names too.
        if self.settings.iter().any(|s| s.key == "PSVR2_BASALT" && s.enabled && (s.target == launch || s.target == Target::Both)) {
            out.push(("BASALT".into(), "1".into()));
        }
        out
    }

    fn manifest(&self) -> PathBuf {
        Path::new(&self.monado_build).join("openxr_monado-dev.json")
    }
}

/*
 * Steam detection.
 */

/// Library folders listed in Steam's steamapps/libraryfolders.vdf ("path" entries), plus the install itself.
fn steam_libraries(root: &Path) -> Vec<PathBuf> {
    let mut libs = vec![root.to_path_buf()];
    let vdf = fs::read_to_string(root.join("steamapps/libraryfolders.vdf")).unwrap_or_default();
    for line in vdf.lines() {
        let t = line.trim();
        if let Some(rest) = t.strip_prefix("\"path\"") {
            let p = rest.trim().trim_matches('"').replace("\\\\", "\\");
            let p = PathBuf::from(p);
            if !libs.contains(&p) {
                libs.push(p);
            }
        }
    }
    libs
}

/// Find Steam's install folder (native, ~/.steam link, Flatpak) and the SteamVR install in any of its libraries.
fn detect_steam() -> (Option<PathBuf>, Option<PathBuf>, String) {
    let home = PathBuf::from(std::env::var("HOME").unwrap_or_default());
    let candidates = [
        home.join(".local/share/Steam"),
        home.join(".steam/steam"),
        home.join(".steam/root"),
        home.join(".var/app/com.valvesoftware.Steam/.local/share/Steam"),
        PathBuf::from("/usr/share/steam"),
    ];
    let mut tried = Vec::new();
    for c in candidates.iter() {
        let real = fs::canonicalize(c).unwrap_or_else(|_| c.clone());
        tried.push(c.display().to_string());
        if !real.join("steamapps").is_dir() {
            continue;
        }
        for lib in steam_libraries(&real) {
            let vr = lib.join("steamapps/common/SteamVR");
            if vr.join("bin/vrstartup.sh").exists() {
                let note = format!("Steam at {}, SteamVR in library {}", real.display(), lib.display());
                return (Some(real), Some(vr), note);
            }
        }
        let note = format!("Steam at {}, but SteamVR isn't installed in any of its libraries", real.display());
        return (Some(real), None, note);
    }
    (None, None, format!("Steam not found (looked in {})", tried.join(", ")))
}

/*
 * Status (polled in a background thread).
 */

#[derive(Default, Clone)]
struct Status {
    headset: bool,
    left: Option<bool>,  // None: not paired
    right: Option<bool>,
    steamvr: bool,
    service: bool,
}

fn usb_present(vid: &str, pid: &str) -> bool {
    let Ok(dir) = fs::read_dir("/sys/bus/usb/devices") else { return false };
    dir.flatten().any(|e| {
        let p = e.path();
        let v = fs::read_to_string(p.join("idVendor")).unwrap_or_default();
        let d = fs::read_to_string(p.join("idProduct")).unwrap_or_default();
        v.trim().eq_ignore_ascii_case(vid) && d.trim().eq_ignore_ascii_case(pid)
    })
}

fn bt_controllers() -> (Option<bool>, Option<bool>) {
    let out = Command::new("bluetoothctl").arg("devices").output();
    let Ok(out) = out else { return (None, None) };
    let text = String::from_utf8_lossy(&out.stdout);
    let mut left = None;
    let mut right = None;
    for line in text.lines() {
        // "Device AA:BB:CC:DD:EE:FF PlayStation VR2 Sense Controller (L)"
        let mut parts = line.splitn(3, ' ');
        let (Some(_), Some(mac), Some(name)) = (parts.next(), parts.next(), parts.next()) else { continue };
        if !name.contains("Sense Controller") {
            continue;
        }
        let info = Command::new("bluetoothctl").args(["info", mac]).output();
        let connected = info.map(|o| String::from_utf8_lossy(&o.stdout).contains("Connected: yes")).unwrap_or(false);
        if name.contains("(L)") {
            left = Some(connected);
        } else if name.contains("(R)") {
            right = Some(connected);
        }
    }
    (left, right)
}

/// SteamVR's chaperone file without a room drawn in Room Setup in the saved-room universe (5022): Some(new text),
/// or None if there is none (the driver's default square, marked monado_rectangle, and other universes are kept).
fn without_drawn_room(text: &str) -> Result<Option<String>, String> {
    let mut doc: serde_json::Value = serde_json::from_str(text).map_err(|e| e.to_string())?;
    let universes = doc.get_mut("universes").and_then(|u| u.as_array_mut()).ok_or("no universes")?;
    let before = universes.len();
    universes.retain(|u| {
        let id = match &u["universeID"] {
            serde_json::Value::String(s) => s.clone(),
            other => other.to_string(),
        };
        !(id == "5022" && u.get("monado_rectangle").is_none() && has_drawn_room(u))
    });
    if universes.len() == before {
        return Ok(None);
    }
    serde_json::to_string_pretty(&doc).map(Some).map_err(|e| e.to_string())
}

/// As the driver's has_drawn_room (ovrn_room.cpp): walls, or a play area with a size.
fn has_drawn_room(u: &serde_json::Value) -> bool {
    if u["collision_bounds"].as_array().is_some_and(|b| !b.is_empty()) {
        return true;
    }
    match u["play_area"].as_array() {
        Some(a) if a.len() == 2 => a.iter().all(|x| x.as_f64().is_some_and(|v| v > 0.0)),
        _ => false,
    }
}

#[cfg(test)]
mod compile_tests {
    use super::*;

    /// Tests that read or change HOME take turns.
    static HOME_LOCK: Mutex<()> = Mutex::new(());

    /// The full development build links GStreamer and others Steam's runtime lacks; the container build bundles what
    /// it needs (only where both builds exist, i.e. on the development machine).
    #[test]
    fn container_check_on_real_builds() {
        let _turn = HOME_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let home = std::env::var("HOME").unwrap_or_default();
        let full = Path::new(&home).join("Documents/Claude/Monado/trial/build/steamvr-monado-psvr2");
        let slim = Path::new(&home).join("Documents/Claude/Monado/trial/build-steamvr/steamvr-monado-psvr2");
        if !full.exists() || !slim.exists() || sniper_libs().is_empty() {
            return;
        }
        let m = container_problems(&full).unwrap();
        assert!(m.iter().any(|n| n.starts_with("libgstapp")), "full build: {m:?}");
        assert_eq!(container_problems(&slim).unwrap(), Vec::<String>::new());
        // The same build through a path with ".." in it.
        let via_kit = Path::new(&home).join("Documents/Claude/Monado/psvr2-test/../trial/build-steamvr/steamvr-monado-psvr2");
        assert_eq!(container_problems(&via_kit).unwrap(), Vec::<String>::new());
    }

    #[test]
    fn ninja_progress_lines() {
        assert_eq!(ninja_progress("[12/345] Building CXX object foo.o"), Some((12, 345)));
        assert_eq!(ninja_progress("STEP 2/4 compile"), None);
        assert_eq!(ninja_progress("[x/y] nope"), None);
        let mut b = BuildState { step: 3, done: 50, total: 100, ..Default::default() };
        assert!((b.fraction() - 0.48).abs() < 1e-3);
        b.result = Some(Ok("/x".into()));
        assert_eq!(b.fraction(), 1.0);
    }

    /// Turn on writes the four camera settings (booleans as booleans) and keeps the rest; Undo restores the file.
    #[test]
    fn psvr2_settings_apply_and_undo() {
        let _turn = HOME_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let real_home = std::env::var("HOME").unwrap_or_default();
        let home = std::env::temp_dir().join(format!("dk-settings-{}", std::process::id()));
        fs::create_dir_all(&home).unwrap();
        std::env::set_var("HOME", &home);
        std::env::remove_var("XDG_CONFIG_HOME");
        let file = home.join("steamvr.vrsettings");
        let start = r#"{"camera":{"enableCamera":false,"roomViewStyle":2},"steamvr":{"supersampleScale":1.4}}"#;
        fs::write(&file, start).unwrap();
        apply_psvr2_settings(&file).unwrap();
        let v: serde_json::Value = serde_json::from_str(&fs::read_to_string(&file).unwrap()).unwrap();
        assert_eq!(psvr2_settings_on(&v), 4);
        assert_eq!(v["camera"]["enableCamera"], serde_json::Value::Bool(true));
        assert_eq!(v["camera"]["roomViewStyle"], 4);
        assert_eq!(v["steamvr"]["supersampleScale"], 1.4, "other settings are kept");
        undo_psvr2_settings(&file).unwrap();
        assert_eq!(fs::read_to_string(&file).unwrap(), start);
        let _ = fs::remove_dir_all(&home);
        std::env::set_var("HOME", real_home);
    }

    /// Register replaces Sony's PS VR2 driver with ours and keeps the others; Unregister removes only ours. Run in a
    /// temporary HOME so the real SteamVR config isn't touched.
    #[test]
    fn register_and_unregister() {
        let _turn = HOME_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let real_home = std::env::var("HOME").unwrap_or_default();
        let home = std::env::temp_dir().join(format!("dk-test-{}", std::process::id()));
        let sony = home.join("Steam/steamapps/common/PlayStation VR2 App/SteamVR_Plug-In");
        let other = home.join("other_driver");
        let plugin = home.join("install/build/steamvr-monado-psvr2");
        for d in [&sony, &other, &plugin] {
            fs::create_dir_all(d).unwrap();
        }
        fs::write(plugin.join("driver.vrdrivermanifest"), "{}").unwrap();
        fs::create_dir_all(home.join(".config/openvr")).unwrap();
        let start = serde_json::json!({"config": ["/c"], "external_drivers": [sony.display().to_string(), other.display().to_string()], "runtime": ["/r"], "version": 1});
        fs::write(home.join(".config/openvr/openvrpaths.vrpath"), start.to_string()).unwrap();
        std::env::set_var("HOME", &home);
        std::env::remove_var("XDG_CONFIG_HOME");

        register_driver(&plugin).unwrap();
        let (v, list) = external_drivers().unwrap();
        assert_eq!(list, vec![other.display().to_string(), plugin.display().to_string()]);
        assert_eq!(v["runtime"][0], "/r", "the rest of the file is kept");

        unregister_driver().unwrap();
        let (_, list) = external_drivers().unwrap();
        assert_eq!(list, vec![other.display().to_string()], "Unregister removes only our driver");
        let _ = fs::remove_dir_all(&home);
        std::env::set_var("HOME", real_home);
    }
}

#[cfg(test)]
mod tests {
    use super::without_drawn_room;

    #[test]
    fn removes_a_drawn_room_only() {
        let t = r#"{"jsonid":"chaperone_info","version":5,"universes":[
            {"universeID":"5022","collision_bounds":[[[0,0,0]]],"play_area":[3,2]},
            {"universeID":"5023","monado_rectangle":true,"play_area":[2,2]},
            {"universeID":"9999","play_area":[1,1]}]}"#;
        let out = without_drawn_room(t).unwrap().expect("a drawn room is taken out");
        let v: serde_json::Value = serde_json::from_str(&out).unwrap();
        let ids: Vec<&str> = v["universes"].as_array().unwrap().iter().map(|u| u["universeID"].as_str().unwrap()).collect();
        assert_eq!(ids, vec!["5023", "9999"]);
        assert_eq!(v["jsonid"], "chaperone_info");
    }

    #[test]
    fn keeps_the_default_square() {
        let t = r#"{"jsonid":"chaperone_info","version":5,"universes":[
            {"universeID":"5022","monado_rectangle":true,"play_area":[2,2]},
            {"universeID":"5023","monado_rectangle":true,"play_area":[2,2]}]}"#;
        assert_eq!(without_drawn_room(t).unwrap(), None);
    }

    #[test]
    fn numeric_ids_and_bad_files() {
        // An undrawn, seeded entry (no walls, no play area) isn't a drawn room.
        let seeded = r#"{"universes":[{"universeID":"5022","seated":{},"standing":{},"time":"x"}]}"#;
        assert_eq!(without_drawn_room(seeded).unwrap(), None);
        let t = r#"{"universes":[{"universeID":5022,"play_area":[3,3]}]}"#;
        assert!(without_drawn_room(t).unwrap().is_some());
        assert!(without_drawn_room("not json").is_err());
        assert!(without_drawn_room(r#"{"x":1}"#).is_err());
    }
}

fn process_running(name: &str) -> bool {
    Command::new("pgrep").args(["-x", name]).output().map(|o| o.status.success()).unwrap_or(false)
}

fn poll_status() -> Status {
    let (left, right) = bt_controllers();
    Status {
        headset: usb_present("054c", "0cde"),
        left,
        right,
        steamvr: process_running("vrserver"),
        service: process_running("monado-service"),
    }
}

/*
 * The app.
 */

#[derive(PartialEq, Clone, Copy)]
enum Tab {
    Launcher,
    Settings,
    Compile,
}

struct Launcher {
    cfg: Config,
    tab: Tab,
    status: Arc<Mutex<Status>>,
    steamvr: Option<Child>,
    service: Option<(Child, ChildStdin)>,
    app: Option<Child>,
    app_path: String,
    app_args: String,
    message: String,
    /// Set Custom Play Area: the "put on your headset" prompt, while shown.
    headset_prompt: bool,
    saved_at: Option<Instant>,
    build: Arc<Mutex<BuildState>>,
    /// The Compile tab's Check: is the installed build the latest on GitHub?
    check: Arc<Mutex<CheckState>>,
    /// --screenshot <file.png> [launcher|settings|compile]: render one tab, save it, quit (for checking the design).
    screenshot: Option<(PathBuf, Instant, bool)>,
    /// The device pictures (assets/): headset, left and right Sense controller.
    pictures: [Option<egui::TextureHandle>; 3],
    /// registrable_plugin's last answer and when (ldd isn't run on every redraw).
    reg_cache: Option<(Instant, Option<PathBuf>)>,
}

fn stamp() -> String {
    Command::new("date")
        .arg("+%Y%m%d-%H%M%S")
        .output()
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .unwrap_or_else(|_| "now".into())
}

fn signal(child: &Child, sig: &str) {
    let _ = Command::new("kill").args([sig, &child.id().to_string()]).status();
}

impl Launcher {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        theme::install(&cc.egui_ctx);
        let status = Arc::new(Mutex::new(Status::default()));
        let s2 = status.clone();
        let ctx = cc.egui_ctx.clone();
        thread::spawn(move || loop {
            let st = poll_status();
            *s2.lock().unwrap() = st;
            ctx.request_repaint();
            thread::sleep(Duration::from_secs(1));
        });
        Launcher {
            cfg: load_config(),
            tab: Tab::Launcher,
            status,
            steamvr: None,
            service: None,
            app: None,
            app_path: String::new(),
            app_args: String::new(),
            message: String::new(),
            headset_prompt: false,
            saved_at: None,
            build: Arc::new(Mutex::new(BuildState::default())),
            check: Arc::new(Mutex::new(CheckState::default())),
            screenshot: None,
            reg_cache: None,
            pictures: [
                theme::texture(&cc.egui_ctx, "headset", include_bytes!("../assets/headset.png")),
                theme::texture(&cc.egui_ctx, "controller_left", include_bytes!("../assets/controller_left.png")),
                theme::texture(&cc.egui_ctx, "controller_right", include_bytes!("../assets/controller_right.png")),
            ],
        }
    }

    fn logs_dir(&self) -> PathBuf {
        let d = Path::new(&self.cfg.test_kit).join("logs");
        let _ = fs::create_dir_all(&d);
        d
    }

    fn start_steamvr(&mut self) {
        self.start_steamvr_with(&[]);
    }

    fn start_steamvr_with(&mut self, extra_env: &[(&str, &str)]) {
        let status = self.status.lock().unwrap().clone();
        if status.service {
            self.message = "Stop monado-service first: it holds the headset.".into();
            return;
        }
        let script = Path::new(&self.cfg.test_kit).join("launch-steamvr.sh");
        let log = self.logs_dir().join(format!("launcher-steamvr-{}.log", stamp()));
        let Ok(f) = fs::File::create(&log) else {
            self.message = format!("Can't write {}", log.display());
            return;
        };
        let f2 = f.try_clone().ok();
        let mut cmd = Command::new("bash");
        cmd.arg(&script).envs(self.cfg.env(Target::SteamVR)).stdin(Stdio::null()).stdout(f);
        // The driver this app uses (the Compile Driver tab's build, or the Monado build in Settings).
        cmd.env("MONADO_BUILD", &self.cfg.monado_build);
        cmd.envs(extra_env.iter().map(|(k, v)| (k.to_string(), v.to_string())));
        if !self.cfg.steam_root.is_empty() {
            cmd.env("STEAM_ROOT", &self.cfg.steam_root);
        }
        if !self.cfg.steamvr_dir.is_empty() {
            cmd.env("STEAMVR_DIR", &self.cfg.steamvr_dir);
        }
        if let Some(f2) = f2 {
            cmd.stderr(f2);
        }
        match cmd.spawn() {
            Ok(c) => {
                self.steamvr = Some(c);
                self.message = format!("SteamVR starting (log {})", log.display());
            }
            Err(e) => self.message = format!("Couldn't start {}: {e}", script.display()),
        }
    }

    /// SteamVR's chaperone file (<SteamVR config>/chaperone_info.vrchap), where the driver keeps its rooms (universes
    /// 5022 and 5023) since 2026-11-19: SteamVR couldn't save into the driver's own file (monado_psvr2/...), which the
    /// driver moves over once. SteamVR's config folder from ~/.config/openvr/openvrpaths.vrpath as the driver finds it.
    fn chaperone_file(&self) -> PathBuf {
        let home = std::env::var("HOME").unwrap_or_default();
        let vrpath = Path::new(&home).join(".config/openvr/openvrpaths.vrpath");
        let from_vrpath = fs::read_to_string(&vrpath)
            .ok()
            .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok())
            .and_then(|v| v["config"].get(0).and_then(|c| c.as_str()).map(PathBuf::from));
        let config = from_vrpath.unwrap_or_else(|| {
            let root = if self.cfg.steam_root.is_empty() {
                Path::new(&home).join(".local/share/Steam")
            } else {
                PathBuf::from(&self.cfg.steam_root)
            };
            root.join("config")
        });
        config.join("chaperone_info.vrchap")
    }

    /// Set Default Play Area: take a room drawn in Room Setup out of the driver's saved-room universe (5022,
    /// kRoomUniverse in ovrn_backend_monado.cpp), so the next start gives SteamVR the default square from the Settings
    /// tab again (the driver writes it into an undrawn room). The file is backed up into logs/ first; the floor anchor
    /// (~/.config/monado/ovrd_room.json) is kept. Only while SteamVR is closed: it owns the file while running.
    fn set_default_play_area(&mut self) {
        let size = self
            .cfg
            .settings
            .iter()
            .find(|s| s.key == "PLAY_AREA")
            .and_then(|s| match &s.kind {
                Kind::Value { value } if s.enabled => Some(value.trim().to_string()),
                _ => None,
            })
            .unwrap_or_else(|| "2".into());
        let path = self.chaperone_file();
        let Ok(text) = fs::read_to_string(&path) else {
            self.message = format!("No room drawn yet: SteamVR gets the default {size} m play area ({} not there).", path.display());
            return;
        };
        let new_text = match without_drawn_room(&text) {
            Err(e) => {
                self.message = format!("Can't read {}: {e}", path.display());
                return;
            }
            Ok(None) => {
                self.message = format!("Already the default play area ({size} m): no room drawn in Room Setup.");
                return;
            }
            Ok(Some(t)) => t,
        };
        let backup = self.logs_dir().join(format!("chaperone-before-default-{}.vrchap", stamp()));
        if fs::write(&backup, &text).is_err() {
            self.message = format!("Can't back up {} (nothing changed)", path.display());
            return;
        }
        match fs::write(&path, new_text) {
            Ok(()) => {
                self.message = format!(
                    "Default play area set: the next SteamVR start uses the {size} m square from Settings (your drawn room is backed up in {}).",
                    backup.display()
                )
            }
            Err(_) => self.message = format!("Couldn't write {}", path.display()),
        }
    }

    /// Set Custom Play Area: SteamVR's in-headset Room Setup (as on Steam Frame: floor, then trace the play area).
    /// Only SteamVR's dashboard can start it (vrcmd --resetroomsetup only reset and recommitted the room, 2026-11-19
    /// 22:05), so the player gets the steps to open it there; SteamVR is started first if it isn't running.
    fn set_custom_play_area(&mut self, steamvr_running: bool) {
        self.headset_prompt = true;
        if !steamvr_running {
            self.start_steamvr_with(&[]);
            if self.steamvr.is_some() {
                self.message = "SteamVR starting; then open Room Setup in the headset (steps on screen).".into();
            }
        } else {
            self.message = "Open Room Setup in the headset (steps on screen).".into();
        }
    }

    /// Set Custom Play Area (legacy): SteamVR's desktop Room Setup tool. SteamVR running: asked to open it
    /// (tools/vrlaunch, a background client); not running: started with Room Setup opening (ROOM_SETUP=1).
    fn set_custom_play_area_legacy(&mut self, steamvr_running: bool) {
        if !steamvr_running {
            self.start_steamvr_with(&[("ROOM_SETUP", "1")]);
            if self.steamvr.is_some() {
                self.message = "SteamVR starting; Room Setup opens once it is up.".into();
            }
            return;
        }
        let tool = Path::new(&self.cfg.test_kit).join("tools/vrlaunch");
        if !tool.exists() {
            self.message = format!("{} isn't built (see its Build: line); open Room Setup from SteamVR", tool.display());
            return;
        }
        match Command::new(&tool).arg("openvr.tool.steamvr_room_setup").stdin(Stdio::null()).spawn() {
            Ok(_) => self.message = "Opening Room Setup in SteamVR...".into(),
            Err(e) => self.message = format!("Couldn't run {}: {e}", tool.display()),
        }
    }

    fn stop_steamvr(&mut self) {
        if let Some(c) = &self.steamvr {
            // The script traps Ctrl+C: stops SteamVR, restores the OpenVR config, keeps the logs.
            signal(c, "-INT");
        } else {
            let _ = Command::new("pkill").args(["-INT", "-x", "vrmonitor"]).status();
            let _ = Command::new("pkill").args(["-INT", "-x", "vrserver"]).status();
        }
        self.message = "Stopping SteamVR...".into();
    }

    fn start_service(&mut self) -> bool {
        let status = self.status.lock().unwrap().clone();
        if status.steamvr {
            self.message = "Stop SteamVR first: it holds the headset.".into();
            return false;
        }
        if self.service.is_some() || status.service {
            return true;
        }
        let bin = Path::new(&self.cfg.monado_build).join("src/xrt/targets/service/monado-service");
        let log = self.logs_dir().join(format!("monado-{}.log", stamp()));
        let Ok(f) = fs::File::create(&log) else {
            self.message = format!("Can't write {}", log.display());
            return false;
        };
        let runtime = std::env::var("XDG_RUNTIME_DIR").unwrap_or_else(|_| "/tmp".into());
        let _ = fs::remove_file(Path::new(&runtime).join("monado_comp_ipc")); // a stale socket blocks a new service
        let f2 = f.try_clone().ok();
        let mut cmd = Command::new(&bin);
        cmd.envs(self.cfg.env(Target::Monado)).env("XRT_LOG", "info").stdin(Stdio::piped()).stdout(f);
        if let Some(f2) = f2 {
            cmd.stderr(f2);
        }
        match cmd.spawn() {
            Ok(mut c) => {
                // monado-service quits when its stdin closes: keep it open for as long as it runs.
                let stdin = c.stdin.take().expect("piped stdin");
                self.service = Some((c, stdin));
                self.message = format!("monado-service starting (log {})", log.display());
                true
            }
            Err(e) => {
                self.message = format!("Couldn't start {}: {e}", bin.display());
                false
            }
        }
    }

    fn stop_service(&mut self) {
        if let Some((c, mut stdin)) = self.service.take() {
            let _ = stdin.write_all(b"\n"); // the service's own "press enter to quit"
            drop(stdin);
            signal(&c, "-INT");
            let mut c = c;
            thread::spawn(move || {
                for _ in 0..50 {
                    if let Ok(Some(_)) = c.try_wait() {
                        return;
                    }
                    thread::sleep(Duration::from_millis(100));
                }
                let _ = c.kill();
                let _ = c.wait();
            });
        } else {
            let _ = Command::new("pkill").args(["-INT", "-x", "monado-service"]).status();
        }
        self.message = "Stopping monado-service...".into();
    }

    fn launch_app(&mut self) {
        let path = self.app_path.trim().to_string();
        if path.is_empty() || !Path::new(&path).exists() {
            self.message = "Pick an app first.".into();
            return;
        }
        if path.to_lowercase().ends_with(".exe") {
            self.message = "Windows apps need Proton: run them from Steam with SteamVR (or with the variables below).".into();
            return;
        }
        if !self.start_service() {
            return;
        }
        // Wait for the service's socket (up to 20 s), off the UI thread.
        let runtime = std::env::var("XDG_RUNTIME_DIR").unwrap_or_else(|_| "/tmp".into());
        let sock = Path::new(&runtime).join("monado_comp_ipc");
        let t0 = Instant::now();
        while !sock.exists() && t0.elapsed() < Duration::from_secs(20) {
            thread::sleep(Duration::from_millis(200));
        }
        if !sock.exists() {
            self.message = "monado-service didn't come up within 20 s (see its log).".into();
            return;
        }
        let args: Vec<&str> = self.app_args.split_whitespace().collect();
        let dir = Path::new(&path).parent().map(Path::to_path_buf).unwrap_or_else(|| PathBuf::from("."));
        match Command::new(&path).args(&args).current_dir(dir).env("XR_RUNTIME_JSON", self.cfg.manifest()).spawn() {
            Ok(c) => {
                self.app = Some(c);
                self.message = format!("Running {path}");
            }
            Err(e) => self.message = format!("Couldn't start {path}: {e}"),
        }
    }

    fn stop_app(&mut self) {
        if let Some(c) = &self.app {
            signal(c, "-TERM");
        }
        self.message = "Stopping the app...".into();
    }

    fn reap(&mut self) {
        if let Some(c) = &mut self.steamvr {
            if let Ok(Some(_)) = c.try_wait() {
                self.steamvr = None;
            }
        }
        if let Some((c, _)) = &mut self.service {
            if let Ok(Some(_)) = c.try_wait() {
                self.service = None;
            }
        }
        if let Some(c) = &mut self.app {
            if let Ok(Some(_)) = c.try_wait() {
                self.app = None;
            }
        }
    }
}

/*
 * Compile Driver tab: build-driver.sh, registration with SteamVR, package lists.
 */

/// A running or finished compile (build-driver.sh's output, read on a thread).
/// What Check found (see check_build).
#[derive(Default)]
struct CheckState {
    running: bool,
    result: Option<CheckResult>,
}

#[derive(Clone)]
enum CheckResult {
    NotInstalled(String),
    UpToDate(String),
    NewBuild(String),
    Failed(String),
}

/// The build installed in @p dest: (commit, commit date) as build-driver.sh recorded it, or the source folder's commit
/// for installs made before it recorded one. None: nothing installed there.
fn installed_build(dest: &Path) -> Option<(String, String)> {
    if !dest.join("build/steamvr-monado-psvr2/driver.vrdrivermanifest").exists() {
        return None;
    }
    let env = driver_env();
    let get = |k: &str| env.iter().find(|(key, _)| key == k).map(|(_, v)| v.clone());
    let same = get("PSVR2_DRIVERKIT_HOME").map(|h| Path::new(&h).canonicalize().ok() == dest.canonicalize().ok()).unwrap_or(false);
    if same {
        if let Some(c) = get("SOURCE_COMMIT").filter(|c| !c.is_empty()) {
            return Some((c, get("SOURCE_DATE").unwrap_or_default()));
        }
    }
    let git = |args: &[&str]| {
        Command::new("git").arg("-C").arg(dest.join("source")).args(args).output().ok().filter(|o| o.status.success()).map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
    };
    let c = git(&["rev-parse", "HEAD"])?;
    Some((c, git(&["log", "-1", "--format=%cI"]).unwrap_or_default()))
}

/// The latest commit on GitHub's main branch: (commit, date, first line of its message). Asks the GitHub CLI when it's
/// signed in (the repository is private), else git with your own credentials, never prompting.
fn latest_on_github() -> Result<(String, String, String), String> {
    let gh = Command::new("gh")
        .args(["api", &format!("repos/{}/commits/main", REPO_WEB.trim_start_matches("github.com/")), "--jq", ".sha, .commit.committer.date, (.commit.message | split(\"\\n\") | .[0])"])
        .stdin(Stdio::null())
        .output();
    if let Ok(o) = &gh {
        if o.status.success() {
            let t = String::from_utf8_lossy(&o.stdout);
            let mut l = t.lines();
            if let (Some(c), Some(d)) = (l.next(), l.next()) {
                return Ok((c.trim().to_string(), d.trim().to_string(), l.next().unwrap_or("").trim().to_string()));
            }
        }
    }
    let o = Command::new("git")
        .args(["ls-remote", &format!("https://{REPO_WEB}.git"), "refs/heads/main"])
        .env("GIT_TERMINAL_PROMPT", "0")
        .stdin(Stdio::null())
        .output()
        .map_err(|e| format!("couldn't run git: {e}"))?;
    let t = String::from_utf8_lossy(&o.stdout);
    match t.split_whitespace().next() {
        Some(c) if o.status.success() && c.len() >= 7 => Ok((c.to_string(), String::new(), String::new())),
        _ => Err("couldn't reach GitHub: sign in first (gh auth login) or check your connection".into()),
    }
}

/// "2026-11-19 14:32" in local time from an ISO date (GitHub's are UTC, git's carry their offset).
fn short_date(iso: &str) -> String {
    if iso.is_empty() {
        return String::new();
    }
    Command::new("date")
        .args(["-d", iso, "+%Y-%m-%d %H:%M"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .unwrap_or_else(|| iso.to_string())
}

fn check_build(dest: &Path) -> CheckResult {
    let installed = installed_build(dest);
    let latest = latest_on_github();
    match (installed, latest) {
        (None, Ok((c, d, m))) => CheckResult::NotInstalled(format!(
            "Not installed in {}. Latest on GitHub: {} ({}){}. Press Compile Driver to install it.",
            dest.display(), &c[..7.min(c.len())], short_date(&d), if m.is_empty() { String::new() } else { format!(", \"{m}\"") })),
        (None, Err(e)) => CheckResult::NotInstalled(format!("Not installed in {}. ({e})", dest.display())),
        (Some(_), Err(e)) => CheckResult::Failed(e),
        (Some((ic, id)), Ok((c, d, m))) => {
            if ic == c {
                CheckResult::UpToDate(format!("Up to date: build {} ({}) is the latest on GitHub.", &ic[..7.min(ic.len())], short_date(&id)))
            } else {
                CheckResult::NewBuild(format!(
                    "New build available: installed {} ({}), GitHub has {} ({}){}. Press Compile Driver to update.",
                    &ic[..7.min(ic.len())], short_date(&id), &c[..7.min(c.len())], short_date(&d),
                    if m.is_empty() { String::new() } else { format!(", \"{m}\"") }))
            }
        }
    }
}

#[derive(Default)]
struct BuildState {
    running: bool,
    /// build-driver.sh's "STEP n/4"
    step: u32,
    /// ninja's [done/total]
    done: u32,
    total: u32,
    lines: Vec<String>,
    /// Ok(install folder) or Err(why)
    result: Option<Result<String, String>>,
}

impl BuildState {
    /// 0..1 over the four steps, the compile step by ninja's count.
    fn fraction(&self) -> f32 {
        let within = if self.step == 3 && self.total > 0 { self.done as f32 / self.total as f32 } else { 0.0 };
        if self.result.as_ref().is_some_and(|r| r.is_ok()) {
            return 1.0;
        }
        // download, configure, DriverKit and record are short next to the compile, which gets most of the bar
        match self.step {
            0 => 0.0,
            1 => 0.03,
            2 => 0.06,
            3 => 0.08 + 0.80 * within,
            4 => 0.92,
            _ => 0.98,
        }
    }
}

/// "[123/456] ..." -> (123, 456)
fn ninja_progress(line: &str) -> Option<(u32, u32)> {
    let rest = line.strip_prefix('[')?;
    let (nums, _) = rest.split_once(']')?;
    let (a, b) = nums.split_once('/')?;
    Some((a.trim().parse().ok()?, b.trim().parse().ok()?))
}

fn openvrpaths_file() -> PathBuf {
    PathBuf::from(std::env::var("HOME").unwrap_or_default()).join(".config/openvr/openvrpaths.vrpath")
}

/// SteamVR's external drivers (openvrpaths.vrpath "external_drivers").
fn external_drivers() -> Result<(serde_json::Value, Vec<String>), String> {
    let text = fs::read_to_string(openvrpaths_file()).map_err(|e| format!("{}: {e}", openvrpaths_file().display()))?;
    let v: serde_json::Value = serde_json::from_str(&text).map_err(|e| e.to_string())?;
    let list = v["external_drivers"]
        .as_array()
        .map(|a| a.iter().filter_map(|x| x.as_str().map(str::to_string)).collect())
        .unwrap_or_default();
    Ok((v, list))
}

fn write_external_drivers(mut v: serde_json::Value, list: &[String]) -> Result<(), String> {
    v["external_drivers"] = serde_json::Value::from(list.to_vec());
    let text = serde_json::to_string_pretty(&v).map_err(|e| e.to_string())?;
    fs::write(openvrpaths_file(), text).map_err(|e| e.to_string())
}

/// Sony's PS VR2 plugin, or another copy of ours: two PS VR2 headset drivers can't both be registered.
fn is_psvr2_driver(path: &str) -> bool {
    path.contains("PlayStation VR2 App") || path.ends_with("steamvr-monado-psvr2") || path.ends_with("steamvr-monado-psvr2/")
}

/// Libraries Steam's runtime container never takes from a driver's folder: the C runtime and the host's graphics and
/// system libraries, which pressure-vessel brings in itself (bundle_libs.py's NEVER list).
const HOST_LIBS: [&str; 22] = [
    "linux-vdso.so", "ld-linux", "libc.so", "libm.so", "libmvec.so", "libdl.so", "libpthread.so", "librt.so",
    "libresolv.so", "libutil.so", "libstdc++.so", "libgcc_s.so", "libvulkan.so", "libGL", "libEGL.so", "libGLX",
    "libGLdispatch.so", "libdrm.so", "libgbm.so", "libnvidia", "libudev.so", "libsystemd.so",
];

/// Library names Steam's sniper runtime provides (its files/lib/x86_64-linux-gnu), from the newest sniper platform.
fn sniper_libs() -> std::collections::HashSet<String> {
    let home = std::env::var("HOME").unwrap_or_default();
    let base = Path::new(&home).join(".local/share/Steam/steamapps/common/SteamLinuxRuntime_sniper");
    let mut platforms: Vec<PathBuf> = fs::read_dir(&base)
        .map(|d| d.filter_map(|e| e.ok().map(|e| e.path())).filter(|p| p.file_name().is_some_and(|n| n.to_string_lossy().starts_with("sniper_platform_"))).collect())
        .unwrap_or_default();
    platforms.sort();
    let mut names = std::collections::HashSet::new();
    if let Some(p) = platforms.last() {
        if let Ok(d) = fs::read_dir(p.join("files/lib/x86_64-linux-gnu")) {
            names.extend(d.filter_map(|e| e.ok().map(|e| e.file_name().to_string_lossy().to_string())));
        }
        // The runtime keeps its files by content hash; usr-mtree.txt.gz is its index ("./lib/x86_64-linux-gnu/<name>").
        if let Ok(out) = Command::new("zcat").arg(p.join("usr-mtree.txt.gz")).output() {
            for line in String::from_utf8_lossy(&out.stdout).lines() {
                let path = line.split(' ').next().unwrap_or("");
                if path.contains("/x86_64-linux-gnu/") && path.contains(".so") {
                    if let Some(n) = path.rsplit('/').next() {
                        names.insert(n.to_string());
                    }
                }
            }
        }
    }
    names
}

/// Why SteamVR started by Steam (inside its runtime container) couldn't load this driver: the libraries it needs that
/// are neither next to it, nor in Steam's runtime, nor ones the container takes from the host. Empty: it can.
fn container_problems(plugin: &Path) -> Result<Vec<String>, String> {
    let so = plugin.join("bin/linux64/driver_monado_psvr2.so");
    if !so.exists() {
        return Err(format!("no driver at {}", so.display()));
    }
    let out = Command::new("ldd").arg(&so).output().map_err(|e| format!("ldd: {e}"))?;
    let runtime = sniper_libs();
    if runtime.is_empty() {
        return Err("Steam's runtime (SteamLinuxRuntime_sniper) isn't installed: start SteamVR once from Steam".into());
    }
    // ldd prints resolved paths: compare with the resolved folder (a path given as kit/../trial/... wouldn't match).
    let folder = so.parent().and_then(|f| f.canonicalize().ok()).unwrap_or_default();
    let mut missing = vec![];
    for line in String::from_utf8_lossy(&out.stdout).lines() {
        let parts: Vec<&str> = line.split_whitespace().collect();
        let (name, path) = match parts.as_slice() {
            [n, "=>", p, ..] => (*n, *p),
            _ => continue,
        };
        if HOST_LIBS.iter().any(|h| name.starts_with(h)) || runtime.contains(name) || Path::new(path).starts_with(&folder) {
            continue;
        }
        missing.push(name.to_string());
    }
    Ok(missing)
}

/// Registered: our plugin in SteamVR's driver list (Steam starts SteamVR with it, as with Sony's). The PS VR2
/// entries it takes out (two drivers can't both run the headset) are named in the message, not saved.
fn register_driver(plugin: &Path) -> Result<String, String> {
    if !plugin.join("driver.vrdrivermanifest").exists() {
        return Err(format!("no plugin at {} (compile first)", plugin.display()));
    }
    let (v, list) = external_drivers()?;
    let ours = plugin.display().to_string();
    let replaced: Vec<String> = list.iter().filter(|p| is_psvr2_driver(p) && **p != ours).cloned().collect();
    let mut new: Vec<String> = list.into_iter().filter(|p| !is_psvr2_driver(p)).collect();
    new.push(ours.clone());
    write_external_drivers(v, &new)?;
    Ok(if replaced.is_empty() {
        format!("Registered {ours} with SteamVR.")
    } else {
        format!("Registered {ours} with SteamVR; taken off SteamVR's list: {} (two PS VR2 drivers can't both run the headset).", replaced.join(", "))
    })
}

/// Unregister: our plugin out of SteamVR's list, the PS VR2 driver it replaced (Sony's) back.
fn unregister_driver() -> Result<String, String> {
    let (v, list) = external_drivers()?;
    let new: Vec<String> = list.into_iter().filter(|p| !p.trim_end_matches('/').ends_with("steamvr-monado-psvr2")).collect();
    write_external_drivers(v, &new)?;
    Ok("Our driver unregistered from SteamVR (the other drivers on the list are left as they are).".to_string())
}


/// The SteamVR settings DriverKit turns on for the PS VR2 (section, key, value): the camera on, Room View available
/// as the full camera picture, and the camera shown near the play-area walls.
const PSVR2_STEAMVR_SETTINGS: [(&str, &str, i64); 4] = [
    ("camera", "enableCamera", 1),
    ("camera", "roomView", 1),
    ("camera", "roomViewStyle", 4),
    ("camera", "enableCameraForCollisionBounds", 1),
];

/// enableCamera and enableCameraForCollisionBounds are booleans in SteamVR's settings; the others numbers.
fn setting_value(key: &str, v: i64) -> serde_json::Value {
    if key.starts_with("enable") {
        serde_json::Value::from(v != 0)
    } else {
        serde_json::Value::from(v)
    }
}

/// SteamVR's settings file (<SteamVR config>/steamvr.vrsettings, the config folder from openvrpaths.vrpath).
fn steamvr_settings_file(steam_root: &str) -> PathBuf {
    let home = std::env::var("HOME").unwrap_or_default();
    let from_vrpath = fs::read_to_string(openvrpaths_file())
        .ok()
        .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok())
        .and_then(|v| v["config"].get(0).and_then(|c| c.as_str()).map(PathBuf::from));
    from_vrpath
        .unwrap_or_else(|| {
            if steam_root.is_empty() {
                Path::new(&home).join(".local/share/Steam/config")
            } else {
                Path::new(steam_root).join("config")
            }
        })
        .join("steamvr.vrsettings")
}

/// How many of the PS VR2 settings are already on in this settings JSON.
fn psvr2_settings_on(v: &serde_json::Value) -> usize {
    PSVR2_STEAMVR_SETTINGS.iter().filter(|(sec, key, val)| v[*sec][*key] == setting_value(key, *val)).count()
}

/// Turn the PS VR2 settings on in SteamVR's settings (SteamVR closed). The file is backed up first, once, into
/// ~/.config/PSVR2-DriverKit/steamvr.vrsettings.before-psvr2 (kept for Undo; a later run keeps the first backup).
fn apply_psvr2_settings(file: &Path) -> Result<String, String> {
    let text = fs::read_to_string(file).map_err(|e| format!("{}: {e}", file.display()))?;
    let mut v: serde_json::Value = serde_json::from_str(&text).map_err(|e| format!("{}: {e}", file.display()))?;
    let backup = config_dir().join("steamvr.vrsettings.before-psvr2");
    if !backup.exists() {
        let _ = fs::create_dir_all(config_dir());
        fs::write(&backup, &text).map_err(|e| format!("backup {}: {e}", backup.display()))?;
    }
    for (sec, key, val) in PSVR2_STEAMVR_SETTINGS {
        if !v[sec].is_object() {
            v[sec] = serde_json::json!({});
        }
        v[sec][key] = setting_value(key, val);
    }
    fs::write(file, serde_json::to_string_pretty(&v).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;
    Ok(format!("SteamVR's camera and Room View are on for the PS VR2 (backup: {}).", backup.display()))
}

/// Undo: SteamVR's settings file back from DriverKit's backup (SteamVR closed).
fn undo_psvr2_settings(file: &Path) -> Result<String, String> {
    let backup = config_dir().join("steamvr.vrsettings.before-psvr2");
    let text = fs::read_to_string(&backup).map_err(|_| "no backup to restore (the features were never turned on from here)".to_string())?;
    fs::write(file, text).map_err(|e| e.to_string())?;
    let _ = fs::remove_file(&backup);
    Ok("SteamVR's settings are back as they were before Turn on PS VR2 features.".to_string())
}

/// The packages build-driver.sh needs (compilers, CMake/Ninja, the Monado build's libraries, Rust for DriverKit).
const PACKAGES: [(&str, &str); 3] = [
    ("Arch / CachyOS / Manjaro",
     "sudo pacman -S --needed base-devel git cmake ninja python pkgconf eigen vulkan-headers vulkan-icd-loader glslang shaderc libusb systemd-libs libjpeg-turbo ceres-solver suitesparse google-glog gflags jsoncpp libx11 libxcb libxrandr wayland wayland-protocols libdrm zlib rust"),
    ("Debian / Ubuntu",
     "sudo apt install build-essential git cmake ninja-build python3 pkg-config libeigen3-dev libvulkan-dev glslang-tools glslc libusb-1.0-0-dev libudev-dev libjpeg-dev libceres-dev libsuitesparse-dev libgoogle-glog-dev libgflags-dev libjsoncpp-dev libx11-dev libx11-xcb-dev libxcb-randr0-dev libxrandr-dev libwayland-dev wayland-protocols libdrm-dev zlib1g-dev cargo"),
    ("Fedora",
     "sudo dnf install gcc-c++ git cmake ninja-build python3 pkgconf-pkg-config eigen3-devel vulkan-loader-devel vulkan-headers glslang glslc libusb1-devel systemd-devel libjpeg-turbo-devel ceres-solver-devel suitesparse-devel glog-devel gflags-devel jsoncpp-devel libX11-devel libxcb-devel libXrandr-devel wayland-devel wayland-protocols-devel libdrm-devel zlib-devel cargo"),
];

/// A command or environment line to paste into a terminal, with its Copy button.
/// This user's runtime folder: XDG_RUNTIME_DIR, else /run/user/<their user id> (from /proc/self/status).
fn runtime_dir() -> String {
    if let Ok(d) = std::env::var("XDG_RUNTIME_DIR") {
        if !d.is_empty() {
            return d;
        }
    }
    let uid = fs::read_to_string("/proc/self/status")
        .ok()
        .and_then(|t| t.lines().find(|l| l.starts_with("Uid:")).and_then(|l| l.split_whitespace().nth(1).map(String::from)))
        .unwrap_or_else(|| "0".into());
    format!("/run/user/{uid}")
}

/// A command with a Copy button; an empty command shows a blank row (nothing compiled yet).
fn copy_row(ui: &mut egui::Ui, text: &str) {
    egui::Frame::new()
        .fill(theme::FIELD)
        .corner_radius(egui::CornerRadius::same(8))
        .inner_margin(egui::Margin::symmetric(10, 6))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(!text.is_empty(), egui::Button::new("Copy"))
                    .on_hover_text("Copy to the clipboard")
                    .on_disabled_hover_text("Compile the driver first (Compile Driver tab): the command appears once it's built.")
                    .clicked()
                {
                    ui.ctx().copy_text(text.to_string());
                }
                ui.add(egui::Label::new(egui::RichText::new(text).monospace().color(theme::GRAPHITE)).wrap());
            });
        });
}

impl eframe::App for Launcher {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.reap();
        self.take_screenshot(ui.ctx());
        theme::paint_background(ui);
        let st = self.status.lock().unwrap().clone();
        if self.headset_prompt {
            let mut open = true;
            egui::Window::new("Put on your headset")
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, 0.0))
                .show(ui.ctx(), |ui| {
                    ui.label(egui::RichText::new("Open Room Setup in the headset").font(theme::bold(16.0)).color(theme::GRAPHITE));
                    ui.add_space(6.0);
                    for step in [
                        "1. Put on your headset and pick up both controllers.",
                        "2. Press the PS button to open the SteamVR dashboard.",
                        "3. Select the Playspace button, right of the ☰ menu at the bottom.",
                        "4. Choose Playspace Setup, then follow the steps: set the floor, then trace the edge of your play area.",
                    ] {
                        ui.label(step);
                    }
                    ui.add_space(4.0);
                    ui.label(egui::RichText::new("Only the floor height? Choose Adjust Floor Height in step 4 instead.").color(theme::SLATE).size(12.0));
                    ui.add_space(10.0);
                    ui.label(egui::RichText::new("Keep the room for next time: anchor it").font(theme::bold(14.0)).color(theme::GRAPHITE));
                    ui.add_space(4.0);
                    for step in [
                        "5. Pick a spot on the floor you can find again, for example a piece of tape: your floor mark.",
                        "6. Put the left controller on the mark, crouch next to it so the headset sees it, and hold the Create button for about 3 seconds until it buzzes.",
                    ] {
                        ui.label(step);
                    }
                    ui.add_space(4.0);
                    ui.label(
                        egui::RichText::new(
                            "Your room is now saved. Each time you start SteamVR, do step 6 on the same mark to bring it back; until then you play without a saved room.",
                        )
                        .color(theme::SLATE)
                        .size(12.0),
                    );
                    ui.add_space(10.0);
                    if ui.add(theme::primary("OK")).clicked() {
                        open = false;
                    }
                });
            if !open {
                self.headset_prompt = false;
            }
        }
        egui::Panel::top("header")
            .frame(egui::Frame::new().inner_margin(egui::Margin::symmetric(20, 16)))
            .show_inside(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.vertical(|ui| {
                        ui.add_space(8.0);
                        ui.label(egui::RichText::new("PSVR2-DriverKit").font(theme::bold(24.0)).color(theme::GRAPHITE));
                        ui.label(egui::RichText::new("PlayStation VR2 on Linux").color(theme::SLATE).size(13.0));
                    });
                    ui.add_space(24.0);
                    // The three tiles share the rest of the row evenly.
                    let gap = ui.spacing().item_spacing.x;
                    let w = ((ui.available_width() - 2.0 * gap) / 3.0).floor();
                    theme::device(ui, w, self.pictures[0].as_ref(), Some(st.headset), "Headset");
                    theme::device(ui, w, self.pictures[1].as_ref(), st.left, "Left controller");
                    theme::device(ui, w, self.pictures[2].as_ref(), st.right, "Right controller");
                });
                ui.add_space(10.0);
                ui.horizontal(|ui| {
                    for (tab, label) in [(Tab::Launcher, "Launcher"), (Tab::Settings, "Settings"), (Tab::Compile, "Compile Driver")] {
                        let on = self.tab == tab;
                        let text = egui::RichText::new(label).font(theme::bold(14.0)).color(if on { theme::NAVY } else { theme::GRAPHITE });
                        let b = egui::Button::new(text)
                            .fill(if on { egui::Color32::WHITE } else { egui::Color32::TRANSPARENT })
                            .stroke(egui::Stroke::new(1.0, if on { egui::Color32::WHITE } else { theme::VISOR }))
                            .corner_radius(egui::CornerRadius::same(15));
                        if ui.add(b).clicked() {
                            self.tab = tab;
                        }
                    }
                });
            });
        egui::Panel::bottom("message")
            .frame(egui::Frame::new().fill(egui::Color32::from_rgba_premultiplied(0x02, 0x05, 0x28, 0xC0)).inner_margin(egui::Margin::symmetric(20, 8)))
            .show_inside(ui, |ui| {
                let text = if self.message.is_empty() { "Ready." } else { self.message.as_str() };
                ui.label(egui::RichText::new(text).color(theme::SLATE).size(13.0));
            });
        egui::CentralPanel::default()
            .frame(egui::Frame::new().inner_margin(egui::Margin::symmetric(20, 8)))
            .show_inside(ui, |ui| {
                egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| match self.tab {
                    Tab::Launcher => self.launcher_tab(ui, &st),
                    Tab::Settings => self.settings_tab(ui),
                    Tab::Compile => self.compile_tab(ui, &st),
                });
            });
    }
}

/// The headset refresh rates the PS VR2 panel supports (PSVR2_REFRESH_HZ values) and their labels.
const REFRESH_RATES: [(&str, &str); 2] = [("89.91", "90 Hz"), ("119.88", "120 Hz")];

impl Launcher {
    /// Headset refresh rate (PSVR2_REFRESH_HZ, the same setting as on the Settings tab): used by the next SteamVR,
    /// monado-service or app start; SteamVR and Monado pick the matching display mode.
    fn refresh_picker(&mut self, ui: &mut egui::Ui) {
        let Some(i) = self.cfg.settings.iter().position(|s| s.key == "PSVR2_REFRESH_HZ") else {
            return;
        };
        let current = match &self.cfg.settings[i].kind {
            Kind::Value { value } if self.cfg.settings[i].enabled => value.clone(),
            _ => "89.91".to_string(),
        };
        let label = REFRESH_RATES
            .iter()
            .find(|(v, _)| *v == current)
            .map(|(_, l)| l.to_string())
            .unwrap_or_else(|| format!("{current} Hz (custom)"));
        let mut picked = current.clone();
        ui.horizontal(|ui| {
            ui.label("Headset refresh rate:");
            egui::ComboBox::from_id_salt("refresh_rate")
                .selected_text(label)
                .show_ui(ui, |ui| {
                    for (v, l) in REFRESH_RATES {
                        ui.selectable_value(&mut picked, v.to_string(), l);
                    }
                });
        })
        .response
        .on_hover_text("Used the next time SteamVR, the Monado service or an app is started.");
        if picked != current {
            self.cfg.settings[i].kind = Kind::Value { value: picked };
            self.cfg.settings[i].enabled = true;
            if let Err(e) = save_config(&self.cfg) {
                eprintln!("can't save the settings: {e}");
            }
        }
    }

    fn launcher_tab(&mut self, ui: &mut egui::Ui, st: &Status) {
        let steamvr = st.steamvr || self.steamvr.is_some();
        theme::section(ui, "SteamVR with our driver", |ui| {
            theme::note(ui, "Starts SteamVR with the PS VR2 driver. Games you start from Steam then run in the headset.");
            ui.horizontal(|ui| {
                let state = if st.steamvr { "Running" } else if self.steamvr.is_some() { "Starting" } else { "Stopped" };
                if ui.add_enabled(!steamvr, theme::primary("Start SteamVR")).clicked() {
                    self.start_steamvr();
                }
                if ui.add_enabled(steamvr, egui::Button::new("Stop SteamVR")).clicked() {
                    self.stop_steamvr();
                }
                ui.label(egui::RichText::new(state).color(if st.steamvr { theme::LIT } else { theme::SLATE }).font(theme::bold(14.0)));
            });
            self.refresh_picker(ui);
            ui.horizontal(|ui| {
                if ui.button("Detect Steam").on_hover_text("Find Steam's and SteamVR's install folders.").clicked() {
                    let (root, vr, note) = detect_steam();
                    if let Some(root) = root {
                        self.cfg.steam_root = root.display().to_string();
                    }
                    self.cfg.steamvr_dir = vr.map(|v| v.display().to_string()).unwrap_or_default();
                    let saved = save_config(&self.cfg).map(|_| " (saved)").unwrap_or(" (couldn't save the settings)");
                    self.message = format!("{note}{saved}");
                }
                theme::note(ui, &format!(
                    "Steam: {}   SteamVR: {}",
                    if self.cfg.steam_root.is_empty() { "default" } else { &self.cfg.steam_root },
                    if self.cfg.steamvr_dir.is_empty() { "default" } else { &self.cfg.steamvr_dir }
                ));
            });
        });

        theme::section(ui, "Play area", |ui| {
            theme::note(ui, "Use the square from Settings, or draw your own room in SteamVR's Room Setup.");
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(!steamvr, egui::Button::new("Set Default Play Area"))
                    .on_hover_text("SteamVR gets the square play area from Settings again (your drawn room is backed up).")
                    .on_disabled_hover_text("Close SteamVR first: it owns the play area file while running.")
                    .clicked()
                {
                    self.set_default_play_area();
                }
                if ui
                    .add_enabled(!st.service, egui::Button::new("Set Custom Play Area"))
                    .on_hover_text("How to open SteamVR's Room Setup in the headset: set the floor, then trace your play area (starts SteamVR if needed).")
                    .on_disabled_hover_text("Stop monado-service first: SteamVR needs the headset.")
                    .clicked()
                {
                    self.set_custom_play_area(steamvr);
                }
                if ui
                    .add_enabled(!st.service, egui::Button::new("Set Custom Play Area (legacy)"))
                    .on_hover_text("SteamVR's older desktop Room Setup tool (starts SteamVR if needed).")
                    .on_disabled_hover_text("Stop monado-service first: SteamVR needs the headset.")
                    .clicked()
                {
                    self.set_custom_play_area_legacy(steamvr);
                }
            });
        });

        theme::section(ui, "OpenXR with Monado", |ui| {
            theme::note(ui, "Runs Monado's own OpenXR service instead of SteamVR, for native OpenXR apps.");
            ui.horizontal(|ui| {
                let running = st.service || self.service.is_some();
                if ui.add_enabled(!running, theme::primary("Start monado-service")).clicked() {
                    self.start_service();
                }
                if ui.add_enabled(running, egui::Button::new("Stop monado-service")).clicked() {
                    self.stop_service();
                }
                let state = if st.service { "Running" } else if self.service.is_some() { "Starting" } else { "Stopped" };
                ui.label(egui::RichText::new(state).color(if st.service { theme::LIT } else { theme::SLATE }).font(theme::bold(14.0)));
            });
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                ui.label("App");
                let w = (ui.available_width() - 100.0).max(200.0);
                ui.add(egui::TextEdit::singleline(&mut self.app_path).hint_text("Path to an OpenXR app").desired_width(w));
                if ui.button("Browse...").clicked() {
                    if let Some(p) = rfd::FileDialog::new().pick_file() {
                        self.app_path = p.display().to_string();
                    }
                }
            });
            ui.horizontal(|ui| {
                ui.label("Arguments");
                let w = (ui.available_width()).max(200.0);
                ui.add(egui::TextEdit::singleline(&mut self.app_args).desired_width(w));
            });
            ui.horizontal(|ui| {
                let running = self.app.is_some();
                if ui.add_enabled(!running, egui::Button::new("Launch app")).clicked() {
                    self.launch_app();
                }
                if ui.add_enabled(running, egui::Button::new("Stop app")).clicked() {
                    self.stop_app();
                }
            });
            ui.add_space(6.0);
            theme::note(ui, "Native OpenXR apps started elsewhere (monado-service must be running):");
            // Only real paths: blank until a compiled build (Monado's OpenXR runtime file) is found on this PC.
            let manifest = self.cfg.manifest();
            let built = manifest.is_file();
            let xr = format!("XR_RUNTIME_JSON={}", manifest.display());
            copy_row(ui, if built { &xr } else { "" });
            theme::note(ui, "Steam games on Monado (Properties > Launch Options):");
            let pv = format!("PRESSURE_VESSEL_FILESYSTEMS_RW={}/monado_comp_ipc", runtime_dir());
            let steam = format!("{xr} {pv} %command%");
            copy_row(ui, if built { &steam } else { "" });
        });
    }

    /// The driver to register: the first build SteamVR can load inside Steam's runtime container (the Compile Driver
    /// tab's install, this app's driver build, the kit's container build next to it).
    fn registrable_plugin(&mut self) -> Option<PathBuf> {
        if let Some((t, p)) = &self.reg_cache {
            if t.elapsed() < Duration::from_secs(5) {
                return p.clone();
            }
        }
        let p = self.find_registrable_plugin();
        self.reg_cache = Some((Instant::now(), p.clone()));
        p
    }

    fn find_registrable_plugin(&self) -> Option<PathBuf> {
        let mut builds: Vec<PathBuf> = driver_env().into_iter().filter(|(k, _)| k == "MONADO_BUILD").map(|(_, v)| PathBuf::from(v)).collect();
        builds.push(PathBuf::from(&self.cfg.monado_build));
        builds.push(Path::new(&self.cfg.test_kit).join("../trial/build-steamvr"));
        builds
            .into_iter()
            .map(|b| b.join("steamvr-monado-psvr2"))
            .map(|p| p.canonicalize().unwrap_or(p))
            .find(|p| container_problems(p).is_ok_and(|m| m.is_empty()))
    }

    /// Compile and install on a thread: the compile script built into this app (build-driver.sh, so an AppImage of
    /// DriverKit can install and update everything), source always our GitHub repository. Its output drives the
    /// progress bar. When it's done, this app uses the new build and kit (saved), and driver.env tells the scripts.
    fn start_compile(&mut self, steamvr_running: bool) {
        if steamvr_running {
            self.message = "Close SteamVR first: it has the driver loaded.".into();
            return;
        }
        let dest = self.cfg.install_dir.trim().to_string();
        if dest.is_empty() {
            self.message = "Pick an install location first.".into();
            return;
        }
        if let Err(e) = fs::create_dir_all(&dest) {
            self.message = format!("Can't create {dest}: {e}");
            return;
        }
        let script = Path::new(&dest).join("build-driver.sh");
        if let Err(e) = fs::write(&script, BUILD_SCRIPT) {
            self.message = format!("Can't write {}: {e}", script.display());
            return;
        }
        let _ = save_config(&self.cfg);
        *self.build.lock().unwrap() = BuildState { running: true, ..Default::default() };
        let state = self.build.clone();
        let log = Path::new(&dest).join("compile-output.log");
        self.message = format!("Compiling into {dest} (log {})", log.display());
        thread::spawn(move || {
            let mut out = fs::File::create(&log).ok();
            let child = Command::new("bash")
                .arg(&script)
                .args(["--dest", &dest])
                .stdin(Stdio::null())
                .stdout(Stdio::piped())
                .stderr(Stdio::piped())
                .spawn();
            let mut child = match child {
                Ok(c) => c,
                Err(e) => {
                    let mut st = state.lock().unwrap();
                    st.running = false;
                    st.result = Some(Err(format!("couldn't run {}: {e}", script.display())));
                    return;
                }
            };
            let stderr = child.stderr.take();
            let st2 = state.clone();
            let err_thread = thread::spawn(move || {
                use std::io::BufRead;
                if let Some(e) = stderr {
                    for line in std::io::BufReader::new(e).lines().map_while(Result::ok) {
                        st2.lock().unwrap().lines.push(line);
                    }
                }
            });
            {
                use std::io::BufRead;
                let stdout = child.stdout.take().expect("piped stdout");
                for line in std::io::BufReader::new(stdout).lines().map_while(Result::ok) {
                    if let Some(f) = out.as_mut() {
                        let _ = writeln!(f, "{line}");
                    }
                    let mut st = state.lock().unwrap();
                    if let Some(n) = line.strip_prefix("STEP ").and_then(|r| r.split('/').next()).and_then(|n| n.parse().ok()) {
                        st.step = n;
                    }
                    if let Some((d, t)) = ninja_progress(&line) {
                        st.done = d;
                        st.total = t;
                    } else {
                        st.lines.push(line);
                        let n = st.lines.len();
                        if n > 400 {
                            st.lines.drain(0..n - 400);
                        }
                    }
                }
            }
            let _ = err_thread.join();
            let ok = child.wait().map(|s| s.success()).unwrap_or(false);
            let mut st = state.lock().unwrap();
            st.running = false;
            st.result = Some(if ok {
                Ok(dest)
            } else {
                Err(st.lines.iter().rev().find(|l| l.starts_with("ERROR")).cloned().unwrap_or_else(|| "The compile failed: see the log.".into()))
            });
        });
    }

    fn compile_tab(&mut self, ui: &mut egui::Ui, st: &Status) {
        let steamvr = st.steamvr || self.steamvr.is_some();
        let (running, frac, done, total, step, result, tail) = {
            let b = self.build.lock().unwrap();
            let tail: Vec<String> = b.lines.iter().rev().take(10).rev().cloned().collect();
            (b.running, b.fraction(), b.done, b.total, b.step, b.result.clone(), tail)
        };
        theme::section(ui, "Compile the driver", |ui| {
            theme::note(ui, "Downloads the latest PSVR2-DriverKit from GitHub and builds everything into one folder: the SteamVR driver, Monado's OpenXR service, the launch scripts and tools, and a new PSVR2-DriverKit app. This app then uses that folder.");
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("Install location");
                let w = (ui.available_width() - 200.0).max(240.0);
                ui.add(egui::TextEdit::singleline(&mut self.cfg.install_dir).desired_width(w));
                if ui.button("Browse...").clicked() {
                    if let Some(p) = rfd::FileDialog::new().set_directory(&self.cfg.install_dir).pick_folder() {
                        self.cfg.install_dir = p.display().to_string();
                    }
                }
                if ui.button("Default").on_hover_text(default_install_dir()).clicked() {
                    self.cfg.install_dir = default_install_dir();
                }
            });
            theme::note(ui, &format!("Source: {REPO_WEB} (main branch)"));
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(!running && !steamvr, theme::primary("Compile Driver"))
                    .on_disabled_hover_text(if steamvr { "Close SteamVR first: it has the driver loaded." } else { "Compiling..." })
                    .clicked()
                {
                    self.start_compile(steamvr);
                }
                let checking = self.check.lock().unwrap().running;
                if ui
                    .add_enabled(!running && !checking, egui::Button::new("Check"))
                    .on_hover_text("Is the installed build the latest on GitHub, is there a newer one, or is nothing installed yet?")
                    .clicked()
                {
                    let dest = PathBuf::from(self.cfg.install_dir.trim());
                    *self.check.lock().unwrap() = CheckState { running: true, result: None };
                    let state = self.check.clone();
                    thread::spawn(move || {
                        let r = check_build(&dest);
                        *state.lock().unwrap() = CheckState { running: false, result: Some(r) };
                    });
                }
                if running || checking {
                    ui.spinner();
                }
            });
            if let Some(r) = self.check.lock().unwrap().result.clone() {
                let (color, text) = match r {
                    CheckResult::UpToDate(t) => (theme::LIT, t),
                    CheckResult::NewBuild(t) => (theme::PS_BLUE.gamma_multiply(1.4), t),
                    CheckResult::NotInstalled(t) => (theme::SLATE, t),
                    CheckResult::Failed(t) => (theme::FAULT, t),
                };
                ui.label(egui::RichText::new(text).color(color).font(theme::bold(13.0)));
            }
            if running || result.is_some() {
                let text = match (running, &result) {
                    (true, _) if step == 3 && total > 0 => format!("Compiling {done} of {total}"),
                    (true, _) => ["Starting", "Downloading the source", "Configuring", "Compiling", "Building PSVR2-DriverKit and tools", "Saving the install location"][step.min(5) as usize].to_string(),
                    (false, Some(Ok(_))) => "Done".to_string(),
                    _ => "Failed".to_string(),
                };
                ui.add(egui::ProgressBar::new(frac).text(egui::RichText::new(text).color(theme::GRAPHITE)).fill(theme::PS_BLUE).corner_radius(egui::CornerRadius::same(6)));
            }
            if let Some(r) = &result {
                match r {
                    Ok(dest) => {
                        let build = Path::new(dest).join("build");
                        let kit = Path::new(dest).join("kit");
                        if self.cfg.monado_build != build.display().to_string() {
                            // Everything launched from here uses the new build and kit from now on.
                            self.cfg.monado_build = build.display().to_string();
                            if kit.join("launch-steamvr.sh").exists() {
                                self.cfg.test_kit = kit.display().to_string();
                            }
                            let _ = save_config(&self.cfg);
                            self.message = format!("Compiled into {dest}. SteamVR and monado-service now start from there.");
                        }
                        ui.colored_label(theme::LIT, format!("Installed in {dest}. The new app is {}/PSVR2-DriverKit.", kit.display()));
                    }
                    Err(e) => {
                        ui.colored_label(theme::FAULT, e);
                    }
                }
            }
            if !tail.is_empty() {
                egui::Frame::new().fill(theme::FIELD).corner_radius(egui::CornerRadius::same(8)).inner_margin(egui::Margin::same(10)).show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    for l in &tail {
                        ui.label(egui::RichText::new(l).monospace().size(12.0).color(theme::SLATE));
                    }
                });
            }
        });

        theme::section(ui, "Register with SteamVR", |ui| {
            let plugin = self.registrable_plugin().unwrap_or_else(|| Path::new(&self.cfg.monado_build).join("steamvr-monado-psvr2"));
            let list = external_drivers().map(|(_, l)| l).unwrap_or_default();
            let ours = list.iter().any(|p| p.trim_end_matches('/').ends_with("steamvr-monado-psvr2"));
            // Registered, but not this (container-ready) copy: Register replaces it.
            let current = list.iter().any(|p| Path::new(p.trim_end_matches('/')) == plugin.as_path());
            let sony = list.iter().any(|p| p.contains("PlayStation VR2 App"));
            theme::note(ui, "When our driver is registered, Steam starts SteamVR with it, from the Library or from any game. Unregister only removes our driver; it doesn't register any other.");
            ui.horizontal(|ui| {
                let (color, text) = match (ours, sony) {
                    (true, _) if !current => (theme::FAULT, "The registered copy of our driver can't load when Steam starts SteamVR: Register replaces it with this one"),
                    (true, _) => (theme::LIT, "Our driver is registered"),
                    (false, true) => (theme::SLATE, "Sony's PS VR2 driver is registered"),
                    _ => (theme::SLATE, "No PS VR2 driver is registered"),
                };
                ui.label(egui::RichText::new(text).color(color).font(theme::bold(14.0)));
            });
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(!steamvr && !current && plugin.join("driver.vrdrivermanifest").exists(), theme::primary("Register our driver"))
                    .on_disabled_hover_text(if steamvr { "Close SteamVR first." } else if current { "Already registered." } else { "Compile the driver first." })
                    .clicked()
                {
                    self.message = match container_problems(&plugin) {
                        Ok(m) if m.is_empty() => register_driver(&plugin).unwrap_or_else(|e| format!("Couldn't register: {e}")),
                        Ok(m) => format!(
                            "Not registered: SteamVR started by Steam runs inside Steam's runtime, which lacks {} needed by {}. Compile the driver on this tab first (it bundles them).",
                            m.join(", "),
                            plugin.display()
                        ),
                        Err(e) => format!("Not registered: {e}"),
                    };
                }
                if ui
                    .add_enabled(!steamvr && ours, egui::Button::new("Unregister"))
                    .on_disabled_hover_text(if steamvr { "Close SteamVR first." } else { "Our driver isn't registered." })
                    .clicked()
                {
                    self.message = unregister_driver().unwrap_or_else(|e| format!("Couldn't unregister: {e}"));
                }
            });
            ui.label(egui::RichText::new(format!("Driver: {}", plugin.display())).color(theme::SLATE).size(12.0));
        });

        theme::section(ui, "SteamVR features", |ui| {
            theme::note(ui, "Turns on SteamVR's camera features for the PS VR2, permanently: the camera, Room View as the full camera picture (headset button or double-press PS), and the camera when you get near your play-area walls. Room setup needs nothing here: the driver offers it itself.");
            let file = steamvr_settings_file(&self.cfg.steam_root);
            let on = fs::read_to_string(&file)
                .ok()
                .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok())
                .map(|v| psvr2_settings_on(&v))
                .unwrap_or(0);
            let all = on == PSVR2_STEAMVR_SETTINGS.len();
            ui.label(egui::RichText::new(if all { "All PS VR2 camera features are on".to_string() } else { format!("{on} of {} PS VR2 camera features are on", PSVR2_STEAMVR_SETTINGS.len()) })
                .font(theme::bold(14.0))
                .color(if all { theme::LIT } else { theme::SLATE }));
            let backup = config_dir().join("steamvr.vrsettings.before-psvr2");
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(!steamvr && !all, theme::primary("Turn on PS VR2 features"))
                    .on_disabled_hover_text(if steamvr { "Close SteamVR first: it rewrites its settings while running." } else { "Already on." })
                    .clicked()
                {
                    self.message = apply_psvr2_settings(&file).unwrap_or_else(|e| format!("Couldn't change SteamVR's settings: {e}"));
                }
                if ui
                    .add_enabled(!steamvr && backup.exists(), egui::Button::new("Undo"))
                    .on_hover_text("Puts SteamVR's settings back from the backup made when the features were turned on.")
                    .on_disabled_hover_text(if steamvr { "Close SteamVR first." } else { "Nothing to undo." })
                    .clicked()
                {
                    self.message = undo_psvr2_settings(&file).unwrap_or_else(|e| format!("Couldn't undo: {e}"));
                }
            });
            ui.label(egui::RichText::new(format!("Settings file: {}", file.display())).color(theme::SLATE).size(12.0));
        });

        theme::section(ui, "Packages needed to compile", |ui| {
            theme::note(ui, "Install these first. Copy the line for your distribution into a terminal.");
            for (distro, cmd) in PACKAGES {
                ui.add_space(4.0);
                ui.label(egui::RichText::new(distro).font(theme::bold(14.0)));
                copy_row(ui, cmd);
            }
        });
    }

    fn settings_tab(&mut self, ui: &mut egui::Ui) {
        theme::section(ui, "Folders", |ui| {
            let w = (ui.available_width() - 200.0).max(240.0);
            egui::Grid::new("paths").num_columns(2).spacing([12.0, 8.0]).show(ui, |ui| {
                ui.label("Launch scripts and logs");
                ui.add(egui::TextEdit::singleline(&mut self.cfg.test_kit).desired_width(w));
                ui.end_row();
                ui.label("Driver build");
                ui.add(egui::TextEdit::singleline(&mut self.cfg.monado_build).desired_width(w));
                ui.end_row();
                ui.label("Steam");
                ui.add(egui::TextEdit::singleline(&mut self.cfg.steam_root).hint_text("found by Detect Steam").desired_width(w));
                ui.end_row();
                ui.label("SteamVR");
                ui.add(egui::TextEdit::singleline(&mut self.cfg.steamvr_dir).hint_text("default").desired_width(w));
                ui.end_row();
            });
        });
        let sections = [
            (Target::SteamVR, "SteamVR driver settings",
             "Used when SteamVR starts from here, and by the games you then start from Steam."),
            (Target::Monado, "OpenXR service settings",
             "Used by monado-service started from here, and the OpenXR apps on it."),
        ];
        for (target, title, what) in sections {
            theme::section(ui, title, |ui| {
                theme::note(ui, what);
                egui::Grid::new(title).num_columns(3).striped(true).spacing([14.0, 6.0]).show(ui, |ui| {
                    for s in self.cfg.settings.iter_mut().filter(|s| s.target == target) {
                        ui.checkbox(&mut s.enabled, &s.label).on_hover_text(format!("{}\n\n{}", s.help, s.key));
                        match &mut s.kind {
                            Kind::Value { value } => {
                                ui.add_enabled(s.enabled, egui::TextEdit::singleline(value).desired_width(220.0));
                            }
                            Kind::Toggle { on, off } => {
                                let v = if s.enabled { on.as_str() } else { off.as_str() };
                                ui.label(egui::RichText::new(if v.is_empty() { format!("{} unset", s.key) } else { format!("{}={}", s.key, v) }).monospace().size(12.0).color(theme::SLATE));
                            }
                        }
                        ui.add(egui::Label::new(egui::RichText::new(&s.help).color(theme::SLATE).size(12.0)).wrap());
                        ui.end_row();
                    }
                });
                ui.add_space(4.0);
                theme::note(ui, "Environment these settings give:");
                let env: Vec<String> = self.cfg.env(target).iter().map(|(k, v)| format!("{k}={v}")).collect();
                copy_row(ui, &env.join(" "));
            });
        }
        ui.horizontal(|ui| {
            if ui.add(theme::primary("Save settings")).clicked() {
                match save_config(&self.cfg) {
                    Ok(()) => {
                        self.saved_at = Some(Instant::now());
                        self.message = format!("Saved {}", config_path().display());
                    }
                    Err(e) => self.message = format!("Couldn't save: {e}"),
                }
            }
            if ui.button("Restore defaults").clicked() {
                let d = Config::default();
                self.cfg.settings = d.settings;
                self.message = "Defaults restored. Save settings to keep them.".into();
            }
        });
    }
}

impl Launcher {
    fn take_screenshot(&mut self, ctx: &egui::Context) {
        let Some((path, t0, asked)) = self.screenshot.clone() else {
            return;
        };
        ctx.request_repaint();
        if !asked && t0.elapsed() > Duration::from_millis(1500) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::UserData::default()));
            self.screenshot = Some((path.clone(), t0, true));
        }
        let shot = ctx.input(|i| {
            i.raw.events.iter().find_map(|e| match e {
                egui::Event::Screenshot { image, .. } => Some(image.clone()),
                _ => None,
            })
        });
        if let Some(img) = shot {
            if let Ok(f) = fs::File::create(&path) {
                let mut enc = png::Encoder::new(std::io::BufWriter::new(f), img.size[0] as u32, img.size[1] as u32);
                enc.set_color(png::ColorType::Rgba);
                enc.set_depth(png::BitDepth::Eight);
                if let Ok(mut w) = enc.write_header() {
                    let _ = w.write_image_data(img.as_raw());
                }
            }
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
    }
}

/// The compile script (psvr2-test/build-driver.sh), built into the app: DriverKit can compile and update everything
/// on its own (e.g. as an AppImage), without a copy of the kit.
const BUILD_SCRIPT: &str = include_str!("../../build-driver.sh");
const REPO_WEB: &str = "github.com/mkooner/PSVR2-DriverKit";

fn main() -> eframe::Result<()> {
    // --check [install folder]: what the Compile tab's Check button reports, printed (default: the saved install folder)
    let args: Vec<String> = std::env::args().collect();
    if let Some(i) = args.iter().position(|a| a == "--check") {
        let dest = args.get(i + 1).cloned().unwrap_or_else(|| load_config().install_dir);
        let text = match check_build(Path::new(&dest)) {
            CheckResult::UpToDate(t) | CheckResult::NewBuild(t) | CheckResult::NotInstalled(t) | CheckResult::Failed(t) => t,
        };
        println!("{text}");
        return Ok(());
    }
    if std::env::args().any(|a| a == "--status") {
        // What the launcher sees, without the window (for checks and scripts).
        let st = poll_status();
        let show = |s: Option<bool>| match s {
            Some(true) => "connected",
            Some(false) => "not connected",
            None => "not paired",
        };
        let cfg = load_config();
        println!("headset (USB 054c:0cde): {}", show(Some(st.headset)));
        println!("left controller: {}", show(st.left));
        println!("right controller: {}", show(st.right));
        println!("SteamVR running: {}, monado-service running: {}", st.steamvr, st.service);
        println!("XR_RUNTIME_JSON={}", cfg.manifest().display());
        println!("Steam detection: {}", detect_steam().2);
        for launch in [Target::SteamVR, Target::Monado] {
            let env: Vec<String> = cfg.env(launch).iter().map(|(k, v)| format!("{k}={v}")).collect();
            println!("settings environment for {}: {}", launch.label(), env.join(" "));
        }
        return Ok(());
    }
    // --screenshot: a tall window, so a whole tab fits in the picture.
    let height = if std::env::args().any(|a| a == "--screenshot") { 1500.0 } else { 760.0 };
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([980.0, height]).with_min_inner_size([720.0, 520.0]).with_title("PSVR2-DriverKit"),
        ..Default::default()
    };
    let args: Vec<String> = std::env::args().collect();
    let shot = args.iter().position(|a| a == "--screenshot").and_then(|i| {
        let tab = match args.get(i + 2).map(String::as_str) {
            Some("settings") => Tab::Settings,
            Some("compile") => Tab::Compile,
            _ => Tab::Launcher,
        };
        args.get(i + 1).map(|p| (PathBuf::from(p), tab))
    });
    eframe::run_native(
        "PSVR2-DriverKit",
        options,
        Box::new(move |cc| {
            let mut app = Launcher::new(cc);
            if let Some((path, tab)) = shot {
                app.tab = tab;
                app.screenshot = Some((path, Instant::now(), false));
            }
            Ok(Box::new(app))
        }),
    )
}
