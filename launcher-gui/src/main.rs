// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//! PSVR2-DriverKit: shows whether the headset and the Sense controllers are connected, starts and stops
//! SteamVR with our Monado plugin, monado-service, and any OpenXR app on monado-service, shows the environment an app
//! needs to use Monado, and keeps the driver settings we use (with their defaults) switchable.

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
              "Report Sony's display properties too: all, or a list of throttle, framerate, syslayer, texindex, supersample, dsc, edid, display, mask. Off by default (no help measured).", "all", false).steamvr(),
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
    /// Settings format: 2 = async reprojection off by default (SteamVR's own choice); 3 = SteamVR render scale 170;
/// 4 = smooth SLAM corrections on.
    #[serde(default)]
    version: u32,
}

const CONFIG_VERSION: u32 = 8;

impl Default for Config {
    fn default() -> Self {
        let home = std::env::var("HOME").unwrap_or_default();
        let kit = format!("{home}/Documents/Claude/Monado/psvr2-test");
        let mut settings = default_settings();
        for s in settings.iter_mut() {
            if s.key == "VIT_SYSTEM_LIBRARY_PATH" {
                s.kind = Kind::Value { value: format!("{home}/Documents/Claude/Monado/basalt-install/lib/libbasalt.so") };
            }
        }
        Config {
            test_kit: kit,
            monado_build: format!("{home}/Documents/Claude/Monado/trial/build"),
            settings,
            steam_root: String::new(),
            steamvr_dir: String::new(),
            version: CONFIG_VERSION,
        }
    }
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

/// The driver's chaperone file without a room drawn in Room Setup in the saved-room universe (5022): Some(new text),
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

#[derive(PartialEq)]
enum Tab {
    Launcher,
    Settings,
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
    saved_at: Option<Instant>,
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
            saved_at: None,
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

    /// The driver's own chaperone file (driver_monado_psvr2: <SteamVR config>/monado_psvr2/chaperone_info.vrchap),
    /// SteamVR's config folder from ~/.config/openvr/openvrpaths.vrpath as the driver finds it.
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
        config.join("monado_psvr2/chaperone_info.vrchap")
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

    /// Set Custom Play Area: SteamVR's Room Setup, to draw your own room. SteamVR running: asked to open it
    /// (tools/vrlaunch, a background client); not running: started with Room Setup opening (ROOM_SETUP=1).
    fn set_custom_play_area(&mut self, steamvr_running: bool) {
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

fn dot(ui: &mut egui::Ui, state: Option<bool>, label: &str) {
    let (color, text) = match state {
        Some(true) => (egui::Color32::from_rgb(60, 180, 90), "connected"),
        Some(false) => (egui::Color32::from_rgb(200, 70, 60), "not connected"),
        None => (egui::Color32::GRAY, "not paired"),
    };
    ui.horizontal(|ui| {
        ui.colored_label(color, "●");
        ui.label(format!("{label}: {text}"));
    });
}

fn copy_row(ui: &mut egui::Ui, text: &str) {
    ui.horizontal(|ui| {
        if ui.button("Copy").clicked() {
            ui.ctx().copy_text(text.to_string());
        }
        ui.add(egui::Label::new(egui::RichText::new(text).monospace()).wrap());
    });
}

impl eframe::App for Launcher {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.reap();
        let st = self.status.lock().unwrap().clone();
        egui::Panel::top("tabs").show_inside(ui, |ui| {
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.tab, Tab::Launcher, "Launcher");
                ui.selectable_value(&mut self.tab, Tab::Settings, "Settings");
            });
        });
        egui::Panel::bottom("message").show_inside(ui, |ui| {
            ui.label(&self.message);
        });
        egui::CentralPanel::default().show_inside(ui, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| match self.tab {
                Tab::Launcher => self.launcher_tab(ui, &st),
                Tab::Settings => self.settings_tab(ui),
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
        ui.heading("Devices");
        dot(ui, Some(st.headset), "PS VR2 headset (USB)");
        dot(ui, st.left, "Left Sense controller");
        dot(ui, st.right, "Right Sense controller");
        ui.horizontal(|ui| {
            let steamvr = st.steamvr || self.steamvr.is_some();
            if ui
                .add_enabled(!steamvr, egui::Button::new("Set Default Play Area"))
                .on_hover_text("SteamVR gets the square play area from Settings again (your drawn room is backed up). SteamVR must be closed.")
                .on_disabled_hover_text("Close SteamVR first: it owns the play area file while running.")
                .clicked()
            {
                self.set_default_play_area();
            }
            if ui
                .add_enabled(!st.service, egui::Button::new("Set Custom Play Area"))
                .on_hover_text("Opens SteamVR's Room Setup to draw your own room (starts SteamVR if needed). Kept until you set the default again.")
                .on_disabled_hover_text("Stop monado-service first: SteamVR needs the headset.")
                .clicked()
            {
                self.set_custom_play_area(steamvr);
            }
        });
        self.refresh_picker(ui);
        ui.separator();

        ui.heading("SteamVR with Monado");
        ui.horizontal(|ui| {
            let running = st.steamvr || self.steamvr.is_some();
            ui.label(if st.steamvr { "running" } else if self.steamvr.is_some() { "starting" } else { "stopped" });
            if ui.add_enabled(!running, egui::Button::new("Start SteamVR")).clicked() {
                self.start_steamvr();
            }
            if ui.add_enabled(running, egui::Button::new("Stop SteamVR")).clicked() {
                self.stop_steamvr();
            }
            if ui.button("Detect Steam").clicked() {
                let (root, vr, note) = detect_steam();
                if let Some(root) = root {
                    self.cfg.steam_root = root.display().to_string();
                }
                self.cfg.steamvr_dir = vr.map(|v| v.display().to_string()).unwrap_or_default();
                let saved = save_config(&self.cfg).map(|_| " (saved)").unwrap_or(" (couldn't save the settings)");
                self.message = format!("{note}{saved}");
            }
        });
        ui.label(format!(
            "Steam: {}   SteamVR: {}",
            if self.cfg.steam_root.is_empty() { "default (not detected yet)" } else { &self.cfg.steam_root },
            if self.cfg.steamvr_dir.is_empty() { "default" } else { &self.cfg.steamvr_dir }
        ));
        ui.separator();

        ui.heading("Monado service (OpenXR runtime)");
        ui.horizontal(|ui| {
            let running = st.service || self.service.is_some();
            ui.label(if st.service { "running" } else if self.service.is_some() { "starting" } else { "stopped" });
            if ui.add_enabled(!running, egui::Button::new("Start monado-service")).clicked() {
                self.start_service();
            }
            if ui.add_enabled(running, egui::Button::new("Stop monado-service")).clicked() {
                self.stop_service();
            }
        });
        ui.separator();

        ui.heading("Launch an app with Monado");
        ui.horizontal(|ui| {
            ui.label("App:");
            ui.text_edit_singleline(&mut self.app_path);
            if ui.button("Browse...").clicked() {
                if let Some(p) = rfd::FileDialog::new().pick_file() {
                    self.app_path = p.display().to_string();
                }
            }
        });
        ui.horizontal(|ui| {
            ui.label("Arguments:");
            ui.text_edit_singleline(&mut self.app_args);
        });
        ui.horizontal(|ui| {
            let running = self.app.is_some();
            ui.label(if running { "app running" } else { "no app running" });
            if ui.add_enabled(!running, egui::Button::new("Launch")).clicked() {
                self.launch_app();
            }
            if ui.add_enabled(running, egui::Button::new("Stop app")).clicked() {
                self.stop_app();
            }
        });
        ui.separator();

        ui.heading("Environment for running apps on Monado");
        ui.label("Native OpenXR apps (monado-service must be running):");
        let manifest = self.cfg.manifest().display().to_string();
        let xr = format!("XR_RUNTIME_JSON={manifest}");
        copy_row(ui, &xr);
        ui.label("Steam games (Proton / Steam Linux Runtime also need the service socket inside the container):");
        let runtime = std::env::var("XDG_RUNTIME_DIR").unwrap_or_else(|_| "/run/user/1000".into());
        let pv = format!("PRESSURE_VESSEL_FILESYSTEMS_RW={runtime}/monado_comp_ipc");
        copy_row(ui, &pv);
        ui.label("Steam launch options (Properties > Launch Options):");
        copy_row(ui, &format!("{xr} {pv} %command%"));
    }

    fn settings_tab(&mut self, ui: &mut egui::Ui) {
        ui.heading("Paths");
        egui::Grid::new("paths").num_columns(2).show(ui, |ui| {
            ui.label("Test kit (launch-steamvr.sh, logs):");
            ui.text_edit_singleline(&mut self.cfg.test_kit);
            ui.end_row();
            ui.label("Monado build:");
            ui.text_edit_singleline(&mut self.cfg.monado_build);
            ui.end_row();
            ui.label("Steam install (Detect Steam on the Launcher tab):");
            ui.text_edit_singleline(&mut self.cfg.steam_root);
            ui.end_row();
            ui.label("SteamVR install:");
            ui.text_edit_singleline(&mut self.cfg.steamvr_dir);
            ui.end_row();
        });
        ui.separator();
        ui.label("Saved with Save and used by every launch from here. Each section goes only to its own runtime.");
        let sections = [
            (Target::SteamVR, "SteamVR plugin (driver_monado_psvr2)",
             "SteamVR started from here (launch-steamvr.sh), and the games you then start from Steam."),
            (Target::Monado, "OpenXR (monado-service)",
             "monado-service started from here, and the OpenXR apps on it (from here or with the variables on the Launcher tab)."),
        ];
        for (target, title, what) in sections {
            ui.separator();
            ui.heading(title);
            ui.label(egui::RichText::new(what).small());
            egui::Grid::new(title).num_columns(3).striped(true).show(ui, |ui| {
                for s in self.cfg.settings.iter_mut().filter(|s| s.target == target) {
                    ui.checkbox(&mut s.enabled, &s.label).on_hover_text(format!("{}\n\n{}", s.help, s.key));
                    match &mut s.kind {
                        Kind::Value { value } => {
                            ui.add_enabled(s.enabled, egui::TextEdit::singleline(value).desired_width(260.0));
                        }
                        Kind::Toggle { on, off } => {
                            let v = if s.enabled { on.as_str() } else { off.as_str() };
                            ui.label(if v.is_empty() { format!("{} unset", s.key) } else { format!("{}={}", s.key, v) });
                        }
                    }
                    ui.label(egui::RichText::new(&s.help).small());
                    ui.end_row();
                }
            });
            ui.label("Environment this section gives:");
            let env: Vec<String> = self.cfg.env(target).iter().map(|(k, v)| format!("{k}={v}")).collect();
            copy_row(ui, &env.join(" "));
        }
        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Save").clicked() {
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
                self.message = "Defaults restored (Save to keep them).".into();
            }
        });
    }
}

fn main() -> eframe::Result<()> {
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
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([900.0, 720.0]).with_title("PSVR2-DriverKit"),
        ..Default::default()
    };
    eframe::run_native("PSVR2-DriverKit", options, Box::new(|cc| Ok(Box::new(Launcher::new(cc)))))
}
