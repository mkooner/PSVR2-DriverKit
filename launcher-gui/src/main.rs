// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
//! PS VR2 Monado launcher: shows whether the headset and the Sense controllers are connected, starts and stops
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

#[derive(Clone, Serialize, Deserialize)]
struct Setting {
    key: String,
    label: String,
    help: String,
    kind: Kind,
    enabled: bool,
}

fn toggle(key: &str, label: &str, help: &str, enabled: bool) -> Setting {
    Setting {
        key: key.into(),
        label: label.into(),
        help: help.into(),
        kind: Kind::Toggle { on: "1".into(), off: "0".into() },
        enabled,
    }
}

fn value(key: &str, label: &str, help: &str, v: &str, enabled: bool) -> Setting {
    Setting { key: key.into(), label: label.into(), help: help.into(), kind: Kind::Value { value: v.into() }, enabled }
}

/// The options we use, with their defaults (as in the driver and the launch scripts).
fn default_settings() -> Vec<Setting> {
    vec![
        toggle("PSVR2_ACCEL_FUSION", "Accelerometer-aided head position",
               "Head position between SLAM poses from the 2 kHz accelerometer (smoother turning).", true),
        value("PSVR2_GYRO_BIAS_TAU", "Gyro bias correction (s)",
              "Learns the headset gyro's offset (~0.8 deg/s) with this time constant.", "5", true),
        value("PSVR2_POSE_BLEND_MS", "Position correction blend (ms)",
              "Spreads each position correction over this long (100: less jitter, no added latency).", "100", true),
        value("PSVR2_GRAVITY_PAUSE_DEG", "Gravity estimate pause (deg/s)",
              "With accelerometer fusion: don't update the gravity estimate while turning faster than this.", "30", true),
        value("PSVR2_REFRESH_HZ", "Display refresh (Hz)",
              "89.91 or 119.88. SteamVR and Monado direct both pick the matching display mode.", "89.91", true),
        value("XRT_COMPOSITOR_SCALE_PERCENTAGE", "SteamVR render scale (%)",
              "Per-eye render size for SteamVR games (140: sharp in the lens centre; 100: lighter).", "140", true),
        value("OXR_VIEWPORT_SCALE_PERCENTAGE", "OpenXR render scale (%)",
              "Recommended render size for apps on monado-service.", "100", true),
        toggle("PSVR2_BASALT", "Basalt referee",
               "Basalt visual-inertial tracking catches the headset's position jumps in fast turns (~2 CPU cores).", false),
        value("VIT_SYSTEM_LIBRARY_PATH", "Basalt library",
              "libbasalt.so for the Basalt referee.", "", true),
        toggle("PSVR2_SONY_LD_CONFIG", "Sony's LED detector settings",
               "Send Sony's LED detector settings to the headset (controller tracking in bright rooms).", true),
        toggle("PSVR2_SLAM_CARRY", "Carry position over SLAM resets",
               "Keeps the head position continuous when the headset's tracking re-originates.", true),
        toggle("OVRD_HIDDEN_AREA", "Hidden-area mesh (SteamVR)",
               "Experimental: our distortion doesn't model the lens edge yet, so it hides almost nothing.", false),
        toggle("ASYNC", "SteamVR async reprojection",
               "SteamVR launcher: re-warps late frames (steadier, can flicker on NVIDIA).", true),
        toggle("RECORD_DATASET", "Record controller dataset", "SteamVR launcher: record LED tracking data.", false),
        toggle("RECORD_SLAM", "Record headset SLAM", "SteamVR launcher: record the headset's raw SLAM stream.", false),
        toggle("RECORD_CAMERA", "Record camera images", "SteamVR launcher: record camera transfers (~2 GB/min).", false),
        toggle("HAND_TRACKING", "Optical hand tracking", "SteamVR launcher: hands from the bottom cameras.", false),
    ]
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
}

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
        }
    }
}

fn config_path() -> PathBuf {
    let base = std::env::var("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from(std::env::var("HOME").unwrap_or_default()).join(".config"));
    base.join("psvr2-monado-launcher").join("settings.json")
}

fn load_config() -> Config {
    let mut cfg: Config = fs::read_to_string(config_path())
        .ok()
        .and_then(|s| serde_json::from_str(&s).ok())
        .unwrap_or_default();
    // Settings added since the file was written get their defaults.
    for d in Config::default().settings {
        if !cfg.settings.iter().any(|s| s.key == d.key) {
            cfg.settings.push(d);
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
    /// Environment for the launches from the settings.
    fn env(&self) -> Vec<(String, String)> {
        let mut out = Vec::new();
        for s in &self.settings {
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
        if self.settings.iter().any(|s| s.key == "PSVR2_BASALT" && s.enabled) {
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
        cmd.arg(&script).envs(self.cfg.env()).stdin(Stdio::null()).stdout(f);
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
        cmd.envs(self.cfg.env()).env("XRT_LOG", "info").stdin(Stdio::piped()).stdout(f);
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

impl Launcher {
    fn launcher_tab(&mut self, ui: &mut egui::Ui, st: &Status) {
        ui.heading("Devices");
        dot(ui, Some(st.headset), "PS VR2 headset (USB)");
        dot(ui, st.left, "Left Sense controller");
        dot(ui, st.right, "Right Sense controller");
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
        ui.heading("Driver and launcher settings");
        ui.label("Applied to SteamVR, monado-service and app launches started from here.");
        egui::Grid::new("settings").num_columns(3).striped(true).show(ui, |ui| {
            for s in self.cfg.settings.iter_mut() {
                ui.checkbox(&mut s.enabled, &s.label).on_hover_text(format!("{}\n\n{}", s.help, s.key));
                match &mut s.kind {
                    Kind::Value { value } => {
                        ui.add_enabled(s.enabled, egui::TextEdit::singleline(value).desired_width(260.0));
                    }
                    Kind::Toggle { on, off } => {
                        ui.label(format!("{}={}", s.key, if s.enabled { on.as_str() } else { off.as_str() }));
                    }
                }
                ui.label(egui::RichText::new(&s.help).small());
                ui.end_row();
            }
        });
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
        ui.label("Environment these settings give:");
        let env: Vec<String> = self.cfg.env().iter().map(|(k, v)| format!("{k}={v}")).collect();
        copy_row(ui, &env.join(" "));
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
        let env: Vec<String> = cfg.env().iter().map(|(k, v)| format!("{k}={v}")).collect();
        println!("settings environment: {}", env.join(" "));
        return Ok(());
    }
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([900.0, 720.0]).with_title("PS VR2 Monado launcher"),
        ..Default::default()
    };
    eframe::run_native("PS VR2 Monado launcher", options, Box::new(|cc| Ok(Box::new(Launcher::new(cc)))))
}
