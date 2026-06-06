#![windows_subsystem = "windows"]

use std::f64::consts::PI;
use std::fs::File;
use std::io::Write;
use std::net::UdpSocket;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use hidapi::{HidApi, HidDevice};
use inputbot::{KeybdKey, get_keybd_key};
use tray_item::{IconSource, TrayItem};
use xreal_one_driver::XrealOne;

#[macro_use]
extern crate ini;

const CONFIG_FILE: &str = "config.ini";
const LOG_FILE: &str = "xrealtrack.log";
const DEFAULT_XREAL_TCP_ADDR: &str = "169.254.2.1:52998";
const OPENTRACK_PACKET_LEN: usize = 48;

const XREAL_VID: u16 = 0x3318;
const XREAL_PID_ONE_PRO: u16 = 0x0436;
const TARGET_CONTROL_INTERFACE: i32 = 0;

#[derive(Clone)]
struct AppConfig {
    ip: String,
    port: u16,
    hotkey: String,
    xreal_addr: String,
}

#[derive(Default)]
struct OrientationState {
    raw_yaw_deg: f64,
    raw_pitch_deg: f64,
    raw_roll_deg: f64,
    yaw_offset: f64,
    pitch_offset: f64,
    roll_offset: f64,
}

fn load_config() -> AppConfig {
    log_message("Loading configuration...");
    if !Path::new(CONFIG_FILE).exists() {
        log_message("config.ini not found. Creating default configuration...");
        let mut file = File::create(CONFIG_FILE).expect("Failed to create configuration file");
        let default_settings = "[opentrack]\n\
                                ip = 127.0.0.1\n\
                                port = 4242\n\n\
                                [hotkeys]\n\
                                recenter = R\n\n\
                                [xreal]\n\
                                addr = 169.254.2.1:52998\n";
        file.write_all(default_settings.as_bytes()).expect("Error writing default settings");
    }

    let map = ini!(CONFIG_FILE);
    
    let ip = map.get("opentrack")
        .and_then(|sec| sec.get("ip").cloned().flatten())
        .unwrap_or_else(|| "127.0.0.1".to_string());
        
    let port = map.get("opentrack")
        .and_then(|sec| sec.get("port").cloned().flatten())
        .and_then(|p| p.parse::<u16>().ok())
        .unwrap_or(4242);
        
    let hotkey = map.get("hotkeys")
        .and_then(|sec| sec.get("recenter").cloned().flatten())
        .unwrap_or_else(|| "C".to_string());

    let xreal_addr = map.get("xreal")
        .and_then(|sec| sec.get("addr").cloned().flatten())
        .unwrap_or_else(|| DEFAULT_XREAL_TCP_ADDR.to_string());

    log_message(&format!("Configuration loaded successfully: OpenTrack={}:{}, Hotkey={}, Xreal={}", ip, port, hotkey, xreal_addr));

    AppConfig { ip, port, hotkey, xreal_addr }
}

fn log_message(message: &str) {
    if let Ok(mut file) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(LOG_FILE)
    {
        let _ = writeln!(file, "[LOG] {}", message);
    }
}

fn send_opentrack_packet(addr: &str, port: u16, yaw_deg: f64, roll_deg: f64, pitch_deg: f64) {
    let socket = match UdpSocket::bind("0.0.0.0:0") {
        Ok(socket) => socket,
        Err(_) => return,
    };

    let mut payload = Vec::with_capacity(OPENTRACK_PACKET_LEN);
    payload.extend_from_slice(&0.0_f64.to_le_bytes());
    payload.extend_from_slice(&0.0_f64.to_le_bytes());
    payload.extend_from_slice(&0.0_f64.to_le_bytes());
    payload.extend_from_slice(&yaw_deg.to_le_bytes());
    payload.extend_from_slice(&pitch_deg.to_le_bytes());
    payload.extend_from_slice(&roll_deg.to_le_bytes());

    let target = format!("{}:{}", addr, port);
    let _ = socket.send_to(&payload, target);
}

fn recenter_orientation(state: &Mutex<OrientationState>) {
    let mut state = state.lock().unwrap();
    state.yaw_offset = state.raw_yaw_deg;
    state.pitch_offset = state.raw_pitch_deg;
    state.roll_offset = state.raw_roll_deg;
    log_message("Recenter requested");
}

fn update_orientation(imu: &xreal_one_driver::XOImu, dt_s: f64, state: &mut OrientationState) -> (f64, f64, f64) {
    let accel = imu.accel;
    let gyro = imu.gyro;

    let pitch = (-(accel[0] as f64) / 9.81).asin().clamp(-PI / 2.0, PI / 2.0);
    let roll = (-(accel[1] as f64) / 9.81).atan2(accel[2] as f64 / 9.81);

    state.raw_pitch_deg = pitch.to_degrees();
    state.raw_roll_deg = roll.to_degrees();
    state.raw_yaw_deg += -(gyro[2] as f64) * dt_s * (180.0 / PI);

    (
        state.raw_yaw_deg - state.yaw_offset,
        state.raw_pitch_deg - state.pitch_offset,
        state.raw_roll_deg - state.roll_offset,
    )
}

fn send_handshake_a(device: &HidDevice) -> Result<(), hidapi::HidError> {
    // Packet must be exactly 65 bytes (1 byte Report ID 0x00 + 64 bytes payload)
    let mut packet = [0u8; 65];
    packet[1] = 0xAA; // Start validation byte
    packet[2] = 0x55; // Subsystem lock byte
    packet[3] = 0x11; // Security state identifier
    packet[4] = 0x01; 
    packet[5] = 0x02; // Request RNDIS subsystem state allocation

    // Compute hardware parity checksum
    let mut crc: u8 = 0;
    for i in 1..64 { crc = crc.wrapping_add(packet[i]); }
    packet[64] = crc;

    device.write(&packet)?;
    Ok(())
}

fn send_handshake_b(device: &HidDevice) -> Result<(), hidapi::HidError> {
    let mut packet = [0u8; 65];
    packet[1] = 0xAA; 
    packet[2] = 0x55; 
    packet[3] = 0x15; // Enable server daemon execution code
    packet[4] = 0x01; // Data size
    packet[5] = 0x01; // Enable flag: 1 (True)

    // Compute hardware parity checksum
    let mut crc: u8 = 0;
    for i in 1..64 { crc = crc.wrapping_add(packet[i]); }
    packet[64] = crc;

    device.write(&packet)?;
    Ok(())
}

fn parse_hotkey_key(hotkey: &str) -> Option<KeybdKey> {
    let key_text = hotkey.trim();
    if key_text.is_empty() {
        return None;
    }

    let ch = key_text.chars().next()?;
    get_keybd_key(ch.to_ascii_uppercase())
}

fn main() {
    let config = load_config();
    log_message("Starting XrealTrack tray app...");

    let running = Arc::new(AtomicBool::new(true));
    let tray_running = Arc::clone(&running);
    let orientation: Arc<Mutex<OrientationState>> = Arc::new(Mutex::new(OrientationState::default()));

    if let Some(key) = parse_hotkey_key(&config.hotkey) {
        let recenter_state = Arc::clone(&orientation);
        key.bind(move || if KeybdKey::LAltKey.is_pressed() && KeybdKey::LShiftKey.is_pressed() { recenter_orientation(&recenter_state) });
    }

    let mut tray = TrayItem::new("XrealTrack", IconSource::Resource("app_icon"))
        .expect("Failed to create tray item");
    let status_label_id = tray.inner_mut().add_label_with_id("status").expect("Failed to add status label");
    tray.inner_mut().set_label("Initializing...", status_label_id).expect("Failed to set status label");
    tray.add_label(&format!("Hotkey: LAlt + LShift + {}", config.hotkey)).expect("Failed to add hotkey label");
    let recenter_menu = Arc::clone(&orientation);
    tray.add_menu_item("Recenter", move || recenter_orientation(&recenter_menu))
        .expect("Failed to add recenter menu item");
    tray.add_menu_item("Exit", move || {
        tray_running.store(false, Ordering::Relaxed);
        std::process::exit(0);
    })
    .expect("Failed to add tray exit item");

    thread::spawn(|| inputbot::handle_input_events());

    // --- STEP 1: SEND USB HID WAKEUP COMMAND ---
    log_message("Initializing USB HID context...");
    let api = HidApi::new().expect("Failed to initialize HID API. Ensure you have the necessary permissions to access USB devices.");

    for device_info in api.device_list() {
        log_message(&format!(
            "Device Found -> VID: {:04x}, PID: {:04x}, Product string: {}, Interface: {}, Path: {:?}",
            device_info.vendor_id(),
            device_info.product_id(),
            device_info.product_string().unwrap_or_else(|| "N/A"),
            device_info.interface_number(),
            device_info.path()
        ));
    }

    let mut target_device_path = None;

    // Enumerate every HID endpoint exposed by the glasses
    for device_info in api.device_list() {
        if device_info.vendor_id() == XREAL_VID && device_info.product_id() == XREAL_PID_ONE_PRO {
            log_message(&format!(
                "Found XReal One Pro Endpoint (vendor: {:04x}, product: {:04x}) -> Interface: {}, Path: {:?}", 
                device_info.vendor_id(),
                device_info.product_id(),
                device_info.interface_number(), 
                device_info.path()
            ));

            // Bind explicitly to the interface designated for runtime controls
            if device_info.interface_number() == TARGET_CONTROL_INTERFACE {
                target_device_path = Some(device_info.path().to_owned());
                break;
            }
        }
    }
    
     if let Some(path) = target_device_path {
        log_message(&format!("Opening targeted control interface (Interface {})...", TARGET_CONTROL_INTERFACE));
        match api.open_path(&path) {
            Ok(device) => {
                log_message("XREAL One Pro detected. Sending IMU server activation command...");
                        
                // Execute the strict X1 Chip activation chain
                log_message("[1/2] Broadcasting hardware initialization handshake...");
                send_handshake_a(&device).expect("Failed to send first handshake packet. Device might be unresponsive or already active.");
                std::thread::sleep(Duration::from_millis(150));

                log_message("[2/2] Triggering IP daemon and streaming server engine...");
                send_handshake_b(&device).expect("Failed to send second handshake packet. Device might be unresponsive or already active.");
                
                log_message("\nHID commands successfully delivered! Waking up RNDIS network driver...");
                
                // Loop until the network adapter fully mounts and responds to ping requests
                log_message(&format!("Polling network layer. Awaiting connection to {}...", &config.xreal_addr));
                
                for attempt in 1..=15 {
                    std::thread::sleep(Duration::from_millis(1000));
                    match XrealOne::new_with_addr(&config.xreal_addr) {
                        Ok(_) => {
                            log_message(&format!("\n[SUCCESS] Network stack active!"));
                            tray.inner_mut().set_label("Active", status_label_id).expect("Failed to set status label");
                            break;
                        }
                        Err(_) => {
                            log_message(&format!("Attempt {}: No response from target. Retrying...", attempt));
                        }
                    };
                }
            }
            Err(_) => {
                log_message("Could not connect to USB HID endpoint. (Ensure you have sufficient permissions / udev rules).");
                log_message("Attempting to read UDP stream anyway...");
            }
        }
    } else {
        log_message(&format!("Error: Could not locate Interface {} on your XREAL device.", TARGET_CONTROL_INTERFACE));
        return;
    }

    let config_for_thread = config.clone();
    let running_for_thread = Arc::clone(&running);
    let orientation_for_thread = Arc::clone(&orientation);
    thread::spawn(move || {
        let mut xreal = match XrealOne::new_with_addr(&config_for_thread.xreal_addr) {
            Ok(driver) => driver,
            Err(err) => {
                log_message(&format!("Xreal driver connection failed: {err}"));
                return;
            }
        };

        let mut last_tick = Instant::now();

        log_message(&format!("Connected to Xreal at {}", config_for_thread.xreal_addr));

        let mut first = true;
        while running_for_thread.load(Ordering::Relaxed) {
            match xreal.next() {
                Ok(imu) => {
                    let dt_s = last_tick.elapsed().as_secs_f64();
                    last_tick = Instant::now();
                    let (yaw_deg, roll_deg, pitch_deg) = {
                        let mut state = orientation_for_thread.lock().unwrap();
                        let res = update_orientation(&imu, dt_s, &mut state);
                        if first {
                            drop(state); // Explicitly release the lock before centering
                            let recenter_state = Arc::clone(&orientation);
                            recenter_orientation(&recenter_state);
                            first = false;
                        }
                        res
                    };
                    send_opentrack_packet(&config_for_thread.ip, config_for_thread.port, yaw_deg, roll_deg, pitch_deg);
                }
                Err(err) => {
                    log_message(&format!("Xreal read error: {err}"));
                    thread::sleep(Duration::from_millis(250));
                }
            }
        }
    });

    while running.load(Ordering::Relaxed) {
        thread::sleep(Duration::from_millis(100));
    }
}
