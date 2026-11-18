//! Compile-time configuration

pub const AP_SSID: &str = "esp";
pub const AP_PASS: &str = "12345678";
pub const AP_CHANNEL: u8 = 6;

pub const PORTAL_IP: [u8; 4] = [192, 168, 4, 1];
pub const HTTP_PORT: u16 = 80;

pub const RESET_HOLD_MS: u64 = 3000;

/// NatNet command port on the Motive PC (unicast connect requests)
pub const NATNET_COMMAND_PORT: u16 = 1510;
/// Client name announced to Motive
pub const NATNET_CLIENT_NAME: &str = "esp32_px4_mocap";
/// Unicast: re-send the connect request this often while streaming, as a keep-alive
pub const NATNET_KEEPALIVE_SECS: u32 = 5;
