//! NatNet frame-of-data parser

#![cfg_attr(not(test), no_std)]

/// NatNet message id for a frame of mocap data
pub const NAT_FRAMEOFDATA: u16 = 7;
/// Client -> server: connect / ping (server adds us as a unicast client)
pub const NAT_CONNECT: u16 = 0;
/// Server -> client: reply to NAT_CONNECT with the server's name and versions
pub const NAT_SERVERINFO: u16 = 1;

/// Length of a connect request built by [`connect_request`]
pub const CONNECT_REQUEST_LEN: usize = 4 + SENDER_LEN;
/// sSender: name[256] + app version[4] + NatNet version[4]
const SENDER_LEN: usize = 256 + 4 + 4;

/// Build a NAT_CONNECT request announcing `name` and the NatNet `version` we parse
pub fn connect_request(name: &str, version: Version, out: &mut [u8; CONNECT_REQUEST_LEN]) {
    out.fill(0);
    out[0..2].copy_from_slice(&NAT_CONNECT.to_le_bytes());
    out[2..4].copy_from_slice(&(SENDER_LEN as u16).to_le_bytes());
    let n = name.len().min(255); // keep the NUL terminator
    out[4..4 + n].copy_from_slice(&name.as_bytes()[..n]);
    out[4 + 260] = version.major;
    out[4 + 261] = version.minor;
}

/// Server info from a NAT_SERVERINFO reply
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ServerInfo<'a> {
    pub name: &'a str,
    pub app_version: [u8; 4],
    pub natnet_version: [u8; 4],
}

/// Parse a NAT_SERVERINFO reply
pub fn parse_server_info(data: &[u8]) -> Option<ServerInfo<'_>> {
    let mut r = ByteReader::new(data);
    if r.u16()? != NAT_SERVERINFO {
        return None;
    }
    r.u16()?;
    let name = r.take(256)?;
    let name = &name[..name.iter().position(|&b| b == 0).unwrap_or(256)];
    Some(ServerInfo {
        name: core::str::from_utf8(name).unwrap_or("?"),
        app_version: r.take(4)?.try_into().ok()?,
        natnet_version: r.take(4)?.try_into().ok()?,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Version {
    pub major: u8,
    pub minor: u8,
}

impl Version {
    pub const fn new(major: u8, minor: u8) -> Self {
        Self { major, minor }
    }

    /// Pre-3.0 embeds each rigid body's marker data inline
    fn has_embedded_rb_markers(self) -> bool {
        self.major < 3
    }

    /// 2.0+ appends the mean marker error
    fn has_mean_error(self) -> bool {
        self.major >= 2
    }

    /// 2.6+ appends the params bitfield (bit 0 = tracking valid)
    fn has_params(self) -> bool {
        self.major > 2 || (self.major == 2 && self.minor >= 6)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigidBody {
    pub id: i32,
    pub pos: [f32; 3],
    pub quat: [f32; 4],
    pub valid: bool,
}

struct ByteReader<'a> {
    data: &'a [u8],
    off: usize,
}

impl<'a> ByteReader<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, off: 0 }
    }

    fn take(&mut self, n: usize) -> Option<&'a [u8]> {
        let end = self.off.checked_add(n)?;
        let s = self.data.get(self.off..end)?;
        self.off = end;
        Some(s)
    }

    fn skip(&mut self, n: usize) -> Option<()> {
        self.take(n).map(|_| ())
    }

    fn u16(&mut self) -> Option<u16> {
        self.take(2).map(|b| u16::from_le_bytes([b[0], b[1]]))
    }

    fn i16(&mut self) -> Option<i16> {
        self.take(2).map(|b| i16::from_le_bytes([b[0], b[1]]))
    }

    fn i32(&mut self) -> Option<i32> {
        self.take(4).map(|b| i32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn f32(&mut self) -> Option<f32> {
        self.take(4).map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn count(&mut self) -> Option<usize> {
        let v = self.i32()?;
        usize::try_from(v).ok()
    }

    fn skip_string(&mut self) -> Option<()> {
        let rest = self.data.get(self.off..)?;
        let nul = rest.iter().position(|&b| b == 0)?;
        self.off += nul + 1;
        Some(())
    }
}

/// Parse a NatNet datagram, returning the rigid body with id `target_id`, or `None` if it's not a well-formed frame-of-data containing it
pub fn parse_frame(data: &[u8], version: Version, target_id: i32) -> Option<RigidBody> {
    let mut r = ByteReader::new(data);

    // header: messageID + byte count
    if r.u16()? != NAT_FRAMEOFDATA {
        return None;
    }
    r.u16()?; // nDataBytes
    r.i32()?; // frame number

    // marker sets: name + count + xyz floats
    let n_marker_sets = r.count()?;
    for _ in 0..n_marker_sets {
        r.skip_string()?;
        let n_markers = r.count()?;
        r.skip(n_markers.checked_mul(3 * 4)?)?;
    }

    // legacy "other" markers
    let n_other = r.count()?;
    r.skip(n_other.checked_mul(3 * 4)?)?;

    // rigid bodies
    let n_rigid = r.count()?;
    let mut found: Option<RigidBody> = None;

    for _ in 0..n_rigid {
        let id = r.i32()?;
        let pos = [r.f32()?, r.f32()?, r.f32()?];
        let quat = [r.f32()?, r.f32()?, r.f32()?, r.f32()?];

        if version.has_embedded_rb_markers() {
            // pre-3.0: associated markers embedded here
            let n_rb_markers = r.count()?;
            r.skip(n_rb_markers.checked_mul(3 * 4)?)?; // positions
            if version.major >= 2 {
                r.skip(n_rb_markers.checked_mul(4)?)?; // ids
                r.skip(n_rb_markers.checked_mul(4)?)?; // sizes
            }
        }

        if version.has_mean_error() {
            r.f32()?; // mean marker error
        }

        let valid = if version.has_params() {
            (r.i16()? & 0x01) != 0
        } else {
            true
        };

        if id == target_id {
            found = Some(RigidBody { id, pos, quat, valid });
        }
    }

    found
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn connect_request_layout() {
        let mut buf = [0u8; CONNECT_REQUEST_LEN];
        connect_request("esp", Version::new(2, 7), &mut buf);
        assert_eq!(&buf[0..4], &[0, 0, 0x08, 0x01]); // id 0, 264 bytes
        assert_eq!(&buf[4..8], b"esp\0");
        assert_eq!(&buf[264..268], &[2, 7, 0, 0]);
    }

    #[test]
    fn server_info_roundtrip() {
        let mut d = [0u8; 4 + 264];
        d[0] = 1;
        d[2..4].copy_from_slice(&264u16.to_le_bytes());
        d[4..10].copy_from_slice(b"Motive");
        d[260..264].copy_from_slice(&[1, 7, 0, 0]);
        d[264..268].copy_from_slice(&[2, 7, 0, 0]);
        let info = parse_server_info(&d).unwrap();
        assert_eq!(info.name, "Motive");
        assert_eq!(info.app_version, [1, 7, 0, 0]);
        assert_eq!(info.natnet_version, [2, 7, 0, 0]);
        assert!(parse_frame(&d, Version::new(2, 7), 1).is_none());
    }
}
