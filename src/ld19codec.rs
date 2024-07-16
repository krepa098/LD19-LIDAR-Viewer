use byteorder::{LittleEndian, ReadBytesExt};
use std::{
    io::Cursor,
    mem::{offset_of, size_of},
};
use tokio::io;
use tokio_util::{bytes::BytesMut, codec::Decoder};

#[repr(packed)]
#[derive(Clone, Copy)]
pub struct Ld19Point {
    distance: u16,
    intensity: u8,
}

impl Ld19Point {
    pub fn from_bytes(cursor: &mut Cursor<BytesMut>) -> Self {
        Ld19Point {
            distance: cursor.read_u16::<LittleEndian>().unwrap(),
            intensity: cursor.read_u8().unwrap(),
        }
    }

    pub fn distance_in_meters(&self) -> f32 {
        self.distance as f32 * 1e-3
    }

    pub fn normalized_intensity(&self) -> f32 {
        self.intensity as f32 / 255.0
    }
}

#[repr(packed)]
#[allow(unused)]
#[derive(Copy, Clone)]
pub struct Ld19Packet {
    header: u8,
    ver_len: u8,
    speed: u16,
    start_angle: u16,
    point: [Ld19Point; 12],
    end_angle: u16,
    timestamp: u16,
    crc8: u8,
}

const PKG_SIZE: usize = size_of::<Ld19Packet>();

#[allow(unused)]
impl Ld19Packet {
    pub fn from_bytes(cursor: &mut Cursor<BytesMut>) -> Self {
        Ld19Packet {
            header: cursor.read_u8().unwrap(),
            ver_len: cursor.read_u8().unwrap(),
            speed: cursor.read_u16::<LittleEndian>().unwrap(),
            start_angle: cursor.read_u16::<LittleEndian>().unwrap(),
            point: core::array::from_fn(|_| Ld19Point::from_bytes(cursor)),
            end_angle: cursor.read_u16::<LittleEndian>().unwrap(),
            timestamp: cursor.read_u16::<LittleEndian>().unwrap(),
            crc8: cursor.read_u8().unwrap(),
        }
    }

    pub fn start_angle_deg(&self) -> f32 {
        self.start_angle as f32 * 1e-2
    }

    pub fn end_angle_deg(&self) -> f32 {
        self.end_angle as f32 * 1e-2
    }

    pub fn delta_angle_deg(&self) -> f32 {
        let delta = (self.end_angle_deg() - self.start_angle_deg()).abs() % 360.0;

        if delta > 180.0 {
            return 360.0 - delta;
        }

        delta
    }

    pub fn timestamp(&self) -> std::time::Duration {
        std::time::Duration::from_millis(self.timestamp as u64)
    }

    pub fn iter_points(&self) -> Ld19PointIter {
        Ld19PointIter {
            packet: self,
            index: 0,
        }
    }
}

pub struct Ld19PointIter<'a> {
    packet: &'a Ld19Packet,
    index: usize,
}

impl<'a> Iterator for Ld19PointIter<'a> {
    type Item = (f32, &'a Ld19Point);

    fn next(&mut self) -> Option<Self::Item> {
        let step = self.packet.delta_angle_deg() / (self.packet.point.len() - 1) as f32;
        let angle = (self.packet.start_angle_deg() + self.index as f32 * step) % 360.0;

        let item = self.packet.point.get(self.index).map(|p| (angle, p));
        self.index += 1;

        item
    }
}

pub struct Ld19Codec {}

impl Decoder for Ld19Codec {
    type Item = Ld19Packet;
    type Error = io::Error;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        let start_pos = src.as_ref().iter().position(|b| *b == 0x54);

        if let Some(start_pos) = start_pos {
            // enough data for a full packet?
            if src.as_ref().len() - start_pos > PKG_SIZE {
                let packet_data = &src.as_ref()[start_pos..start_pos + PKG_SIZE];

                // check crc
                let crc_indicated = packet_data[offset_of!(Ld19Packet, crc8)];
                let crc_calculated = crc8(&packet_data[0..packet_data.len() - 1]);

                if crc_calculated == crc_indicated {
                    // remove packet data from the buffer
                    let data = src.split_to(start_pos + PKG_SIZE);

                    // decode the packet
                    let mut cursor = Cursor::new(data);
                    let packet = Ld19Packet::from_bytes(&mut cursor);

                    return Ok(Some(packet));
                } else {
                    println!("crc mismatch {}", src.len());
                    // crc mismatch
                    // clear previous including start_pos
                    let _ = src.split_to(start_pos + 1);
                }
            }
        } else {
            // no start byte found, clear the buffer
            src.clear();
        }

        // more data needed
        Ok(None)
    }
}

fn crc8(data: &[u8]) -> u8 {
    let mut crc = 0;
    for byte in data {
        crc = CRC_TABLE[(crc ^ *byte) as usize];
    }

    crc
}

const CRC_TABLE: [u8; 256] = [
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8,
];
