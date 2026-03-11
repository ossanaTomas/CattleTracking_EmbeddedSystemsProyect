from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Dict, Tuple
from .crc16 import crc16_ccitt_false

HDR_LEN = 8
CRC_LEN = 2

TYPE_DATA = 0x10

FLAG_GPS_VALID  = 0x08
FLAG_TEMP_VALID = 0x10
FLAG_BATT_VALID = 0x20

ERR_GPS_NO_FIX = 1 << 0
ERR_TEMP_FAIL  = 1 << 3

def u16_le(b: bytes, o: int) -> int:
    return b[o] | (b[o+1] << 8)

def u32_le(b: bytes, o: int) -> int:
    return b[o] | (b[o+1] << 8) | (b[o+2] << 16) | (b[o+3] << 24)

def i32_le(b: bytes, o: int) -> int:
    v = u32_le(b, o)
    return v - 0x100000000 if v & 0x80000000 else v

@dataclass
class FrameHdr:
    ver: int
    net: int
    type: int
    src: int
    dst: int
    seq: int
    flags: int
    plen: int

@dataclass
class DataV1:
    t_ms: int
    utc_raw_x1e3: int
    lat_raw_x1e4: int
    lon_raw_x1e4: int
    sats: int
    course_cdeg: int
    temp_mC: int
    batt_mV: int
    err_mask: int

def raw_x1e4_to_decimal(raw: int) -> Optional[float]:
    if raw == 0:
        return None
    sign = -1.0 if raw < 0 else 1.0
    r = abs(raw)
    whole = r // 10000  # ddmm or dddmm
    frac = r % 10000
    deg = whole // 100
    mm = whole % 100
    minutes = mm + (frac / 10000.0)
    return sign * (deg + minutes / 60.0)

def unpack_frame(buf: bytes, verify_crc: bool = True) -> Tuple[FrameHdr, bytes]:
    if len(buf) < HDR_LEN + CRC_LEN:
        raise ValueError("frame too short")
    ver, net, typ, src, dst, seq, flags, plen = buf[:8]
    total = HDR_LEN + plen + CRC_LEN
    if len(buf) != total:
        raise ValueError(f"length mismatch: got={len(buf)} expected={total} plen={plen}")
    if verify_crc:
        rx_crc = u16_le(buf, HDR_LEN + plen)
        calc = crc16_ccitt_false(buf[:HDR_LEN + plen])
        if rx_crc != calc:
            raise ValueError("CRC mismatch")
    return FrameHdr(ver, net, typ, src, dst, seq, flags, plen), buf[HDR_LEN:HDR_LEN+plen]

def parse_data(h: FrameHdr, payload: bytes) -> DataV1:
    if h.plen == 23:
        t_ms = u32_le(payload, 0)
        lat  = i32_le(payload, 4)
        lon  = i32_le(payload, 8)
        sats = payload[12]
        course = u16_le(payload, 13)
        temp_mC = i32_le(payload, 15)
        batt_mV = u16_le(payload, 19)
        err = u16_le(payload, 21)
        return DataV1(t_ms, None, lat, lon, sats, course, temp_mC, batt_mV, err)
    if h.plen == 27:
        t_ms = u32_le(payload, 0)
        utc = u32_le(payload, 4)
        lat  = i32_le(payload, 8)
        lon  = i32_le(payload, 12)
        sats = payload[16]
        course = u16_le(payload, 17)
        temp_mC = i32_le(payload, 19)
        batt_mV = u16_le(payload, 23)
        err = u16_le(payload, 25)
        return DataV1(t_ms, utc, lat, lon, sats, course, temp_mC, batt_mV, err)
    raise ValueError(f"unsupported DATA plen={h.plen}")
