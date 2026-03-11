from __future__ import annotations

from pydantic import BaseModel, ConfigDict
from datetime import datetime
from typing import Optional


class SerialStatusOut(BaseModel):
    connected: bool
    port: str
    last_rx_ts: float
    last_error: str


class NodeOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)
    id: int
    name: str = ""


class FrameOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: int
    node_id: int
    received_at: datetime
    rssi_dbm: Optional[int] = None

    seq: int
    flags: int
    plen: int

    t_ms: Optional[int] = None
    utc_raw_x1e3: Optional[int] = None
    lat_raw_x1e4: Optional[int] = None
    lon_raw_x1e4: Optional[int] = None
    lat_deg: Optional[float] = None
    lon_deg: Optional[float] = None
    sats: Optional[int] = None
    course_cdeg: Optional[int] = None
    temp_mC: Optional[int] = None
    batt_mV: Optional[int] = None
    err_mask: Optional[int] = None

    raw_hex: str


class LatestOut(BaseModel):
    node: NodeOut
    frame: Optional[FrameOut]


class CommandIn(BaseModel):
    command: str
