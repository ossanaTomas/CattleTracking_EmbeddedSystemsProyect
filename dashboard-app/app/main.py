from __future__ import annotations

from fastapi import FastAPI
from dataclasses import dataclass, field
from threading import Lock
from collections import defaultdict
from datetime import datetime, timezone
import time
from app.db.session import init_db, SessionLocal
from app.db import models
from app.serial.serial_manager import SerialManager
from app.proto.frame import (
    unpack_frame, parse_data, raw_x1e4_to_decimal,
    TYPE_DATA, FLAG_GPS_VALID, FLAG_TEMP_VALID
)

app = FastAPI(title="Cow Tracker Backend", version="0.1.1")


@dataclass
class RuntimeStats:
    lock: Lock = field(default_factory=Lock)
    dropped_total: int = 0
    dropped_last_reason: str = ""
    dropped_last_raw: str = ""
    dropped_last_at_utc: datetime | None = None
    gaps_per_node: dict[int, int] = field(default_factory=lambda: defaultdict(int))
    last_seq_per_node: dict[int, int] = field(default_factory=dict)
    last_seen_seq_ts: dict[tuple[int,int], float] = field(default_factory=dict)


runtime_stats = RuntimeStats()

# The node may send the same DATA frame twice as a simple redundancy measure.
# We deduplicate back-to-back repeats so the DB/UI doesn't show duplicated rows.
# If the same seq appears again after a longer time (e.g., node reboot), we keep it.
DEDUP_WINDOW_S = 5.0


def _store_frame(raw: bytes, rssi: int | None, line: str) -> None:
    """Called from SerialManager background thread.

    Stores raw frames always; if CRC+layout are valid, also stores decoded fields.
    """
    hexstr = raw.hex(" ").upper()
    db = SessionLocal()
    try:
        try:
            hdr, payload = unpack_frame(raw, verify_crc=True)
        except Exception as e:
            # For a "make it work" demo we simply ignore corrupted frames so the UI
            # doesn't get polluted with "node 0".
            with runtime_stats.lock:
                runtime_stats.dropped_total += 1
                runtime_stats.dropped_last_reason = str(e)
                runtime_stats.dropped_last_raw = hexstr[:160]
                runtime_stats.dropped_last_at_utc = datetime.now(timezone.utc)
            return

        # Keep only DATA frames (the field UI is focused on telemetry).
        if hdr.type != TYPE_DATA:
            return

        # src=0 is not a field node (it's reserved for base). Ignore to avoid confusion.
        if hdr.src == 0:
            return

        # Drop duplicated DATA frames (same node, same seq) arriving back-to-back.
        # This is expected when the node repeats a frame for redundancy.
        now_ts = time.time()
        with runtime_stats.lock:
            last_seq = runtime_stats.last_seq_per_node.get(hdr.src)
            if last_seq is not None and hdr.seq == last_seq:
                last_seen = runtime_stats.last_seen_seq_ts.get((hdr.src, hdr.seq), 0.0)
                if (now_ts - last_seen) <= DEDUP_WINDOW_S:
                    runtime_stats.dropped_total += 1
                    runtime_stats.dropped_last_reason = "duplicate seq (redundant TX)"
                    runtime_stats.dropped_last_raw = hexstr[:160]
                    runtime_stats.dropped_last_at_utc = datetime.now(timezone.utc)
                    # Update timestamp so repeated repeats within the window are also ignored.
                    runtime_stats.last_seen_seq_ts[(hdr.src, hdr.seq)] = now_ts
                    return
            runtime_stats.last_seen_seq_ts[(hdr.src, hdr.seq)] = now_ts

        node = db.get(models.Node, hdr.src)
        if not node:
            node = models.Node(id=hdr.src, name=f"Animal {hdr.src}")
            db.add(node)
            db.commit()

        fr = models.Frame(
            node_id=node.id,
            rssi_dbm=rssi,
            ver=hdr.ver,
            net=hdr.net,
            type=hdr.type,
            seq=hdr.seq,
            flags=hdr.flags,
            plen=hdr.plen,
            raw_hex=hexstr,
        )

        d = parse_data(hdr, payload)
        fr.t_ms = d.t_ms
        fr.utc_raw_x1e3 = d.utc_raw_x1e3
        fr.lat_raw_x1e4 = d.lat_raw_x1e4
        fr.lon_raw_x1e4 = d.lon_raw_x1e4
        fr.sats = d.sats
        fr.course_cdeg = d.course_cdeg
        fr.temp_mC = d.temp_mC
        fr.batt_mV = d.batt_mV
        fr.err_mask = d.err_mask

        fr.gps_valid = bool(hdr.flags & FLAG_GPS_VALID)
        fr.temp_valid = bool(hdr.flags & FLAG_TEMP_VALID)
        fr.lat_deg = raw_x1e4_to_decimal(d.lat_raw_x1e4)
        fr.lon_deg = raw_x1e4_to_decimal(d.lon_raw_x1e4)

        # Track sequence gaps (best-effort; packets can be lost on LoRa).
        with runtime_stats.lock:
            if hdr.src in runtime_stats.last_seq_per_node:
                last = runtime_stats.last_seq_per_node[hdr.src]
                diff = (hdr.seq - last) & 0xFF
                if diff > 1:
                    runtime_stats.gaps_per_node[hdr.src] += (diff - 1)
            runtime_stats.last_seq_per_node[hdr.src] = hdr.seq

        db.add(fr)
        db.commit()
    finally:
        db.close()


@app.on_event("startup")
def on_startup() -> None:
    init_db()
    # Store SerialManager on app.state to avoid circular imports.
    app.state.serial_manager = SerialManager(on_frame=_store_frame)
    app.state.runtime_stats = runtime_stats
    app.state.serial_manager.start()


@app.on_event("shutdown")
def on_shutdown() -> None:
    sm = getattr(app.state, "serial_manager", None)
    if sm:
        sm.stop()


from app.api.routes import router as api_router
from app.web.routes import router as web_router

app.include_router(api_router, prefix="/api")
app.include_router(web_router)


@app.get("/")
def root():
    return {"ok": True, "dashboard": "/dashboard", "debug": "/debug", "docs": "/docs"}
