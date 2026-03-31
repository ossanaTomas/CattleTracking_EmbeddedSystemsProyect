from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, Request
from sqlalchemy.orm import Session
from sqlalchemy import select
from datetime import datetime, timedelta, timezone
from typing import List

from app.db.session import SessionLocal
from app.db import models
from app.api.schemas import SerialStatusOut, NodeOut, FrameOut, LatestOut, CommandIn

router = APIRouter()


def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


@router.get("/status/serial", response_model=SerialStatusOut)
def serial_status(request: Request):
    return request.app.state.serial_manager.status()


@router.get("/nodes", response_model=List[NodeOut])
def list_nodes(db: Session = Depends(get_db)):
    rows = db.execute(select(models.Node).where(models.Node.id > 0).order_by(models.Node.id)).scalars().all()
    return [NodeOut.model_validate(r) for r in rows]


@router.get("/nodes/{node_id}/latest", response_model=LatestOut)
def latest(node_id: int, db: Session = Depends(get_db)):
    node = db.get(models.Node, node_id)
    if not node:
        node = models.Node(id=node_id, name=f"Animal {node_id}")
        db.add(node)
        db.commit()

    fr = db.execute(
        select(models.Frame)
        .where(models.Frame.node_id == node_id)
        .order_by(models.Frame.received_at.desc())
        .limit(1)
    ).scalars().first()

    return LatestOut(
        node=NodeOut.model_validate(node),
        frame=FrameOut.model_validate(fr) if fr else None,
    )


@router.get("/nodes/{node_id}/history", response_model=List[FrameOut])
def history(node_id: int, hours: int = 24, limit: int = 2000, db: Session = Depends(get_db)):
    q = select(models.Frame).where(models.Frame.node_id == node_id)
    if hours > 0:
        since = datetime.utcnow() - timedelta(hours=hours)
        q = q.where(models.Frame.received_at >= since)
    q = q.order_by(models.Frame.received_at.desc()).limit(limit)
    rows = db.execute(q).scalars().all()
    return [FrameOut.model_validate(r) for r in rows]


@router.get("/frames/latest")
def latest_frames(limit: int = 200, db: Session = Depends(get_db)):
    """Latest frames across all nodes (for the UI table)."""
    rows = db.execute(
        select(models.Frame)
        .where(models.Frame.node_id > 0)
        .order_by(models.Frame.received_at.desc())
        .limit(limit)
    ).scalars().all()
    out = []
    for fr in rows:
        t_ms = int(fr.received_at.replace(tzinfo=timezone.utc).timestamp() * 1000)
        out.append({
            "t": t_ms,
            "node_id": fr.node_id,
            "seq": fr.seq,
            "flags": fr.flags,
            "plen": fr.plen,
            "temp_c": (fr.temp_mC/1000.0) if fr.temp_mC is not None else None,
            "batt_v": (fr.batt_mV/1000.0) if fr.batt_mV is not None else None,
            "rssi_dbm": fr.rssi_dbm,
            "err_mask": fr.err_mask,
            "lat": fr.lat_deg,
            "lon": fr.lon_deg,
            "raw": fr.raw_hex,
        })
    return out


@router.get("/field/summary")
def field_summary(request: Request, active_minutes: int = 30, db: Session = Depends(get_db)):
    """Single-call summary for the field dashboard.

    - active_minutes: a node is considered "active" if it has frames within this window.
    """
    now = datetime.utcnow()
    since = now - timedelta(minutes=active_minutes)

    # Load nodes and their latest frame.
    nodes = db.execute(select(models.Node).where(models.Node.id > 0).order_by(models.Node.id)).scalars().all()
    out_nodes = []
    for n in nodes:
        st = db.get(models.NodeSettings, n.id)
        fr = db.execute(
            select(models.Frame)
            .where(models.Frame.node_id == n.id)
            .order_by(models.Frame.received_at.desc())
            .limit(1)
        ).scalars().first()

        last_seen = fr.received_at if fr else None
        last_seen_ms = None
        if last_seen is not None:
            last_seen_ms = int(last_seen.replace(tzinfo=timezone.utc).timestamp() * 1000)

        out_nodes.append({
            "id": n.id,
            "name": n.name,
            "active": bool(fr and fr.received_at >= since),
            "temp_alert_low_c": st.temp_alert_low_c if st else None,
            "temp_alert_high_c": st.temp_alert_high_c if st else None,
            "last_seen_ms": last_seen_ms,
            "last_seen_utc": last_seen.replace(tzinfo=timezone.utc).isoformat().replace('+00:00', 'Z') if last_seen else None,
            "last_seq": fr.seq if fr else None,
            "last_rssi_dbm": fr.rssi_dbm if fr else None,
            "last_temp_c": (fr.temp_mC / 1000.0) if (fr and fr.temp_mC is not None) else None,
            "last_batt_v": (fr.batt_mV / 1000.0) if (fr and fr.batt_mV is not None) else None,
            "last_lat": fr.lat_deg if fr else None,
            "last_lon": fr.lon_deg if fr else None,
            "last_sats": fr.sats if fr else None,
            "last_err_mask": fr.err_mask if fr else None,
        })

    # Runtime stats (drops + estimated gaps)
    rs = getattr(request.app.state, "runtime_stats", None)
    drop = {"total": 0, "last_reason": "", "last_raw": "", "last_at_utc": None}
    gaps = {}
    if rs is not None:
        with rs.lock:
            drop = {
                "total": rs.dropped_total,
                "last_reason": rs.dropped_last_reason,
                "last_raw": rs.dropped_last_raw,
                "last_at_utc": rs.dropped_last_at_utc.isoformat().replace('+00:00','Z') if rs.dropped_last_at_utc else None,
            }
            gaps = dict(rs.gaps_per_node)

    return {
        "serial": request.app.state.serial_manager.status(),
        "drop": drop,
        "gaps": gaps,
        "nodes": out_nodes,
        "active_minutes": active_minutes,
        "server_time_utc": datetime.now(timezone.utc).isoformat().replace('+00:00','Z'),
    }


@router.get("/nodes/{node_id}/temp_series")
def temp_series(node_id: int, hours: int = 24, limit: int = 4000, db: Session = Depends(get_db)):
    """Temperature time series for charts."""
    q = select(models.Frame).where(models.Frame.node_id == node_id).where(models.Frame.temp_mC.is_not(None))
    if hours > 0:
        since = datetime.utcnow() - timedelta(hours=hours)
        q = q.where(models.Frame.received_at >= since)
    q = q.order_by(models.Frame.received_at.asc()).limit(limit)
    rows = db.execute(q).scalars().all()
    out = []
    for fr in rows:
        t_ms = int(fr.received_at.replace(tzinfo=timezone.utc).timestamp() * 1000)
        out.append({
            "t": t_ms,
            "temp_c": fr.temp_mC / 1000.0,
            "rssi_dbm": fr.rssi_dbm,
            "batt_v": (fr.batt_mV / 1000.0) if fr.batt_mV is not None else None,
            "seq": fr.seq,
            "err_mask": fr.err_mask,
        })
    return out


@router.get("/nodes/{node_id}/positions")
def positions(node_id: int, hours: int = 24, limit: int = 4000, db: Session = Depends(get_db)):
    """GPS positions for the map."""
    q = (
        select(models.Frame)
        .where(models.Frame.node_id == node_id)
        .where(models.Frame.lat_deg.is_not(None))
        .where(models.Frame.lon_deg.is_not(None))
    )
    if hours > 0:
        since = datetime.utcnow() - timedelta(hours=hours)
        q = q.where(models.Frame.received_at >= since)
    q = q.order_by(models.Frame.received_at.asc()).limit(limit)
    rows = db.execute(q).scalars().all()
    out = []
    for fr in rows:
        t_ms = int(fr.received_at.replace(tzinfo=timezone.utc).timestamp() * 1000)
        out.append({
            "t": t_ms,
            "lat": fr.lat_deg,
            "lon": fr.lon_deg,
            "temp_c": (fr.temp_mC / 1000.0) if fr.temp_mC is not None else None,
            "rssi_dbm": fr.rssi_dbm,
            "seq": fr.seq,
        })
    return out


@router.get("/nodes/{node_id}/alerts")
def get_alerts(node_id: int, db: Session = Depends(get_db)):
    # ensure node exists
    node = db.get(models.Node, node_id)
    if not node:
        node = models.Node(id=node_id, name=f"Animal {node_id}")
        db.add(node)
        db.commit()

    st = db.get(models.NodeSettings, node_id)
    if not st:
        return {"node_id": node_id, "temp_alert_low_c": None, "temp_alert_high_c": None}
    return {"node_id": node_id, "temp_alert_low_c": st.temp_alert_low_c, "temp_alert_high_c": st.temp_alert_high_c}


@router.post("/nodes/{node_id}/alerts")
def set_alerts(node_id: int, body: dict, db: Session = Depends(get_db)):
    # body: {temp_alert_low_c: float|null, temp_alert_high_c: float|null}
    low = body.get("temp_alert_low_c", None)
    high = body.get("temp_alert_high_c", None)

    node = db.get(models.Node, node_id)
    if not node:
        node = models.Node(id=node_id, name=f"Animal {node_id}")
        db.add(node)
        db.commit()

    st = db.get(models.NodeSettings, node_id)
    if not st:
        st = models.NodeSettings(node_id=node_id)
        db.add(st)

    st.temp_alert_low_c = float(low) if low is not None and str(low).strip() != "" else None
    st.temp_alert_high_c = float(high) if high is not None and str(high).strip() != "" else None
    st.updated_at = datetime.utcnow()
    db.commit()
    return {"ok": True, "node_id": node_id, "temp_alert_low_c": st.temp_alert_low_c, "temp_alert_high_c": st.temp_alert_high_c}


@router.post("/nodes/{node_id}/command")
def send_command(node_id: int, cmd: CommandIn, request: Request):
    # Keep it simple and tolerant: trim and normalize known commands.
    command = (cmd.command or "").strip()
    if not command:
        raise HTTPException(status_code=400, detail="Empty command")

    head = command.split(maxsplit=1)[0].upper()
    if head in {"CONT", "LP"}:
        command = head + (" " + command.split(maxsplit=1)[1] if len(command.split(maxsplit=1)) == 2 else "")

    ok = request.app.state.serial_manager.send_line(command)
    if not ok:
        raise HTTPException(status_code=503, detail="Serial not connected or send failed")
    return {"ok": True, "sent": command, "node_id": node_id}
