from __future__ import annotations
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column, relationship
from sqlalchemy import Integer, String, DateTime, Float, ForeignKey, Text, Boolean, Index
from datetime import datetime

class Base(DeclarativeBase):
    pass

class Node(Base):
    __tablename__ = "nodes"
    id: Mapped[int] = mapped_column(Integer, primary_key=True)
    name: Mapped[str] = mapped_column(String(64), default="")
    created_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)
    frames: Mapped[list["Frame"]] = relationship(back_populates="node")


class NodeSettings(Base):
    """Per-node user configuration.

    Kept in its own table so we don't need to migrate the existing SQLite schema
    (SQLite won't auto-add new columns to an existing table).
    """

    __tablename__ = "node_settings"

    node_id: Mapped[int] = mapped_column(ForeignKey("nodes.id"), primary_key=True)
    temp_alert_low_c: Mapped[float | None] = mapped_column(Float, nullable=True)
    temp_alert_high_c: Mapped[float | None] = mapped_column(Float, nullable=True)
    updated_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)

class Frame(Base):
    __tablename__ = "frames"
    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    node_id: Mapped[int] = mapped_column(ForeignKey("nodes.id"), index=True)
    received_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow, index=True)
    rssi_dbm: Mapped[int | None] = mapped_column(Integer, nullable=True)

    ver: Mapped[int] = mapped_column(Integer)
    net: Mapped[int] = mapped_column(Integer)
    type: Mapped[int] = mapped_column(Integer)
    seq: Mapped[int] = mapped_column(Integer)
    flags: Mapped[int] = mapped_column(Integer)
    plen: Mapped[int] = mapped_column(Integer)

    t_ms: Mapped[int | None] = mapped_column(Integer, nullable=True)
    utc_raw_x1e3: Mapped[int | None] = mapped_column(Integer, nullable=True)
    lat_raw_x1e4: Mapped[int | None] = mapped_column(Integer, nullable=True)
    lon_raw_x1e4: Mapped[int | None] = mapped_column(Integer, nullable=True)
    sats: Mapped[int | None] = mapped_column(Integer, nullable=True)
    course_cdeg: Mapped[int | None] = mapped_column(Integer, nullable=True)
    temp_mC: Mapped[int | None] = mapped_column(Integer, nullable=True)
    batt_mV: Mapped[int | None] = mapped_column(Integer, nullable=True)
    err_mask: Mapped[int | None] = mapped_column(Integer, nullable=True)

    lat_deg: Mapped[float | None] = mapped_column(Float, nullable=True)
    lon_deg: Mapped[float | None] = mapped_column(Float, nullable=True)
    gps_valid: Mapped[bool | None] = mapped_column(Boolean, nullable=True)
    temp_valid: Mapped[bool | None] = mapped_column(Boolean, nullable=True)

    raw_hex: Mapped[str] = mapped_column(Text)

    node: Mapped["Node"] = relationship(back_populates="frames")

Index("ix_frames_node_time", Frame.node_id, Frame.received_at)
