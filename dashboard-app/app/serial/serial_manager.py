from __future__ import annotations
import threading, time, re
from dataclasses import dataclass
from typing import Optional, Callable
import serial

# Used for best-effort auto-detection when SERIAL_PORT isn't set.
from serial.tools import list_ports

from app.core.settings import settings

HEX_LINE_RE = re.compile(r"^RX\s+(\d+)B:\s+(.*)$")
DATA_LINE_RE = re.compile(r"\brssi=([-\d]+)\b", re.IGNORECASE)

@dataclass
class SerialStatus:
    connected: bool
    port: str
    last_rx_ts: float
    last_error: str

class SerialManager:
    def __init__(self, on_frame: Callable[[bytes, Optional[int], str], None]):
        self._on_frame = on_frame
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._ser: Optional[serial.Serial] = None
        self._connected_port: str = ""
        self._last_rssi: Optional[int] = None
        self._last_rx_ts = 0.0
        self._last_error = ""
        self._tx_lock = threading.Lock()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass

    def status(self) -> SerialStatus:
        return SerialStatus(
            connected=bool(self._ser and self._ser.is_open),
            port=self._connected_port or settings.serial_port,
            last_rx_ts=self._last_rx_ts,
            last_error=self._last_error,
        )

    def send_line(self, line: str) -> bool:
        # Base Station parser is tolerant, but CRLF keeps behavior consistent across OSes.
        line = line.strip("\r\n") + "\r\n"
        with self._tx_lock:
            try:
                if not self._ser or not self._ser.is_open:
                    return False
                self._ser.write(line.encode("ascii", errors="ignore"))
                self._ser.flush()
                return True
            except Exception as e:
                self._last_error = f"send failed: {e}"
                return False

    def _guess_port(self) -> str:
        """Best-effort auto-detection when SERIAL_PORT is not set.

        Keeps the project 'plug-and-run' for the common case (STM32 USB CDC).
        """
        ports = list(list_ports.comports())
        if not ports:
            return ""

        def score(p) -> int:
            desc = (getattr(p, "description", "") or "").lower()
            hwid = (getattr(p, "hwid", "") or "").lower()
            s = 0
            for key, w in [
                ("stmicro", 8),
                ("stm32", 8),
                ("stm", 5),
                ("cdc", 4),
                ("usb", 2),
                ("serial", 1),
            ]:
                if key in desc or key in hwid:
                    s += w
            if getattr(p, "vid", None) is not None and getattr(p, "pid", None) is not None:
                s += 1
            return s

        ports.sort(key=score, reverse=True)
        return ports[0].device

    def _open(self) -> bool:
        port = settings.serial_port or self._guess_port()
        if not port:
            self._last_error = "SERIAL_PORT not set and no serial ports detected"
            return False
        try:
            self._ser = serial.Serial(
                port,
                settings.serial_baud,
                timeout=settings.serial_timeout_s,
                write_timeout=1.0,
            )
            try:
                self._ser.setDTR(True)
            except Exception:
                pass
            self._connected_port = port
            self._last_error = ""
            return True
        except Exception as e:
            self._last_error = f"open failed: {e}"
            self._ser = None
            self._connected_port = ""
            return False

    def _run(self) -> None:
        while not self._stop.is_set():
            if not self._ser or not self._ser.is_open:
                if not self._open():
                    time.sleep(settings.serial_reconnect_s)
                    continue

            try:
                line = self._ser.readline()
                if not line:
                    continue
                s = line.decode("utf-8", errors="replace").strip()

                m = DATA_LINE_RE.search(s)
                if m:
                    try:
                        self._last_rssi = int(m.group(1))
                    except Exception:
                        pass

                m = HEX_LINE_RE.match(s)
                if m:
                    nbytes = int(m.group(1))
                    hexpart = m.group(2).strip()
                    toks = [t for t in hexpart.split() if len(t) == 2]
                    if len(toks) < nbytes:
                        # Incomplete line (rare). Ignore instead of throwing.
                        continue
                    raw = bytes(int(t, 16) for t in toks[:nbytes])
                    self._last_rx_ts = time.time()
                    self._on_frame(raw, self._last_rssi, s)
                    self._last_rssi = None

            except (serial.SerialException, OSError) as e:
                self._last_error = f"serial error: {e}"
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
                time.sleep(settings.serial_reconnect_s)
            except Exception as e:
                self._last_error = f"parse error: {e}"
