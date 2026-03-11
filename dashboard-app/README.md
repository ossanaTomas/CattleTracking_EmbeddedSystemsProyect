# AgroOS / Cattle Tracking Backend (FastAPI + SQLite)

This backend:
- Reads LoRa frames forwarded over USB CDC (COM port) from the Base Station.
- Decodes frames (DATA type 0x10) and stores them in SQLite.
- Exposes REST APIs for latest, history, tracks.
- Provides two UIs:
  - `/debug` (raw frames + decoded fields + errors)
  - `/dashboard` ("Field dashboard": KPIs, temp chart + alert lines, map with points+track, latest samples)

## Run

```bash
python -m venv .venv
# Windows:
#   .venv\Scripts\activate
# Linux/Mac:
#   source .venv/bin/activate
pip install -r requirements.txt

# Set your serial port, e.g. COM3 on Windows, /dev/ttyACM0 on Linux
set SERIAL_PORT=COM3
set SERIAL_BAUD=115200
set DB_PATH=agroos.db

uvicorn app.main:app --host 127.0.0.0 --port 8080
```

Open:
- http://localhost:8080/dashboard
- http://localhost:8080/debug
- http://localhost:8080/mode
- http://localhost:8080/docs

## Notes (important for "it must work")

- **No more "node 0"**: corrupted frames (CRC/length mismatch) are ignored so the UI doesn't get polluted.
  The dashboard still shows counters (drops + estimated seq gaps).
- **Time handling**: the UI uses the server receive timestamp as the source of truth and can display it as
  **Local** or **UTC**.
- **Alert lines**: you can store per-node low/high temperature thresholds. They are drawn as horizontal lines
  on the chart and also used to highlight points on the map.

## Serial input format

Supports Base Station output like:
- `DATA? type=0x10 src=1 dst=0 seq=216 flags=0x10 plen=23 rssi=-49`
- `RX 33B: 01 23 10 01 00 D8 10 17 ...`
Only the `RX ...` line is required; RSSI is taken from the optional DATA line when available.
