from pydantic import BaseModel
import os


class Settings(BaseModel):
    serial_port: str = os.getenv("SERIAL_PORT", "")
    serial_baud: int = int(os.getenv("SERIAL_BAUD", "115200"))
    serial_timeout_s: float = float(os.getenv("SERIAL_TIMEOUT_S", "0.2"))
    serial_reconnect_s: float = float(os.getenv("SERIAL_RECONNECT_S", "1.0"))

    db_path: str = os.getenv("DB_PATH", "agroos.db")

    dashboard_title: str = os.getenv("DASHBOARD_TITLE", "Cow Tracking System")

settings = Settings()

