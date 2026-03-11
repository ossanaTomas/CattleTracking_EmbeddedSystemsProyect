from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from .models import Base
from app.core.settings import settings

engine = create_engine(f"sqlite:///{settings.db_path}", connect_args={"check_same_thread": False})
SessionLocal = sessionmaker(bind=engine, autoflush=False, autocommit=False)

def init_db() -> None:
    Base.metadata.create_all(engine)
