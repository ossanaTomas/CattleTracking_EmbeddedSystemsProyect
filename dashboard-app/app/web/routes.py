from fastapi import APIRouter, Request
from fastapi.templating import Jinja2Templates
from app.core.settings import settings
from pathlib import Path

router = APIRouter()
templates = Jinja2Templates(directory=str(Path(__file__).parent / "templates"))

@router.get("/debug")
def debug_page(request: Request):
    return templates.TemplateResponse("debug.html", {"request": request, "title": settings.dashboard_title})

@router.get("/dashboard")
def dash_page(request: Request):
    return templates.TemplateResponse("dashboard.html", {"request": request, "title": settings.dashboard_title})


@router.get("/mode")
def mode_page(request: Request):
    return templates.TemplateResponse("mode.html", {"request": request, "title": settings.dashboard_title})
