"""ASGI app entrypoint for local/backend-only execution.

This module simply re-exports the FastAPI `app` defined in
`smart_gui.ros2_inspector_api` so tools like `uvicorn app.main:app` can run it.
"""

from smart_gui.ros2_inspector_api import app
