import threading
import logging
from flask import Flask, render_template
from flask_socketio import SocketIO

log = logging.getLogger(__name__)

cascading_manager = None
auto_demo_manager = None

def set_cascading_manager(mgr):
    global cascading_manager
    cascading_manager = mgr

def set_auto_demo_manager(mgr):
    global auto_demo_manager
    auto_demo_manager = mgr

socketio = SocketIO(
    cors_allowed_origins="*",
    async_mode="threading",
    logger=False,
    engineio_logger=False,
)

_lock  = threading.Lock()
_state = {
    "stick_override_active":    False,
    "throttle_override_active": False,
    "stick_value":              0.0,
    "throttle_value":           0.5,
    "stick_fault":              "NONE",
    "throttle_fault":           "NONE",
}

def get_override_state():
    with _lock:
        return dict(_state)

def _update(**kwargs):
    with _lock:
        _state.update(kwargs)

def create_app():
    app = Flask(__name__, template_folder="templates")
    app.config["SECRET_KEY"] = "fcs-secret"
    socketio.init_app(app)

    @app.route("/")
    def index():
        return render_template("dashboard.html")

    @socketio.on("connect")
    def on_connect():
        log.info("Dashboard connected")

    @socketio.on("disconnect")
    def on_disconnect():
        log.info("Dashboard disconnected")

    @socketio.on("set_stick_fault")
    def on_stick_fault(data):
        _update(stick_fault=data.get("fault", "NONE"))

    @socketio.on("set_throttle_fault")
    def on_throttle_fault(data):
        _update(throttle_fault=data.get("fault", "NONE"))

    @socketio.on("set_stick_override")
    def on_stick_override(data):
        _update(
            stick_override_active=data.get("active", False),
            stick_value=float(data.get("value", 0.0)),
        )

    @socketio.on("set_throttle_override")
    def on_throttle_override(data):
        _update(
            throttle_override_active=data.get("active", False),
            throttle_value=float(data.get("value", 0.5)),
        )

    @socketio.on("start_cascading")
    def on_cascade(data):
        if cascading_manager:
            cascading_manager.start_scenario(data.get("scenario", ""))

    @socketio.on("reset_all")
    def on_reset():
        # Stop auto demo if running
        if auto_demo_manager and auto_demo_manager.is_running():
            auto_demo_manager.stop()
        _update(
            stick_override_active=False,
            throttle_override_active=False,
            stick_value=0.0,
            throttle_value=0.5,
            stick_fault="NONE",
            throttle_fault="NONE",
        )
        if cascading_manager:
            cascading_manager.stop()
        log.info("Full reset")

    @socketio.on("start_auto_demo")
    def on_start_auto_demo():
        if auto_demo_manager:
            if not auto_demo_manager.is_running():
                auto_demo_manager.start()
                log.info("Auto demo started from dashboard")

    @socketio.on("stop_auto_demo")
    def on_stop_auto_demo():
        if auto_demo_manager:
            auto_demo_manager.stop()
            _update(stick_fault="NONE", throttle_fault="NONE")
            log.info("Auto demo stopped from dashboard")

    return app
