import time
import threading
import logging

from server           import create_app, socketio, get_override_state, set_cascading_manager, set_auto_demo_manager, _update
from fault_injector   import FaultInjector
from sensor_guard     import SensorGuard
from fg_bridge        import FlightGearBridge
from cascading_faults import CascadingFaultManager
from ai_analyst       import AIAnalyst
from auto_demo        import AutoDemoManager

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
log = logging.getLogger("main")

SIM_DT        = 0.05
FLASK_PORT    = 5001
STARTUP_GRACE = 3.0


def sim_loop(bridge, injector, guard, cascading, analyst, demo):
    log.info("Loop started")
    t            = 0.0
    startup_time = time.time()

    while True:
        t0 = time.perf_counter()
        t  = round(t + SIM_DT, 2)

        state = bridge.read_state()

        if time.time() - startup_time < STARTUP_GRACE:
            time.sleep(max(0, SIM_DT - (time.perf_counter() - t0)))
            continue

        raw_stick    = state["raw_stick"]
        raw_throttle = state["raw_throttle"]

        cascading.update(state)

        ov = get_override_state()

        if ov["stick_fault"] != injector.stick_fault:
            injector.set_stick_fault(ov["stick_fault"])
        if ov["throttle_fault"] != injector.throttle_fault:
            injector.set_throttle_fault(ov["throttle_fault"])

        fault_active = (
            injector.stick_fault    != "NONE" or
            injector.throttle_fault != "NONE" or
            cascading.active_scenario is not None
        )

        if fault_active:
            inject_stick    = 0.3
            inject_throttle = 0.5
        else:
            inject_stick    = raw_stick
            inject_throttle = raw_throttle

        fr = injector.inject(inject_stick, inject_throttle, dt=SIM_DT)
        gr = guard.process(fr["faulty_stick"], fr["faulty_throttle"],
                           dt=SIM_DT,
                           stick_fault=fr["stick_fault"],
                           throttle_fault=fr["throttle_fault"])

        if fault_active:
            bridge.write_safe_commands(gr["safe_stick"], gr["safe_throttle"])

        if fault_active:
            show_raw_stick     = fr["faulty_stick"]
            show_raw_throttle  = fr["faulty_throttle"]
            show_safe_stick    = gr["safe_stick"]
            show_safe_throttle = gr["safe_throttle"]
            if cascading.active_scenario and gr["status"] == "HEALTHY":
                show_status = "FAILED"
                show_reason = f"CASCADE: {cascading.active_scenario} (stage {cascading._stage})"
            else:
                show_status = gr["status"]
                show_reason = gr["reason"]
        else:
            show_raw_stick     = raw_stick
            show_raw_throttle  = raw_throttle
            show_safe_stick    = raw_stick
            show_safe_throttle = raw_throttle
            show_status        = "HEALTHY"
            show_reason        = ""

        alt = state["altitude_ft"]
        spd = state["airspeed_kts"]

        analyst.push(t, show_raw_stick, show_raw_throttle,
                     fr["stick_fault"], fr["throttle_fault"], show_status)

        demo_status = demo.get_status()

        socketio.emit("telemetry", {
            "t":                t,
            "raw_stick":        round(show_raw_stick,     4),
            "raw_throttle":     round(show_raw_throttle,  4),
            "safe_stick":       round(show_safe_stick,    4),
            "safe_throttle":    round(show_safe_throttle, 4),
            "pitch_q":          round(state.get("pitch_rate_q", 0.0), 3),
            "accel_ax":         round(state.get("accel_ax",     0.0), 2),
            "accel_az":         round(state.get("accel_az",     0.0), 2),
            "altitude_ft":      round(alt, 0) if alt is not None else None,
            "airspeed_kts":     round(spd, 1) if spd is not None else None,
            "groundspeed_kts":  state.get("groundspeed_kts", 0.0),
            "vs_fpm":           state.get("vs_fpm",           0.0),
            "mach":             state.get("mach",             0.0),
            "heading_deg":      state.get("heading_deg",      0.0),
            "pitch_deg":        state.get("pitch_deg",        0.0),
            "roll_deg":         state.get("roll_deg",         0.0),
            "n1":               state.get("n1"),
            "n2":               state.get("n2"),
            "epr":              state.get("epr",              0.0),
            "stick_fault":      fr["stick_fault"],
            "throttle_fault":   fr["throttle_fault"],
            "guard_status":     show_status,
            "guard_reason":     show_reason,
            "fg_connected":     bridge.connected,
            "fault_active":     fault_active,
            "cascade_scenario": cascading.active_scenario or "",
            "cascade_label":    cascading.stage_label if cascading.active_scenario else "",
            "ai":               analyst.get_latest(),
            "fg_write":         bridge.get_write_status(),
            "auto_demo":        demo_status,
        })

        time.sleep(max(0, SIM_DT - (time.perf_counter() - t0)))


def main():
    log.info("FCS PORTAL STARTING")

    bridge = FlightGearBridge()
    if not bridge.connected:
        log.error("FlightGear not connected. Start FG with --telnet=5401 first.")
        return

    injector  = FaultInjector()
    guard     = SensorGuard()
    cascading = CascadingFaultManager(injector)
    set_cascading_manager(cascading)

    analyst = AIAnalyst()
    analyst.start()

    demo = AutoDemoManager(
        update_fn=_update,
        cascading_manager_fn=lambda: cascading,
    )
    set_auto_demo_manager(demo)

    app = create_app()

    threading.Thread(target=sim_loop,
                     args=(bridge, injector, guard, cascading, analyst, demo),
                     daemon=True).start()

    log.info(f"Dashboard → http://localhost:{FLASK_PORT}")
    socketio.run(app, host="0.0.0.0", port=FLASK_PORT, debug=False, use_reloader=False)
    bridge.close()


if __name__ == "__main__":
    main()
