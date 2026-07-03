import threading
import time
import logging

log = logging.getLogger(__name__)

DEMO_SEQUENCE = [
    ("stick",    "HARD_OVER",      8,  "Actuator jam — stick locked at max"),
    ("stick",    "LOSS_OF_SIGNAL", 8,  "Wire cut — stick signal lost"),
    ("throttle", "GAIN_FAILURE",   8,  "ADC scaling error — throttle 1.6x"),
    ("throttle", "BIAS_DRIFT",     10, "Sensor calibration drift — slow upward"),
    ("stick",    "STUCK_AT",       8,  "Mechanical jam — stick frozen"),
    ("cascade",  "throttle_runaway", 14, "CASCADE: Throttle runaway → stick"),
    ("cascade",  "control_failure",  14, "CASCADE: Stick failure → throttle"),
    ("cascade",  "sensor_degradation", 12, "CASCADE: Sensor degradation chain"),
]

HEALTHY_GAP = 8   


class AutoDemoManager:
    def __init__(self, update_fn, cascading_manager_fn):
    
        self._update      = update_fn
        self._get_cascade = cascading_manager_fn

        self._running  = False
        self._thread   = None
        self._lock     = threading.Lock()

       
        self._status = {
            "active":       False,
            "phase":        "idle",      
            "fault_name":   "",
            "description":  "",
            "step":         0,
            "total_steps":  len(DEMO_SEQUENCE),
            "countdown":    0,
        }

    def start(self):
        with self._lock:
            if self._running:
                return
            self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        log.info("Auto demo started")

    def stop(self):
        with self._lock:
            self._running = False
        self._reset_faults()
        with self._lock:
            self._status = {
                "active": False, "phase": "idle",
                "fault_name": "", "description": "",
                "step": 0, "total_steps": len(DEMO_SEQUENCE), "countdown": 0,
            }
        log.info("Auto demo stopped")

    def is_running(self):
        with self._lock:
            return self._running

    def get_status(self) -> dict:
        with self._lock:
            return dict(self._status)

    def _run(self):
        step = 0
        while True:
            with self._lock:
                if not self._running:
                    break

            channel, fault, hold_secs, description = DEMO_SEQUENCE[step]

            log.info(f"Auto demo step {step+1}/{len(DEMO_SEQUENCE)}: {fault}")
            self._inject(channel, fault)
            self._set_status(
                active=True,
                phase="injecting",
                fault_name=fault,
                description=description,
                step=step + 1,
                countdown=hold_secs,
            )

            for remaining in range(hold_secs, 0, -1):
                with self._lock:
                    if not self._running:
                        self._reset_faults()
                        return
                self._set_status(countdown=remaining)
                time.sleep(1.0)

            self._reset_faults()
            self._set_status(
                phase="resetting",
                fault_name=fault,
                description="System recovering — guard clearing",
                countdown=HEALTHY_GAP,
            )

          
            for remaining in range(HEALTHY_GAP, 0, -1):
                with self._lock:
                    if not self._running:
                        return
                self._set_status(countdown=remaining)
                time.sleep(1.0)

            step = (step + 1) % len(DEMO_SEQUENCE)

        self._reset_faults()

    def _inject(self, channel: str, fault: str):
        if channel == "stick":
            self._update(stick_fault=fault)
        elif channel == "throttle":
            self._update(throttle_fault=fault)
        elif channel == "cascade":
            cm = self._get_cascade()
            if cm:
                cm.start_scenario(fault)

    def _reset_faults(self):
        self._update(
            stick_fault="NONE",
            throttle_fault="NONE",
        )
        cm = self._get_cascade()
        if cm:
            cm.stop()

    def _set_status(self, **kwargs):
        with self._lock:
            self._status.update(kwargs)
