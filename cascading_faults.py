
import time
import logging

log = logging.getLogger(__name__)


class CascadingFaultManager:

    def __init__(self, injector):
        self.injector        = injector
        self.active_scenario = None
        self._start_time     = None
        self._stage          = 0
        self.stage_label     = ""   

    def start_scenario(self, name):
        self.active_scenario = name
        self._start_time     = time.time()
        self._stage          = 0
        self.stage_label     = ""
        log.warning(f"CASCADE START: {name}")

    def stop(self):
        self.active_scenario = None
        self._start_time     = None
        self._stage          = 0
        self.stage_label     = ""
        self.injector.set_stick_fault("NONE")
        self.injector.set_throttle_fault("NONE")
        log.info("CASCADE STOPPED")

    def update(self, state):
        if not self.active_scenario:
            return

        elapsed = time.time() - self._start_time

      
        if self.active_scenario == "throttle_runaway":
            if self._stage == 0:
                self.injector.set_throttle_fault("GAIN_FAILURE")
                self._stage = 1
                self.stage_label = "Stage 1 — Throttle: GAIN_FAILURE"
                log.warning("CASCADE Stage 1: Throttle GAIN_FAILURE")
            elif self._stage == 1 and elapsed > 5:
                self.injector.set_stick_fault("STUCK_AT")
                self._stage = 2
                self.stage_label = "Stage 2 — Throttle: GAIN_FAILURE + Stick: STUCK_AT"
                log.error("CASCADE Stage 2: Stick STUCK_AT")

       
        elif self.active_scenario == "control_failure":
            if self._stage == 0:
                self.injector.set_stick_fault("HARD_OVER")
                self._stage = 1
                self.stage_label = "Stage 1 — Stick: HARD_OVER"
                log.warning("CASCADE Stage 1: Stick HARD_OVER")
            elif self._stage == 1 and elapsed > 5:
                self.injector.set_throttle_fault("BIAS_DRIFT")
                self._stage = 2
                self.stage_label = "Stage 2 — Stick: HARD_OVER + Throttle: BIAS_DRIFT"
                log.error("CASCADE Stage 2: Throttle BIAS_DRIFT")

       
        elif self.active_scenario == "sensor_degradation":
            if self._stage == 0:
                self.injector.set_stick_fault("LOSS_OF_SIGNAL")
                self._stage = 1
                self.stage_label = "Stage 1 — Stick: LOSS_OF_SIGNAL"
                log.warning("CASCADE Stage 1: Stick LOSS_OF_SIGNAL")
            elif self._stage == 1 and elapsed > 3:
                self.injector.set_throttle_fault("GAIN_FAILURE")
                self._stage = 2
                self.stage_label = "Stage 2 — Stick: LOSS_OF_SIGNAL + Throttle: GAIN_FAILURE"
                log.error("CASCADE Stage 2: Throttle GAIN_FAILURE")