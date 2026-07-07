import time

class StickFault:
    NONE            = "NONE"
    LOSS_OF_SIGNAL  = "LOSS_OF_SIGNAL"
    HARD_OVER       = "HARD_OVER"
    STUCK_AT        = "STUCK_AT"

class ThrottleFault:
    NONE            = "NONE"
    BIAS_DRIFT      = "BIAS_DRIFT"
    GAIN_FAILURE    = "GAIN_FAILURE"

class FaultInjector:


    def __init__(
        self,
        stick_fault: str    = StickFault.NONE,
        throttle_fault: str = ThrottleFault.NONE,
        hard_over_value: float = 1.0,
        bias_rate: float       = 0.08,   
        gain_factor: float     = 1.6,
    ):
        self.stick_fault    = stick_fault
        self.throttle_fault = throttle_fault
        self.hard_over_value = hard_over_value
        self.bias_rate      = bias_rate
        self.gain_factor    = gain_factor

        
        self._stuck_value   = None          
        self._fault_start   = time.time()   
        self._bias_accum    = 0.0          

   

    def set_stick_fault(self, fault_type: str):
       
        self.stick_fault  = fault_type
        self._stuck_value = None
        self._fault_start = time.time()

    def set_throttle_fault(self, fault_type: str):
        
        self.throttle_fault = fault_type
        self._bias_accum    = 0.0
        self._fault_start   = time.time()

    def inject(
        self,
        raw_stick: float,
        raw_throttle: float,
        dt: float = 0.02,
    ) -> dict:
       
        faulty_stick    = self._apply_stick_fault(raw_stick)
        faulty_throttle = self._apply_throttle_fault(raw_throttle, dt)

        return {
            "faulty_stick":    round(faulty_stick,    4),
            "faulty_throttle": round(faulty_throttle, 4),
            "stick_fault":     self.stick_fault,
            "throttle_fault":  self.throttle_fault,
        }

  

    def _apply_stick_fault(self, value: float) -> float:
        if self.stick_fault == StickFault.NONE:
            return value

        elif self.stick_fault == StickFault.LOSS_OF_SIGNAL:
            return 0.0

        elif self.stick_fault == StickFault.HARD_OVER:
            return self.hard_over_value

        elif self.stick_fault == StickFault.STUCK_AT:
            if self._stuck_value is None:
                self._stuck_value = value   
            return self._stuck_value

        return value  

    def _apply_throttle_fault(self, value: float, dt: float) -> float:
        if self.throttle_fault == ThrottleFault.NONE:
            return value

        elif self.throttle_fault == ThrottleFault.BIAS_DRIFT:
            self._bias_accum += self.bias_rate * dt
            return float(min(value + self._bias_accum, 1.0))

        elif self.throttle_fault == ThrottleFault.GAIN_FAILURE:
            return float(min(value * self.gain_factor, 1.0))

        return value  

    def reset(self):
        
        self.stick_fault    = StickFault.NONE
        self.throttle_fault = ThrottleFault.NONE
        self._stuck_value   = None
        self._bias_accum    = 0.0
        self._fault_start   = time.time()

    def status(self) -> dict:
        return {
            "stick_fault":    self.stick_fault,
            "throttle_fault": self.throttle_fault,
        }
