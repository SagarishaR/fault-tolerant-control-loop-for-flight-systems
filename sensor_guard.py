

STICK_RATE_LIMIT    = 0.5
THROTTLE_RATE_LIMIT = 0.15

STICK_BASELINE    = 0.3   
THROTTLE_BASELINE = 0.5   


class SensorGuard:

    def __init__(self):
        self._prev_stick    = STICK_BASELINE
        self._prev_throttle = THROTTLE_BASELINE
        self._fault_was_active = False

    def process(self, faulty_stick, faulty_throttle, dt=0.05,
                stick_fault="NONE", throttle_fault="NONE"):

        fault_now = stick_fault != "NONE" or throttle_fault != "NONE"

        
        if fault_now and not self._fault_was_active:
            self._prev_stick    = STICK_BASELINE
            self._prev_throttle = THROTTLE_BASELINE

        self._fault_was_active = fault_now

        if stick_fault != "NONE":
            
            safe_stick = STICK_BASELINE
        else:
            safe_stick = self._rate_limit(faulty_stick, self._prev_stick, STICK_RATE_LIMIT, dt)

        if throttle_fault != "NONE":
           
            safe_throttle = THROTTLE_BASELINE
        else:
            safe_throttle = self._rate_limit(faulty_throttle, self._prev_throttle, THROTTLE_RATE_LIMIT, dt)

        safe_stick    = max(-1.0, min(1.0, safe_stick))
        safe_throttle = max( 0.0, min(1.0, safe_throttle))

        self._prev_stick    = safe_stick
        self._prev_throttle = safe_throttle

        
        diff_stick    = abs(faulty_stick    - safe_stick)
        diff_throttle = abs(faulty_throttle - safe_throttle)
        fault_names_active = stick_fault != "NONE" or throttle_fault != "NONE"
        diff_exceeded      = diff_stick > 0.15 or diff_throttle > 0.15

        if fault_names_active or diff_exceeded:
            status = "FAILED"
            reason = []
            if stick_fault != "NONE":
                reason.append(f"stick: {stick_fault} ({faulty_stick:.2f})")
            if throttle_fault != "NONE":
                reason.append(f"throttle: {throttle_fault} ({faulty_throttle:.2f})")
            if not reason:
                if diff_stick    > 0.15: reason.append(f"stick Δ={diff_stick:.2f}")
                if diff_throttle > 0.15: reason.append(f"throttle Δ={diff_throttle:.2f}")
            reason_str = " | ".join(reason)
        else:
            status     = "HEALTHY"
            reason_str = ""

        return {
            "safe_stick":    round(safe_stick,    4),
            "safe_throttle": round(safe_throttle, 4),
            "status":        status,
            "reason":        reason_str,
        }

    def reset(self):
        self._prev_stick       = STICK_BASELINE
        self._prev_throttle    = THROTTLE_BASELINE
        self._fault_was_active = False

    @staticmethod
    def _rate_limit(target, previous, max_rate, dt):
        max_delta = max_rate * dt
        delta     = max(-max_delta, min(max_delta, target - previous))
        return previous + delta