

import threading
import time
import math
from collections import deque


# ── Parameters ────────────────────────────────────────────────────────────────

WINDOW_SIZE = 40      # 2 seconds at 20Hz
MIN_SAMPLES = 25      # minimum samples before prediction starts

# Kalman innovation thresholds (in sigma units)
# Calibrated: FG noise floor < 3 sigma, real faults produce 15-30 sigma
INNOV_MEDIUM = 5.0
INNOV_HIGH   = 10.0

# CUSUM parameters
CUSUM_K             = 2.0   # reference — min fault sigma to accumulate above
CUSUM_H             = 8.0   # alarm threshold
CUSUM_DECAY         = 0.15  # FIX 1: per-sample decay inside loop (old = 0.05 once at end)
CUSUM_WARN          = 4.0   # pre-drift warning (below alarm)
CUSUM_WARMUP        = 100   # FIX 2: frames before CUSUM activates (5 s at 20 Hz)
CUSUM_HEALTHY_RESET = 60    # FIX 3: hard-reset after this many consecutive HEALTHY frames (3 s)

# Spike rejection
SPIKE_STICK    = 0.8
SPIKE_THROTTLE = 0.35

# Post-fault hold cycles
HOLD_CYCLES = 3

# Trajectory extrapolation
GUARD_THRESHOLD = 0.15
PREDICT_HORIZON = 10.0
TRAJ_MIN_DIFF   = 0.01
TRAJ_MIN_RATE   = 0.002


# ── Kalman Filter (1D, adaptive) ──────────────────────────────────────────────

class KalmanFilter1D:
    """
    1-dimensional Kalman Filter for signal tracking and fault detection.
    State: x = [value, velocity].
    Adaptive: estimates its own measurement noise R from signal history.
    """

    def __init__(self, process_noise=1e-4, initial_variance=1.0):
        self.x    = 0.0
        self.v    = 0.0
        self.P_xx = initial_variance
        self.P_xv = 0.0
        self.P_vv = initial_variance
        self.Q    = process_noise
        self.R    = 1.0

        self._innov_history = deque(maxlen=40)
        self._innov_mean    = 0.0
        self._innov_std     = 1.0
        self.initialized    = False

    def initialize(self, value: float):
        self.x    = value
        self.v    = 0.0
        self.P_xx = 0.1
        self.P_xv = 0.0
        self.P_vv = 0.01
        self.initialized = True

    def update(self, measurement: float, dt: float = 0.05) -> dict:
        if not self.initialized:
            self.initialize(measurement)
            return {"innovation": 0.0, "innovation_sigma": 0.0,
                    "predicted": measurement, "estimated": measurement,
                    "velocity": 0.0, "innov_std": 1.0}

        # PREDICT
        x_pred    = self.x + self.v * dt
        v_pred    = self.v
        P_xx_pred = self.P_xx + dt * (2 * self.P_xv + dt * self.P_vv) + self.Q
        P_xv_pred = self.P_xv + dt * self.P_vv
        P_vv_pred = self.P_vv + self.Q * 0.1

        # MEASURE
        innovation       = measurement - x_pred
        S                = P_xx_pred + self.R
        innovation_sigma = innovation / math.sqrt(max(S, 1e-10))

        # UPDATE
        K_x    = P_xx_pred / S
        K_v    = P_xv_pred / S
        self.x = x_pred + K_x * innovation
        self.v = v_pred + K_v * innovation

        self.P_xx = (1 - K_x) * P_xx_pred
        self.P_xv = (1 - K_x) * P_xv_pred
        self.P_vv = P_vv_pred - K_v * P_xv_pred

        # ADAPTIVE noise estimation
        self._innov_history.append(innovation)
        if len(self._innov_history) >= 10:
            innov_list       = list(self._innov_history)
            self._innov_mean = sum(innov_list) / len(innov_list)
            variance         = sum((x - self._innov_mean)**2
                                   for x in innov_list) / (len(innov_list) - 1)
            self._innov_std  = math.sqrt(max(variance, 1e-10))
            if abs(innovation) < 3 * self._innov_std:
                self.R = max(variance * 0.5, 1e-6)

        return {
            "innovation":       innovation,
            "innovation_sigma": innovation_sigma,
            "predicted":        x_pred,
            "estimated":        self.x,
            "velocity":         self.v,
            "innov_std":        self._innov_std,
        }


# ── AIAnalyst ─────────────────────────────────────────────────────────────────

class AIAnalyst:

    def __init__(self):
        self._kf_stick    = KalmanFilter1D(process_noise=1e-4)
        self._kf_throttle = KalmanFilter1D(process_noise=1e-4)

        # Healthy baseline buffers (frozen when fault active)
        self._stick_buf    = deque(maxlen=WINDOW_SIZE)
        self._throttle_buf = deque(maxlen=WINDOW_SIZE)

        # Live buffers — all samples including faulty
        self._live_stick    = deque(maxlen=WINDOW_SIZE)
        self._live_throttle = deque(maxlen=WINDOW_SIZE)

        # Innovation residual buffers
        self._innov_stick    = deque(maxlen=WINDOW_SIZE)
        self._innov_throttle = deque(maxlen=WINDOW_SIZE)

        self._kf_result_stick    = {}
        self._kf_result_throttle = {}

        self._stick_fault    = "NONE"
        self._throttle_fault = "NONE"
        self._guard_status   = "HEALTHY"

        # CUSUM accumulators
        self._csp = 0.0
        self._csn = 0.0
        self._ctp = 0.0
        self._ctn = 0.0

        self._cycles_since_fault = HOLD_CYCLES + 1

        # FIX 2 & 3: counters
        self._frame_count         = 0
        self._healthy_frame_count = 0

        self._result = {
            "risk":       "LOW",
            "fault_type": "NONE",
            "prediction": "Initialising Kalman filters...",
        }

        self._lock   = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def start(self):
        pass

    def push(self, t: float, stick: float, throttle: float,
             stick_fault: str = "NONE", throttle_fault: str = "NONE",
             status: str = "HEALTHY"):
        fault_now = (stick_fault != "NONE" or throttle_fault != "NONE"
                     or status == "FAILED")

        # FIX 2: frame counter
        self._frame_count += 1

        
        if not fault_now:
            self._healthy_frame_count += 1
            if self._healthy_frame_count >= CUSUM_HEALTHY_RESET:
                self._csp = 0.0
                self._csn = 0.0
                self._ctp = 0.0
                self._ctn = 0.0
        else:
            self._healthy_frame_count = 0

        # Kalman runs every frame at 20 Hz
        kf_s = self._kf_stick.update(stick,    dt=0.05)
        kf_t = self._kf_throttle.update(throttle, dt=0.05)

        self._kf_result_stick    = kf_s
        self._kf_result_throttle = kf_t

        self._innov_stick.append(kf_s["innovation"])
        self._innov_throttle.append(kf_t["innovation"])

        self._live_stick.append(stick)
        self._live_throttle.append(throttle)

        # Healthy baseline only updates during fault-free frames
        if not fault_now:
            if self._stick_buf and abs(stick - self._stick_buf[-1]) > SPIKE_STICK:
                stick = self._stick_buf[-1]
            if self._throttle_buf and abs(throttle - self._throttle_buf[-1]) > SPIKE_THROTTLE:
                throttle = self._throttle_buf[-1]
            self._stick_buf.append(stick)
            self._throttle_buf.append(throttle)

        with self._lock:
            self._stick_fault    = stick_fault
            self._throttle_fault = throttle_fault
            self._guard_status   = status

    def get_latest(self) -> dict:
        with self._lock:
            return dict(self._result)

    def _run(self):
        while True:
            time.sleep(2.0)
            try:
                self._analyse()
            except Exception as e:
                with self._lock:
                    self._result = {
                        "risk":       "LOW",
                        "fault_type": "NONE",
                        "prediction": f"Analyst error: {e}",
                    }

    def _analyse(self):
        with self._lock:
            stick_fault  = self._stick_fault
            throt_fault  = self._throttle_fault
            guard_status = self._guard_status

        fault_confirmed = (stick_fault != "NONE" or throt_fault != "NONE"
                           or guard_status == "FAILED")

        baseline_stick    = list(self._stick_buf)
        baseline_throttle = list(self._throttle_buf)
        live_stick        = list(self._live_stick)
        live_throttle     = list(self._live_throttle)
        innov_stick       = list(self._innov_stick)
        innov_throttle    = list(self._innov_throttle)

        if len(baseline_stick) < MIN_SAMPLES:
            with self._lock:
                self._result = {
                    "risk":       "LOW",
                    "fault_type": "NONE",
                    "prediction": (f"Building Kalman baseline — "
                                   f"{len(baseline_stick)}/{MIN_SAMPLES} samples"),
                }
            return

        # FIX 2: warmup guard — skip classification until Kalman converges
        if self._frame_count < CUSUM_WARMUP:
            with self._lock:
                self._result = {
                    "risk":       "LOW",
                    "fault_type": "NONE",
                    "prediction": (f"Kalman converging — "
                                   f"warmup frame {self._frame_count}/{CUSUM_WARMUP}"),
                }
            return

        kf_s = self._kf_result_stick
        kf_t = self._kf_result_throttle

        innov_sigma_s = abs(kf_s.get("innovation_sigma", 0.0))
        innov_sigma_t = abs(kf_t.get("innovation_sigma", 0.0))
        innov_s       = kf_s.get("innovation", 0.0)
        innov_t       = kf_t.get("innovation", 0.0)
        predicted_s   = kf_s.get("predicted",  0.3)
        predicted_t   = kf_t.get("predicted",  0.5)
        velocity_s    = kf_s.get("velocity",   0.0)
        velocity_t    = kf_t.get("velocity",   0.0)

        cs = self._cusum_innov(innov_stick,    "stick")
        ct = self._cusum_innov(innov_throttle, "throttle")

        trend_s     = _trend(innov_stick)
        trend_t     = _trend(innov_throttle)
        var_innov_s = _variance(innov_stick[-20:])    if len(innov_stick)    >= 20 else 0
        var_innov_t = _variance(innov_throttle[-20:]) if len(innov_throttle) >= 20 else 0

        last_s  = live_stick[-1]    if live_stick    else 0.3
        last_t  = live_throttle[-1] if live_throttle else 0.5
        var_s   = _variance(live_stick[-20:])    if len(live_stick)    >= 20 else 0
        var_t   = _variance(live_throttle[-20:]) if len(live_throttle) >= 20 else 0
        rng_s   = max(live_stick)    - min(live_stick)    if live_stick    else 0
        rng_t   = max(live_throttle) - min(live_throttle) if live_throttle else 0

        cusum_max = max(self._csp, self._csn, self._ctp, self._ctn)

        b_mean_s = _mean(baseline_stick)
        b_mean_t = _mean(baseline_throttle)
        traj_s   = self._trajectory(live_stick,    b_mean_s)
        traj_t   = self._trajectory(live_throttle, b_mean_t)

        result = self._predict(
            innov_sigma_s, innov_sigma_t,
            innov_s, innov_t,
            predicted_s, predicted_t,
            velocity_s, velocity_t,
            cs, ct,
            trend_s, trend_t,
            var_s, var_t,
            var_innov_s, var_innov_t,
            last_s, last_t,
            rng_s, rng_t,
            cusum_max,
            traj_s, traj_t,
            fault_confirmed,
            stick_fault, throt_fault, guard_status,
        )

        if fault_confirmed:
            self._cycles_since_fault = 0
        else:
            self._cycles_since_fault += 1

        with self._lock:
            self._result = result

    def _cusum_innov(self, innovations: list, channel: str) -> dict:
       
        if len(innovations) < 5:
            return {"pos": 0.0, "neg": 0.0, "alarm": False, "dir": "none"}

        # Zero during warmup
        if self._frame_count < CUSUM_WARMUP:
            if channel == "stick":
                self._csp, self._csn = 0.0, 0.0
            else:
                self._ctp, self._ctn = 0.0, 0.0
            return {"pos": 0.0, "neg": 0.0, "alarm": False, "dir": "none"}

        recent = innovations[-20:] if len(innovations) >= 20 else innovations
        std    = max(_std(recent), 1e-6)

        c_pos = self._csp if channel == "stick" else self._ctp
        c_neg = self._csn if channel == "stick" else self._ctn

        for inn in recent:
            xi    = inn / std
            c_pos = max(0.0, c_pos + xi - CUSUM_K)
            c_neg = max(0.0, c_neg - xi - CUSUM_K)
            # FIX 1: per-sample decay always applied
            c_pos = max(0.0, c_pos - CUSUM_DECAY)
            c_neg = max(0.0, c_neg - CUSUM_DECAY)

        c_pos = min(c_pos, CUSUM_H * 2)
        c_neg = min(c_neg, CUSUM_H * 2)

        if channel == "stick":
            self._csp, self._csn = c_pos, c_neg
        else:
            self._ctp, self._ctn = c_pos, c_neg

        alarm     = c_pos >= CUSUM_H or c_neg >= CUSUM_H
        direction = ("up"   if c_pos >= CUSUM_H else
                     "down" if c_neg >= CUSUM_H else "none")
        return {"pos": c_pos, "neg": c_neg, "alarm": alarm, "dir": direction}

    def _trajectory(self, live: list, baseline_mean: float) -> dict:
        if len(live) < 10:
            return {"drift_rate": 0.0, "current_diff": 0.0,
                    "time_to_breach": None, "converging": False}

        data   = live[-20:] if len(live) >= 20 else live
        n      = len(data)
        x_mean = (n - 1) / 2.0
        y_mean = _mean(data)

        num   = sum((i - x_mean) * (data[i] - y_mean) for i in range(n))
        den   = sum((i - x_mean) ** 2                  for i in range(n))
        slope = (num / den) if abs(den) > 1e-10 else 0.0

        drift_rate   = slope * 20.0
        current_val  = _mean(data[-5:])
        current_diff = abs(current_val - baseline_mean)
        remaining    = GUARD_THRESHOLD - current_diff

        moving_away = ((current_val > baseline_mean and slope > 0) or
                       (current_val < baseline_mean and slope < 0))

        if moving_away and abs(slope) > 1e-6 and remaining > 0:
            time_to_breach = (remaining / abs(slope)) * 0.05
            converging     = True
        elif current_diff >= GUARD_THRESHOLD:
            time_to_breach = 0.0
            converging     = True
        else:
            time_to_breach = None
            converging     = False

        return {
            "drift_rate":     round(drift_rate,     4),
            "current_diff":   round(current_diff,   4),
            "time_to_breach": round(time_to_breach, 1) if time_to_breach is not None else None,
            "converging":     converging,
        }

    def _predict(self,
                 innov_sigma_s, innov_sigma_t,
                 innov_s, innov_t,
                 predicted_s, predicted_t,
                 velocity_s, velocity_t,
                 cs, ct,
                 trend_s, trend_t,
                 var_s, var_t,
                 var_innov_s, var_innov_t,
                 last_s, last_t,
                 rng_s, rng_t,
                 cusum_max,
                 traj_s, traj_t,
                 fault_confirmed,
                 stick_fault, throt_fault, guard_status) -> dict:

        # 1. HARD_OVER
        if innov_sigma_s > INNOV_MEDIUM and innov_s > 0 and last_s > 0.6:
            risk = "HIGH" if innov_sigma_s > INNOV_HIGH else "MEDIUM"
            return {
                "risk":       risk,
                "fault_type": "HARD_OVER",
                "prediction": (f"HARD_OVER predicted — Kalman innovation {innov_sigma_s:.1f}sigma "
                               f"[predicted={predicted_s:.3f}, actual={last_s:.3f}, "
                               f"residual={innov_s:+.3f}] — guard not yet tripped"),
            }

        # 2. LOSS_OF_SIGNAL
        if innov_sigma_s > INNOV_MEDIUM and innov_s < 0 and last_s < 0.15:
            risk = "HIGH" if innov_sigma_s > INNOV_HIGH else "MEDIUM"
            return {
                "risk":       risk,
                "fault_type": "LOSS_OF_SIGNAL",
                "prediction": (f"LOSS_OF_SIGNAL predicted — Kalman innovation {innov_sigma_s:.1f}sigma "
                               f"[predicted={predicted_s:.3f}, actual={last_s:.3f}, "
                               f"residual={innov_s:+.3f}] — guard not yet tripped"),
            }

        # 3. GAIN_FAILURE
        if (innov_sigma_t > INNOV_MEDIUM and innov_t > 0
                and last_t > 0.6 and abs(velocity_t) > 0.02):
            risk = "HIGH" if innov_sigma_t > INNOV_HIGH else "MEDIUM"
            return {
                "risk":       risk,
                "fault_type": "GAIN_FAILURE",
                "prediction": (f"GAIN_FAILURE predicted — Kalman innovation {innov_sigma_t:.1f}sigma "
                               f"[predicted={predicted_t:.3f}, actual={last_t:.3f}, "
                               f"velocity={velocity_t:+.4f}] — guard not yet tripped"),
            }

        # 4. BIAS_DRIFT
        if (self._ctp > 3.0 and trend_t > 0.002
                and velocity_t > 0.001 and innov_t > 0):
            if self._ctp > 5.0 or innov_sigma_t > INNOV_MEDIUM:
                risk, conf = "HIGH", "HIGH confidence"
            else:
                risk, conf = "MEDIUM", "early warning"
            traj_note = ""
            if traj_t["converging"] and traj_t["time_to_breach"] is not None:
                traj_note = f" — breach in ~{traj_t['time_to_breach']:.1f}s"
            return {
                "risk":       risk,
                "fault_type": "BIAS_DRIFT",
                "prediction": (f"BIAS_DRIFT predicted [{conf}] — "
                               f"Kalman velocity={velocity_t:+.4f}/frame, "
                               f"CUSUM={self._ctp:.2f}, "
                               f"innovation={innov_t:+.3f}"
                               f"{traj_note} — guard not yet tripped"),
            }

        # 5. STUCK_AT
        if (var_s < 0.0002 and abs(velocity_s) < 0.001
                and abs(innov_s) > 0.01 and abs(trend_s) < 0.001):
            return {
                "risk":       "MEDIUM",
                "fault_type": "STUCK_AT",
                "prediction": (f"STUCK_AT predicted — signal frozen at {last_s:.3f} "
                               f"[velocity~0, var={var_s:.5f}, "
                               f"innovation={innov_s:+.3f}] — guard not yet tripped"),
            }

        # 6. Guard confirmed
        if fault_confirmed:
            self._cycles_since_fault = 0
            active = stick_fault if stick_fault != "NONE" else (
                     throt_fault if throt_fault != "NONE" else "CASCADE")
            return {
                "risk":       "HIGH",
                "fault_type": active,
                "prediction": (f"{active} CONFIRMED by guard — "
                               f"stick {innov_sigma_s:.1f}sigma, "
                               f"throttle {innov_sigma_t:.1f}sigma, "
                               f"CUSUM={cusum_max:.2f}"),
            }

        # Hold-down after fault clears
        if self._cycles_since_fault <= HOLD_CYCLES:
            rem = HOLD_CYCLES - self._cycles_since_fault + 1
            return {
                "risk":       "MEDIUM",
                "fault_type": "NONE",
                "prediction": f"Fault cleared — Kalman filters reconverging ({rem} cycle(s))",
            }

        # 7A. Unknown trajectory (stick)
        if (traj_s["converging"] and traj_s["time_to_breach"] is not None
                and traj_s["time_to_breach"] <= PREDICT_HORIZON
                and traj_s["current_diff"] > TRAJ_MIN_DIFF
                and abs(traj_s["drift_rate"]) > TRAJ_MIN_RATE):
            ttb  = traj_s["time_to_breach"]
            risk = "HIGH" if ttb <= 2.5 else "MEDIUM"
            return {
                "risk":       risk,
                "fault_type": "UNKNOWN_ANOMALY",
                "prediction": (f"UNKNOWN FAULT PREDICTED — stick trajectory converging "
                               f"[rate={traj_s['drift_rate']:+.3f}/s, "
                               f"breach in ~{ttb:.1f}s]"),
            }

        # 7B. Unknown trajectory (throttle)
        if (traj_t["converging"] and traj_t["time_to_breach"] is not None
                and traj_t["time_to_breach"] <= PREDICT_HORIZON
                and traj_t["current_diff"] > TRAJ_MIN_DIFF
                and abs(traj_t["drift_rate"]) > TRAJ_MIN_RATE):
            ttb  = traj_t["time_to_breach"]
            risk = "HIGH" if ttb <= 2.5 else "MEDIUM"
            return {
                "risk":       risk,
                "fault_type": "UNKNOWN_ANOMALY",
                "prediction": (f"UNKNOWN FAULT PREDICTED — throttle trajectory converging "
                               f"[rate={traj_t['drift_rate']:+.3f}/s, "
                               f"breach in ~{ttb:.1f}s]"),
            }

        # 7C. Both channels anomalous
        if innov_sigma_s > INNOV_MEDIUM and innov_sigma_t > INNOV_MEDIUM:
            return {
                "risk":       "HIGH",
                "fault_type": "UNKNOWN_ANOMALY",
                "prediction": (f"UNKNOWN — both channels anomalous "
                               f"[stick {innov_sigma_s:.1f}sigma, "
                               f"throttle {innov_sigma_t:.1f}sigma]"),
            }

        # 7D. High-variance oscillation
        if rng_s > 0.35 and var_s > 0.015 and abs(trend_s) < 0.01:
            return {
                "risk":       "HIGH",
                "fault_type": "UNKNOWN_ANOMALY",
                "prediction": (f"UNKNOWN — stick oscillating erratically "
                               f"[range={rng_s:.3f}] possible sensor flutter"),
            }

        # 7E. Throttle drifting down
        if (trend_t < -0.003 and self._ctn > 1.0
                and innov_t < 0 and velocity_t < -0.001):
            return {
                "risk":       "HIGH" if self._ctn > 2.5 else "MEDIUM",
                "fault_type": "UNKNOWN_ANOMALY",
                "prediction": (f"UNKNOWN — throttle drifting downward "
                               f"[velocity={velocity_t:+.4f}, CUSUM-={self._ctn:.2f}]"),
            }

        # 7F. Stick drifting without locking
        if abs(trend_s) > 0.008 and cs["alarm"]:
            drn = "upward" if trend_s > 0 else "downward"
            return {
                "risk":       "HIGH",
                "fault_type": "UNKNOWN_ANOMALY",
                "prediction": (f"UNKNOWN — stick drifting {drn} without locking "
                               f"[velocity={velocity_s:+.4f}]"),
            }

        # 7G. Elevated innovation catch-all
        if innov_sigma_s > INNOV_MEDIUM or innov_sigma_t > INNOV_MEDIUM:
            ch  = "stick"      if innov_sigma_s >= innov_sigma_t else "throttle"
            sig = innov_sigma_s if innov_sigma_s >= innov_sigma_t else innov_sigma_t
            return {
                "risk":       "HIGH" if sig > INNOV_HIGH else "MEDIUM",
                "fault_type": "UNKNOWN_ANOMALY",
                "prediction": (f"UNKNOWN — {ch} innovation {sig:.1f}sigma "
                               f"no matching fault pattern"),
            }

        # 8. CUSUM pre-drift warning
        if cusum_max > CUSUM_WARN:
            if   self._csp >= cusum_max: detail = "stick drifting UP"
            elif self._csn >= cusum_max: detail = "stick drifting DOWN"
            elif self._ctp >= cusum_max: detail = "throttle drifting UP"
            else:                        detail = "throttle drifting DOWN"
            return {
                "risk":       "MEDIUM",
                "fault_type": "NONE",
                "prediction": (f"Pre-fault drift — Kalman residual accumulating: {detail} "
                               f"[CUSUM={cusum_max:.2f}] — guard still nominal"),
            }

        # 9. Nominal — everything healthy
        return {
            "risk":       "LOW",
            "fault_type": "NONE",
            "prediction": (f"Nominal — stick {innov_sigma_s:.2f}sigma, "
                           f"throttle {innov_sigma_t:.2f}sigma, "
                           f"CUSUM={cusum_max:.2f}"),
        }


# ── Pure math helpers ─────────────────────────────────────────────────────────

def _mean(data):
    return sum(data) / len(data) if data else 0.0

def _variance(data):
    if len(data) < 2: return 0.0
    m = _mean(data)
    return sum((x - m)**2 for x in data) / (len(data) - 1)

def _std(data):
    return _variance(data) ** 0.5

def _trend(data):
    n = len(data)
    if n < 10: return 0.0
    mid = n // 2
    return _mean(data[mid:]) - _mean(data[:mid])