import socket
import threading
import logging
import time

log = logging.getLogger(__name__)


class FlightGearBridge:

    _SPIKE_LIMIT_STICK    = 0.8
    _SPIKE_LIMIT_THROTTLE = 0.35

    def __init__(self, host="127.0.0.1", port=5401):
        self.host      = host
        self.port      = port
        self.sock      = None
        self.connected = False
        self._lock     = threading.Lock()
        self._prev_stick    = 0.0
        self._prev_throttle = 0.5

        self.last_write_time       = 0.0
        self.last_written_stick    = None
        self.last_written_throttle = None
        self.write_count           = 0

        self._connect()

    def _connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            self.sock.settimeout(0.08)
            try:
                self.sock.recv(4096)
            except:
                pass
            self.connected = True
            log.info("FlightGear connected (bidirectional mode)")
        except Exception as e:
            log.error(f"FlightGear connection failed: {e}")
            self.connected = False

    def _batch_get(self, paths):
        defaults = [0.0] * len(paths)
        if not self.connected:
            return defaults
        with self._lock:
            try:
                cmd = "".join(f"get {p}\r\n" for p in paths)
                self.sock.sendall(cmd.encode())
                raw = b""
                deadline = time.time() + 0.08
                while raw.count(b"\n") < len(paths):
                    try:
                        chunk = self.sock.recv(4096)
                        if chunk:
                            raw += chunk
                    except socket.timeout:
                        break
                    if time.time() > deadline:
                        break
                lines = [l.strip() for l in raw.decode(errors="ignore").splitlines() if "=" in l]
                results = []
                for i, line in enumerate(lines[:len(paths)]):
                    try:
                        results.append(float(line.split("=")[1].strip().split("'")[1]))
                    except:
                        results.append(defaults[i] if i < len(defaults) else 0.0)
                while len(results) < len(paths):
                    results.append(0.0)
                return results
            except Exception as e:
                log.warning(f"Batch GET failed: {e}")
                self.connected = False
                return defaults

    def _batch_set(self, prop_value_pairs):
        if not self.connected:
            return False
        with self._lock:
            try:
                cmd = "".join(f"set {p} {v:.6f}\r\n" for p, v in prop_value_pairs)
                self.sock.sendall(cmd.encode())
                try:
                    self.sock.recv(4096)
                except:
                    pass
                return True
            except Exception as e:
                log.warning(f"Batch SET failed: {e}")
                self.connected = False
                return False

    def read_state(self):
        vals = self._batch_get([
            # ── FCS signals (existing) ────────────────────────────────────────
            "/controls/flight/elevator",
            "/controls/engines/engine[0]/throttle",
            # ── Navigation / Position ─────────────────────────────────────────
            "/position/altitude-ft",
            "/velocities/airspeed-kt",
            "/velocities/vertical-speed-fps",
            "/orientation/heading-deg",
            # ── Attitude ──────────────────────────────────────────────────────
            "/orientation/pitch-deg",
            "/orientation/roll-deg",
            # ── Angular rates (real, replaces fake derivation) ────────────────
            "/velocities/pitchrate-degps",
            # ── Engine (engine 0) ─────────────────────────────────────────────
            "/engines/engine[0]/n1",
            "/engines/engine[0]/n2",
        ])

        (raw_stick_read, raw_throttle_read,
         raw_alt, raw_airspeed, raw_vspeed, raw_heading,
         raw_pitch, raw_roll,
         raw_pitchrate,
         raw_n1, raw_n2) = vals

        # ── Spike filter (unchanged) ──────────────────────────────────────────
        if abs(raw_stick_read - self._prev_stick) > self._SPIKE_LIMIT_STICK:
            raw_stick = self._prev_stick
        else:
            raw_stick = raw_stick_read
            self._prev_stick = raw_stick

        if abs(raw_throttle_read - self._prev_throttle) > self._SPIKE_LIMIT_THROTTLE:
            raw_throttle = self._prev_throttle
        else:
            raw_throttle = raw_throttle_read
            self._prev_throttle = raw_throttle

        # ── Sanity gates ──────────────────────────────────────────────────────
        altitude_ft  = raw_alt      if raw_alt > 500.0    else None
        airspeed_kts = raw_airspeed if raw_airspeed > 1.0 else None
        vspeed_fpm   = raw_vspeed * 60.0      # fps → fpm
        heading_deg  = raw_heading % 360.0    # normalise 0–360
        pitch_deg    = raw_pitch
        roll_deg     = raw_roll
        pitch_rate_q = raw_pitchrate          # real deg/s from FG
        engine_n1    = max(0.0, raw_n1)
        engine_n2    = max(0.0, raw_n2)
        accel_ax     = (raw_throttle - 0.3) * 10.0  # keep derived for sensor_guard

        return {
            # ── Existing keys (sensor_guard / main.py depend on these) ────────
            "raw_stick":    raw_stick,
            "raw_throttle": raw_throttle,
            "pitch_rate_q": round(pitch_rate_q, 4),
            "accel_ax":     round(accel_ax,     4),
            "altitude_ft":  round(altitude_ft, 1)  if altitude_ft  is not None else None,
            "airspeed_kts": round(airspeed_kts, 1) if airspeed_kts is not None else None,
            # ── New cockpit instrument keys ───────────────────────────────────
            "vspeed_fpm":   round(vspeed_fpm,  0),
            "heading_deg":  round(heading_deg, 1),
            "pitch_deg":    round(pitch_deg,   2),
            "roll_deg":     round(roll_deg,    2),
            "engine_n1":    round(engine_n1,   1),
            "engine_n2":    round(engine_n2,   1),
        }

    def write_safe_commands(self, safe_stick, safe_throttle):
        """Write guard-corrected safe signals to FlightGear FCS (fault active only)."""
        success = self._batch_set([
            ("/fdm/jsbsim/fcs/elevator-cmd-norm",    safe_stick),
            ("/fdm/jsbsim/fcs/throttle-cmd-norm[0]", safe_throttle),
            ("/fdm/jsbsim/fcs/throttle-cmd-norm[1]", safe_throttle),
        ])
        if success:
            self.last_write_time       = time.time()
            self.last_written_stick    = round(safe_stick,    4)
            self.last_written_throttle = round(safe_throttle, 4)
            self.write_count          += 1

    def get_write_status(self):
        age = time.time() - self.last_write_time
        return {
            "active":   age < 0.5,
            "stick":    self.last_written_stick,
            "throttle": self.last_written_throttle,
            "count":    self.write_count,
        }

    def close(self):
        if self.sock:
            self.sock.close()
        log.info("FlightGear bridge closed")