import socket
import time
import logging
import threading

log = logging.getLogger(__name__)


class FlightGearBridge:
 
    
    def __init__(self, host="127.0.0.1", port=5401):
        self.host = host
        self.port = port
        self.sock = None
        self.connected = False
        self._lock = threading.Lock()
        self._connect()
    
    def _connect(self):
        
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            self.sock.settimeout(0.5)
            
            try:
                self.sock.recv(4096)
            except:
                pass
            self.connected = True
            log.info(f" FlightGear connected at {self.host}:{self.port}")
        except Exception as e:
            log.error(f" FlightGear connection failed: {e}")
            self.connected = False
    
    def get_property(self, prop_path):
        
        if not self.connected:
            return 0.0
        
        with self._lock:
            try:
                cmd = f"get {prop_path}\r\n"
                self.sock.sendall(cmd.encode('ascii'))
                
                response = self.sock.recv(1024).decode('ascii').strip()
                
                if '=' in response:
                    value_str = response.split('=')[1].strip().split("'")[1]
                    return float(value_str)
                return 0.0
            except Exception as e:
                log.warning(f"Get property {prop_path} failed: {e}")
                return 0.0
    
    def set_property(self, prop_path, value):
       
        if not self.connected:
            return False
        
        with self._lock:
            try:
                cmd = f"set {prop_path} {value:.6f}\r\n"
                self.sock.sendall(cmd.encode('ascii'))
                # Read response
                try:
                    self.sock.recv(1024)
                except:
                    pass
                return True
            except Exception as e:
                log.warning(f"Set property {prop_path} failed: {e}")
                return False
    
    def read_state(self):
       
        return {
            
            "raw_stick": self.get_property("/controls/flight/elevator"),
            "raw_throttle": self.get_property("/controls/engines/engine[0]/throttle"),
            
           
            "pitch_rate_q": self.get_property("/fdm/jsbsim/velocities/q-rad_sec"),
            "accel_ax": self.get_property("/fdm/jsbsim/accelerations/udot-ft_sec2"),
            "altitude_ft": self.get_property("/position/altitude-ft"),
            "airspeed_kts": self.get_property("/velocities/airspeed-kt"),
        }
    
    def write_safe_commands(self, safe_stick, safe_throttle):
       
        
        self.set_property("/fdm/jsbsim/fcs/elevator-cmd-norm", safe_stick)
        self.set_property("/fdm/jsbsim/fcs/throttle-cmd-norm[0]", safe_throttle)
        self.set_property("/fdm/jsbsim/fcs/throttle-cmd-norm[1]", safe_throttle)
    
    def close(self):
        if self.sock:
            self.sock.close()
        log.info("FlightGear bridge closed")
