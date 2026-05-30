

import socket
import logging

log = logging.getLogger(__name__)


class UDPSender:
  

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 5401,  
        mode: str = "text",
    ):
        self.host = host
        self.port = port
        self.mode = mode
        self._sock = None
        self._connected = False
        self._connect()
        log.info(f"UDPSender (Telnet) ready → {host}:{port}")

    def _connect(self):
        
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.connect((self.host, self.port))
            self._sock.settimeout(0.1)
            
            try:
                self._sock.recv(1024)
            except:
                pass
            self._connected = True
            log.info("Telnet connection established")
        except Exception as e:
            log.warning(f"Telnet connection failed: {e}")
            self._connected = False

    def send(self, safe_stick: float, safe_throttle: float) -> bool:
      
        if not self._connected:
            self._connect()
            if not self._connected:
                return False
        
        try:
            
            cmd1 = f"set /controls/flight/elevator {safe_stick:.4f}\r\n"
            self._sock.sendall(cmd1.encode('ascii'))
            
            
            cmd2 = f"set /controls/engines/engine[0]/throttle {safe_throttle:.4f}\r\n"
            self._sock.sendall(cmd2.encode('ascii'))
            
            cmd3 = f"set /controls/engines/engine[1]/throttle {safe_throttle:.4f}\r\n"
            self._sock.sendall(cmd3.encode('ascii'))
            
            
            try:
                self._sock.recv(1024)
            except:
                pass
            
            return True
            
        except Exception as e:
            log.warning(f"Telnet send failed: {e}")
            self._connected = False
            return False

    def close(self):
        if self._sock:
            self._sock.close()
        log.info("Telnet connection closed.")

    @property
    def connected(self) -> bool:
        return self._connected