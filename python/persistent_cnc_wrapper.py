#!/usr/bin/env python3
"""
Improved Persistent CNC Wrapper - Continuous status polling, no callbacks
"""

import sys
import json
import threading
import time
import signal
import requests
import urllib3
from RestfulAPIBase import RestfulInterface

urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

class SimpleCNCWrapper:
    def __init__(self, status_polling_interval=0.5):
        self.running = False
        self.connected = False
        self.rest_interface = None
        self.status_polling_interval = status_polling_interval
        
        # Simple status tracking with thread-safe access
        self.current_status = -1  # Unknown
        self.status_lock = threading.Lock()
        
        # Status polling thread
        self.status_thread = None
        self.status_polling_enabled = False  # Only start after load_and_run
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.log(f"Received signal {signum}, shutting down...")
        self.shutdown()

    def log(self, message):
        """Thread-safe logging"""
        print(f"[CNC] {message}", flush=True)

    def send_response(self, success, message, data=None):
        """Send response back to C++"""
        response = {
            "success": success,
            "message": message,
            "timestamp": time.time()
        }
        if data:
            response["data"] = data
        
        print(json.dumps(response), flush=True)

    def start(self):
        """Start the wrapper and connect"""
        self.log("Starting CNC wrapper with continuous status polling...")
        self.running = True
        
        # Connect to CNC immediately
        if not self._connect_to_cnc():
            self.shutdown()
            return
        
        # Start background status polling thread
        self.status_thread = threading.Thread(target=self._continuous_status_poller, name="StatusPoller", daemon=True)
        self.status_thread.start()
        
        # Main input processing loop - handle commands directly
        self._process_commands()

    def shutdown(self):
        """Clean shutdown"""
        self.log("Shutting down...")
        self.running = False
        
        # Disconnect from CNC
        if self.rest_interface:
            try:
                self.rest_interface.Shutdown()
                self.log("Disconnected from CNC")
            except Exception as e:
                self.log(f"Error during disconnect: {e}")
        
        self.log("Shutdown complete")

    def _connect_to_cnc(self):
        """Connect to CNC and store token"""
        try:
            self.log("Connecting to CNC...")
            self.rest_interface = RestfulInterface('SID_RT_PROGRAM_RUNNING')
            
            if self.rest_interface.DidInitialize():
                self.rest_interface.SetMaxFeedOverrides()
                self.connected = True
                self.log("Connected to CNC successfully")
                self.send_response(True, "Connected to CNC", {"status": "connected"})
                return True
            else:
                self.send_response(False, "Failed to initialize CNC connection")
                return False
                
        except Exception as e:
            self.log(f"Connection error: {e}")
            self.send_response(False, f"Connection failed: {e}")
            return False

    def _continuous_status_poller(self):
        """Status polling thread - sends status commands every 0.5s after load_and_run"""
        self.log("Status polling thread started - will send status commands after load_and_run")
        
        while self.running:
            try:
                if self.connected and self.status_polling_enabled:
                    # Send status command every 0.5 seconds
                    self._handle_status_request()
                    
                # Sleep between status commands
                time.sleep(self.status_polling_interval)
                    
            except Exception as e:
                self.log(f"Status polling error: {e}")
                time.sleep(1.0)  # Back off on error
        
        self.log("Status polling thread ending")

    def _get_current_status_from_cnc(self):
        """Get current status directly from CNC REST API"""
        try:
            url = "https://localhost:4504/DataService/Integer/SID_CURRENT_PROGRAM_STATUS"
            headers = {'token': self.rest_interface.tokenValue}
            
            response = requests.get(url, headers=headers, verify=False, timeout=2)
            if response.status_code == 200:
                return response.json()
            else:
                return None
                
        except Exception as e:
            # Silent failure for polling
            return None

    def get_cached_status(self):
        """Get cached status (thread-safe) - called by C++"""
        with self.status_lock:
            return self.current_status

    def is_program_complete(self):
        """Check if program is complete (thread-safe)"""
        status = self.get_cached_status()
        return status in [2, 3, 4]  # COMPLETED_SUCCESSFUL, COMPLETED_ERROR, COMPLETED_ABORT

    def _process_commands(self):
        """Simple command processing"""
        self.log("Command processor started - waiting for commands...")
        
        try:
            while self.running:
                try:
                    # Read command from stdin (blocking)
                    line = sys.stdin.readline()
                    if not line:  # EOF
                        break
                    
                    line = line.strip()
                    if not line:
                        continue
                    
                    # Parse and handle command immediately
                    try:
                        command = json.loads(line)
                        self._handle_command_immediately(command)
                        
                    except json.JSONDecodeError as e:
                        self.send_response(False, f"Invalid JSON: {e}")
                        
                except Exception as e:
                    self.log(f"Command processing error: {e}")
                    break
                    
        except KeyboardInterrupt:
            pass
        
        self.log("Command processor ending")
        self.shutdown()

    def _handle_command_immediately(self, command):
        """Handle command immediately - no queuing"""
        cmd_type = command.get("command")
        if cmd_type == "status":
            self._handle_status_request()
        elif cmd_type == "load_and_run":
            self._handle_load_and_run(command.get("file_path"))
        elif cmd_type == "start_monitoring":
            self._handle_start_monitoring()
        elif cmd_type == "stop_monitoring":
            self._handle_stop_monitoring()
        elif cmd_type == "reset_status":  # NEW COMMAND
            self._handle_reset_status()
        elif cmd_type == "shutdown":
            self.shutdown()
        else:
            self.send_response(False, f"Unknown command: {cmd_type}")

    def _handle_reset_status(self):
        """Reset status for new experiment"""
        try:
            self.log("Resetting status for new experiment...")
            
            # Reset internal status tracking
            self.current_status = -1  # Set to UNKNOWN
            
            # Stop any current monitoring
            self.status_polling_enabled = False
            
            # Send response confirming reset
            self.send_response(True, "Status reset to UNKNOWN for new experiment", 
                             {"status_code": -1, "status_text": "Unknown"})
            
            self.log("Status reset complete - ready for new experiment")
            
        except Exception as e:
            self.log(f"Reset status error: {e}")
            self.send_response(False, f"Reset status failed: {e}")

    def _handle_status_request(self):
        """Handle status request - get fresh status and send response"""
        try:
            if not self.connected:
                self.send_response(False, "Not connected to CNC")
                return
            
            # Get fresh status from CNC
            status_code = self._get_current_status_from_cnc()
            
            if status_code is not None:
                # Update cached status
                with self.status_lock:
                    old_status = self.current_status
                    self.current_status = status_code
                
                # Always send response with current status
                self.send_response(True, f"Status: {status_code}", 
                                 {"status_code": status_code, "status_text": self._get_status_text(status_code)})
                
                # Log significant changes
                if old_status != status_code and old_status != -1:
                    self.log(f"Status change: {old_status} → {status_code} ({self._get_status_text(status_code)})")

                if status_code in [2, 3, 4]:  # COMPLETED_SUCCESSFUL, COMPLETED_ERROR, COMPLETED_ABORT
                    self.log(f"Program completed with status {status_code} - stopping status polling")
                    self.status_polling_enabled = False

            else:
                self.send_response(False, "Failed to get status from CNC")
                
        except Exception as e:
            self.log(f"Status request error: {e}")
            self.send_response(False, f"Status request failed: {e}")

    def _handle_start_monitoring(self):
        """Handle start monitoring command"""
        try:
            self.status_polling_enabled = True
            self.send_response(True, "Status monitoring started")
            self.log("Status monitoring enabled manually")
        except Exception as e:
            self.log(f"Start monitoring error: {e}")
            self.send_response(False, f"Start monitoring failed: {e}")

    def _handle_stop_monitoring(self):
        """Handle stop monitoring command"""
        try:
            self.status_polling_enabled = False
            self.send_response(True, "Status monitoring stopped")
            self.log("Status monitoring disabled manually")
        except Exception as e:
            self.log(f"Stop monitoring error: {e}")
            self.send_response(False, f"Stop monitoring failed: {e}")

    def _handle_load_and_run(self, file_path):
        """Handle load and run command with proper CNC startup sequence"""
        try:
            if not self.connected:
                self.send_response(False, "Not connected to CNC")
                return
            
            if not file_path:
                self.send_response(False, "No file path provided")
                return
            
            self.log(f"Loading program: {file_path}")
            
            # Step 1: Load the program (but don't run yet)
            if not self.rest_interface.LoadAnyFile(file_path, RunAfterLoading=1):  # 1 = queue to run after loading
                self.send_response(False, f"Failed to load program: {file_path}")
                return
            
            # Enable automatic status polling now that program is running
            self.status_polling_enabled = True
            self.log("Status polling ENABLED - will send status commands every 0.5s")
            
            self.log("Program started successfully")
            self.send_response(True, f"Successfully started: {file_path}")
                
        except Exception as e:
            self.log(f"Load and run error: {e}")
            self.send_response(False, f"Load and run failed: {e}")

    def _get_status_text(self, status_code):
        """Convert status code to readable text"""
        status_map = {
            0: "Uninitialized",
            1: "Started",
            2: "Completed Successful", 
            3: "Completed Error",
            4: "Completed Abort",
            5: "Pending",
            6: "Checking Ready",
            7: "Ready Check Failed",
            -1: "Unknown"
        }
        return status_map.get(status_code, f"Unknown ({status_code})")

def main():
    """Main entry point"""
    try:
        wrapper = SimpleCNCWrapper(status_polling_interval=0.5)
        wrapper.start()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(json.dumps({
            "success": False,
            "message": f"Wrapper crashed: {e}"
        }), flush=True)

if __name__ == "__main__":
    main()