#!/usr/bin/env python3
"""
Persistent Python API Wrapper for CNC Overseer
Maintains connection and processes commands via queue
"""

import sys
import json
import argparse
import traceback
import threading
import queue
import time
from RestfulAPIBase import RestfulInterface

class PersistentCNCConnection:
    def __init__(self):
        self.rest_interface = None
        self.connected = False
        self.command_queue = queue.Queue()
        self.response_queue = queue.Queue()
        self.worker_thread = None
        self.shutdown_event = threading.Event()
        
    def start_connection_thread(self):
        """Start the persistent connection thread"""
        self.worker_thread = threading.Thread(target=self._connection_worker)
        self.worker_thread.daemon = True
        self.worker_thread.start()
        
    def _connection_worker(self):
        """Worker thread that maintains connection and processes commands"""
        try:
            # Initial connection
            self.rest_interface = RestfulInterface('SID_RT_PROGRAM_RUNNING')
            
            if self.rest_interface.DidInitialize():
                self.rest_interface.SetMaxFeedOverrides()
                self.connected = True
                print(f"THREAD: Connected to CNC successfully", file=sys.stderr)
            else:
                self.connected = False
                print(f"THREAD: Failed to connect to CNC", file=sys.stderr)
                return
                
            # Process commands until shutdown
            while not self.shutdown_event.is_set():
                try:
                    # Wait for command with timeout
                    command_data = self.command_queue.get(timeout=1.0)
                    
                    # Process the command
                    response = self._process_command(command_data)
                    
                    # Send response back
                    self.response_queue.put(response)
                    
                except queue.Empty:
                    continue  # Timeout, check shutdown event again
                except Exception as e:
                    error_response = self._create_response(False, f"Command processing error: {str(e)}")
                    self.response_queue.put(error_response)
                    
        except Exception as e:
            print(f"THREAD: Connection worker failed: {str(e)}", file=sys.stderr)
        finally:
            if self.rest_interface:
                self.rest_interface.Shutdown()
            self.connected = False
            
    def _process_command(self, command_data):
        """Process individual commands"""
        command = command_data.get('command')
        args = command_data.get('args', {})
        
        try:
            if command == 'status':
                return self._get_status()
            elif command == 'load_and_run':
                return self._load_and_run(args.get('file_path'))
            elif command == 'test':
                return self._test_connection()
            else:
                return self._create_response(False, f"Unknown command: {command}")
                
        except Exception as e:
            return self._create_response(False, f"Command {command} failed: {str(e)}")
    
    def _get_status(self):
        """Get program status"""
        import requests
        try:
            url = "https://localhost:4504/DataService/Integer/SID_CURRENT_PROGRAM_STATUS"
            headers = {'token': self.rest_interface.tokenValue}
            
            response = requests.get(url, headers=headers, verify=False)
            if response.status_code == 200:
                status_code = response.json()
                return self._create_response(True, f"Got status: {status_code}", {"status_code": status_code})
            else:
                return self._create_response(False, f"Status request failed: {response.status_code}")
        except Exception as e:
            return self._create_response(False, f"Status check failed: {str(e)}")
    
    def _load_and_run(self, file_path):
        """Load and run G-code program"""
        try:
            if self.rest_interface.LoadAndRunProgram(file_path):
                return self._create_response(True, f"Successfully loaded and started: {file_path}")
            else:
                return self._create_response(False, f"Failed to load/run program: {file_path}")
        except Exception as e:
            return self._create_response(False, f"Load/run failed: {str(e)}")
    
    def _test_connection(self):
        """Test connection"""
        if self.connected and self.rest_interface:
            return self._create_response(True, "Connection is alive")
        else:
            return self._create_response(False, "Connection is down")
    
    def _create_response(self, success, message, data=None):
        """Create standardized response"""
        response = {"success": success, "message": message}
        if data:
            response["data"] = data
        return response
    
    def send_command(self, command, args=None, timeout=10.0):
        """Send command to worker thread and wait for response"""
        if not self.connected:
            return self._create_response(False, "Not connected to CNC")
        
        command_data = {'command': command, 'args': args or {}}
        self.command_queue.put(command_data)
        
        try:
            response = self.response_queue.get(timeout=timeout)
            return response
        except queue.Empty:
            return self._create_response(False, f"Command {command} timed out")
    
    def shutdown(self):
        """Shutdown the connection"""
        self.shutdown_event.set()
        if self.worker_thread:
            self.worker_thread.join(timeout=5.0)

# Global connection instance
cnc_connection = None

def ensure_connection():
    """Ensure connection is established"""
    global cnc_connection
    
    if cnc_connection is None:
        cnc_connection = PersistentCNCConnection()
        cnc_connection.start_connection_thread()
        
        # Wait for connection to establish
        for i in range(50):  # 5 second timeout
            if cnc_connection.connected:
                break
            time.sleep(0.1)
        
        if not cnc_connection.connected:
            return {"success": False, "message": "Failed to establish initial connection"}
    
    return {"success": True, "message": "Connection ready"}

def main():
    """Main command dispatcher"""
    parser = argparse.ArgumentParser(description='Persistent CNC REST API Wrapper')
    parser.add_argument('command', choices=['connect', 'disconnect', 'status', 'load_and_run', 'test'],
                       help='Command to execute')
    parser.add_argument('--file', help='G-code file path for load_and_run command')
    
    args = parser.parse_args()
    
    try:
        if args.command == 'connect':
            # Establish persistent connection
            result = ensure_connection()
            print(json.dumps(result))
            
        elif args.command == 'disconnect':
            if cnc_connection:
                cnc_connection.shutdown()
            print(json.dumps({"success": True, "message": "Disconnected"}))
            
        elif args.command == 'status':
            connection_result = ensure_connection()
            if connection_result["success"]:
                result = cnc_connection.send_command('status')
                print(json.dumps(result))
            else:
                print(json.dumps(connection_result))
                
        elif args.command == 'load_and_run':
            if not args.file:
                print(json.dumps({"success": False, "message": "Missing --file argument"}))
            else:
                connection_result = ensure_connection()
                if connection_result["success"]:
                    result = cnc_connection.send_command('load_and_run', {'file_path': args.file})
                    print(json.dumps(result))
                else:
                    print(json.dumps(connection_result))
                    
        elif args.command == 'test':
            connection_result = ensure_connection()
            if connection_result["success"]:
                result = cnc_connection.send_command('test')
                print(json.dumps(result))
            else:
                print(json.dumps(connection_result))
                
    except Exception as e:
        error_response = {
            "success": False, 
            "message": f"Command failed: {str(e)}\n{traceback.format_exc()}"
        }
        print(json.dumps(error_response))

if __name__ == "__main__":
    main()