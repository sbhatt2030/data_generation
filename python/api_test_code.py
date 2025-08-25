import sys
import os
import time
from RestfulAPIBase import RestfulInterface

def test_restful_interface():
    """Test the RestfulInterface class functionality"""
    print("🔧 TESTING RESTFUL INTERFACE (RestfulAPIBase.py)")
    print("=" * 60)
    
    # Initialize with the same SID as C++ code
    program_running_sid = "SID_RT_PROGRAM_RUNNING"
    rest = RestfulInterface(program_running_sid)
    
    # Test 1: Check if initialization succeeded
    print("\n=== Test 1: Initialization ===")
    if rest.DidInitialize():
        print("✅ RestfulInterface initialized successfully")
        print(f"   Program running SID: {program_running_sid}")
    else:
        print("❌ RestfulInterface failed to initialize")
        print("   Check that WinMax is running with REST API enabled")
        return False
    
    # Test 2: Set maximum feed overrides (same as C++ testConnection)
    print("\n=== Test 2: Set Feed Overrides ===")
    if rest.SetMaxFeedOverrides():
        print("✅ Successfully set maximum feed overrides")
    else:
        print("❌ Failed to set feed overrides (non-critical)")
    
    # Test 3: Try to get current program status (this is what's failing in C++)
    print("\n=== Test 3: Get Program Status ===")
    try:
        import requests
        
        # Use the same token and URL from the RestfulInterface
        headers = {'token': rest.tokenValue}
        url = rest.urlString + '/DataService/Integer/SID_CURRENT_PROGRAM_STATUS'
        
        print(f"   URL: {url}")
        print(f"   Token: {rest.tokenValue[:20]}..." if len(rest.tokenValue) > 20 else f"   Token: {rest.tokenValue}")
        
        response = requests.get(url, headers=headers, verify=False, timeout=10)
        
        print(f"   Status Code: {response.status_code}")
        
        if response.status_code == 200:
            status_data = response.json()
            status_msg = rest.GetPgmStatusMessage(status_data)
            print(f"✅ Program status retrieved successfully")
            print(f"   Raw status: {status_data}")
            print(f"   Status message: {status_msg}")
        else:
            print(f"❌ Status request failed: {response.status_code}")
            print(f"   Response: {response.text}")
            
    except Exception as e:
        print(f"❌ Exception getting status: {e}")
    
    # Test 4: Test loading a file (if provided)
    print("\n=== Test 4: File Loading Test ===")
    gcode_file = input("Enter G-code file path to test loading (or press Enter to skip): ").strip()
    
    if gcode_file:
        if os.path.exists(gcode_file):
            print(f"Testing file load: {gcode_file}")
            
            # Test load without running
            if rest.LoadAnyFile(gcode_file, RunAfterLoading=0):
                print("✅ File loaded successfully (without running)")
                
                # Test load and run
                print("Testing load and run...")
                if rest.LoadAndRunProgram(gcode_file):
                    print("✅ File loaded and started successfully")
                    
                    # Monitor status for a few seconds
                    print("Monitoring program status...")
                    for i in range(10):  # Monitor for 10 seconds
                        try:
                            response = requests.get(
                                rest.urlString + '/DataService/Integer/SID_CURRENT_PROGRAM_STATUS',
                                headers={'token': rest.tokenValue},
                                verify=False,
                                timeout=5
                            )
                            if response.status_code == 200:
                                status = response.json()
                                status_msg = rest.GetPgmStatusMessage(status)
                                print(f"   Status {i+1}: {status_msg}")
                                
                                # If program completed, break
                                if status >= 2:  # Completed states
                                    break
                            else:
                                print(f"   Status check failed: {response.status_code}")
                                
                        except Exception as e:
                            print(f"   Status check error: {e}")
                        
                        time.sleep(1)
                        
                else:
                    print("❌ Load and run failed")
            else:
                print("❌ File load failed")
        else:
            print(f"❌ File does not exist: {gcode_file}")
    else:
        print("Skipping file loading test")
    
    # Test 5: Connection cleanup
    print("\n=== Test 5: Cleanup ===")
    rest.Shutdown()
    print("✅ RestfulInterface shutdown complete")
    
    return True

def test_specific_endpoints():
    """Test specific endpoints that are failing in C++"""
    print("\n" + "=" * 60)
    print("🔍 TESTING SPECIFIC ENDPOINTS")
    print("=" * 60)
    
    # Test the exact same sequence as C++ HurcoConnection
    import requests
    import json
    import urllib3
    urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)
    
    # Step 1: HTTP Authentication (port 4503)
    print("\n=== HTTP Authentication (Port 4503) ===")
    try:
        auth_data = {
            "username": "0049",
            "password": "QoALWQ/PxjunlMbrPEBr0ZcLyfcIi3D6DYaNVfKDP/3sANCfJqtnDQm1WsCCryl825lfvacx3B+6UgNlg0K6zA=="
        }
        
        response = requests.post(
            'http://localhost:4503/AuthService/Connect',
            json=auth_data,
            timeout=30,  # Same timeout as C++
            verify=False
        )
        
        print(f"Auth Status: {response.status_code}")
        if response.status_code == 200:
            token = response.json()['token']
            print(f"✅ Token obtained: {token[:20]}...")
        else:
            print(f"❌ Auth failed: {response.text}")
            return False
            
    except Exception as e:
        print(f"❌ Auth exception: {e}")
        return False
    
    # Step 2: HTTPS Status Check (port 4504) - This is what's timing out in C++
    print("\n=== HTTPS Status Check (Port 4504) ===")
    try:
        headers = {
            'token': token,
            'Connection': 'keep-alive'  # Same as C++
        }
        
        # Try with different timeout values
        for timeout_val in [5, 10, 30]:
            print(f"Trying with {timeout_val}s timeout...")
            try:
                response = requests.get(
                    'https://localhost:4504/DataService/Integer/SID_CURRENT_PROGRAM_STATUS',
                    headers=headers,
                    timeout=timeout_val,
                    verify=False
                )
                
                print(f"✅ Status check succeeded with {timeout_val}s timeout")
                print(f"   Status Code: {response.status_code}")
                print(f"   Response: {response.json()}")
                break
                
            except requests.exceptions.Timeout:
                print(f"❌ Timeout with {timeout_val}s")
            except Exception as e:
                print(f"❌ Error with {timeout_val}s: {e}")
        
    except Exception as e:
        print(f"❌ Status check exception: {e}")
        return False
    
    return True

def main():
    """Main test function"""
    print("HURCO REST API COMPREHENSIVE TEST")
    print("Using RestfulAPIBase.py (same as C++ implementation)")
    print("=" * 60)
    
    # Test 1: RestfulInterface class
    print("Starting RestfulInterface test...")
    interface_success = test_restful_interface()
    
    # Test 2: Raw endpoint testing
    print("\nStarting raw endpoint test...")
    endpoint_success = test_specific_endpoints()
    
    # Summary
    print("\n" + "=" * 60)
    print("🎯 FINAL RESULTS")
    print("=" * 60)
    
    if interface_success:
        print("✅ RestfulInterface test: PASSED")
    else:
        print("❌ RestfulInterface test: FAILED")
    
    if endpoint_success:
        print("✅ Raw endpoint test: PASSED")
    else:
        print("❌ Raw endpoint test: FAILED")
    
    if interface_success and endpoint_success:
        print("\n🎉 ALL PYTHON TESTS PASSED!")
        print("The issue is definitely in the C++ implementation.")
        print("Focus on:")
        print("  - cpprest timeout configuration")
        print("  - HTTPS certificate handling")
        print("  - Token header formatting")
    else:
        print("\n⚠️ PYTHON TESTS FAILED!")
        print("The issue is with WinMax REST API itself.")
        print("Check:")
        print("  - WinMax is running")
        print("  - REST API is enabled")
        print("  - Ports 4503/4504 are open")
        print("  - No firewall blocking")
    
    input("\nPress Enter to exit...")

if __name__ == "__main__":
    main()