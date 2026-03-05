import subprocess
import platform
import os

# Detect platform
system = platform.system()

if system == "Windows":
    script = "DriverStation\\find_robot_ip.bat"  # Use backslashes on Windows
    cmd = ["cmd", "/c", script]
else:
    script = "DriverStation/find_robot_ip.sh"
    # Make sure the script is executable
    if not os.access(script, os.X_OK):
        os.chmod(script, 0o755)
    cmd = [script]

# Run the script and capture output
result = subprocess.run(cmd, capture_output=True, text=True)

# Process the result
if result.returncode == 0:
    ip = result.stdout.strip()
    print("Robot IP:", ip)
else:
    print("Robot not found")