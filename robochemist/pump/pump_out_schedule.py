import subprocess
import serial
import time

def upload_sketch_without_output(sketch_path, board_fqbn, port):
    try:
        # Compile the sketch
        subprocess.run([
            "arduino-cli", "compile", "--fqbn", board_fqbn, sketch_path
        ], check=True)

        # Upload the sketch
        subprocess.run([
            "arduino-cli", "upload", "-p", port, "--fqbn", board_fqbn, sketch_path
        ], check=True)

        print("✅ Upload complete.")

    except subprocess.CalledProcessError as e:
        print("❌ Error during upload:", e)

def upload_sketch(sketch_path, board_fqbn, port):
    try:
        # Compile the sketch
        subprocess.run([
            "arduino-cli", "compile", "--fqbn", board_fqbn, sketch_path
        ], check=True)

        # Upload the sketch
        subprocess.run([
            "arduino-cli", "upload", "-p", port, "--fqbn", board_fqbn, sketch_path
        ], check=True)

        print("✅ Upload complete.")
        time.sleep(5)  # Allow Arduino to reboot

        # Open serial port and listen for output
        with serial.Serial(port, baudrate=9600, timeout=1) as ser:
            start_time = time.time()
            while time.time() - start_time < 130:  # Read for up to 130 seconds
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='replace').strip()
                    if line:
                        print("📟 Arduino:", line)
                        # If the sketch signals it's done, break early
                        if "✅ Pump stopped." in line:
                            print("🛑 Pump cycle complete.")
                            break

    except subprocess.CalledProcessError as e:
        print("❌ Error during upload:", e)


upload_sketch(
    sketch_path="/home/szhuang/autohiam_ws/src/robochemist/pump/pump_out_iron",
    board_fqbn="arduino:avr:uno",
    port="/dev/ttyACM0"
)
print("Old iron solution has been pumped out.")

upload_sketch(
    sketch_path="/home/szhuang/autohiam_ws/src/robochemist/pump/pump_out_ammonia",
    board_fqbn="arduino:avr:uno",
    port="/dev/ttyACM0"
)
print("Old ammonia solution has been pumped out.")

upload_sketch(
    sketch_path="/home/szhuang/autohiam_ws/src/robochemist/pump/pump_out_water",
    board_fqbn="arduino:avr:uno",
    port="/dev/ttyACM0"
)
print("Old water has been pumped out.")

upload_sketch_without_output(
    sketch_path="/home/szhuang/autohiam_ws/src/robochemist/pump/minimal_sketch",
    board_fqbn="arduino:avr:uno",
    port="/dev/ttyACM0"
)
print("Minimal sketch is uploaded.")