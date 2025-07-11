import subprocess
import serial
import time

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

    except subprocess.CalledProcessError as e:
        print("❌ Error during upload:", e)

upload_sketch(
    sketch_path="/home/szhuang/autohiam_ws/src/robochemist/pump/pump_out_water",
    board_fqbn="arduino:avr:uno",
    port="/dev/ttyACM0"
)
time.sleep(90)
print("Old water has been pumped out.")

upload_sketch(
    sketch_path="/home/szhuang/autohiam_ws/src/robochemist/pump/pump_in_water",
    board_fqbn="arduino:avr:uno",
    port="/dev/ttyACM0"
)
time.sleep(90)
print("New water has been pumped in.")

upload_sketch(
    sketch_path="/home/szhuang/autohiam_ws/src/robochemist/pump/minimal_sketch",
    board_fqbn="arduino:avr:uno",
    port="/dev/ttyACM0"
)
print("Minimal sketch is uploaded.")