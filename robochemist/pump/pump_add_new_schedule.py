import subprocess

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

# upload_sketch(
#     sketch_path="/home/szhuang/autohiam_ws/src/robochemist/pump/pump_out",
#     board_fqbn="arduino:avr:uno",
#     port="/dev/ttyACM0"
# )