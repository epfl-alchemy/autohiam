import subprocess
import time
import sys
import os

num_cycle = 2

# List of (script, delay_in_seconds)
scripts = [
    ("/home/szhuang/autohiam_ws/src/hiam_gen2/test1.py", 5), 
    ("/home/szhuang/autohiam_ws/src/hiam_gen2/test2.py", 5), 
    ("/home/szhuang/autohiam_ws/src/hiam_gen2/test3.py", 5), 
    ("/home/szhuang/autohiam_ws/src/hiam_gen2/test4.py", 5),
]

status_file = "live_status_log.txt"

# Clear status file at start
with open(status_file, "w") as f:
    f.write(f"=== Live Status Log ===\nStarted at {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")

def countdown(seconds, prev_script, next_script):
    for remaining in range(seconds, 0, -1):
        try:
            hrs, rem = divmod(remaining, 3600)
            mins, secs = divmod(rem, 60)
            sys.stdout.write(
                f"\rWaiting after {prev_script}: {hrs:02d}h:{mins:02d}m:{secs:02d}s remaining before running {next_script}"
            )
            sys.stdout.flush()
            time.sleep(1)
        except KeyboardInterrupt:
            print("\nCountdown interrupted by user.")
            log_event(f"KeyboardInterrupt during countdown after {prev_script}")
            sys.exit(1)
    print(f"\rWaiting complete after {prev_script}, now running {next_script}.{' '*40}")

def write_live_status(cycle, script):
    with open(status_file, "a") as f:
        f.write(f"Cycle {cycle} | Running: {os.path.basename(script)} | Time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")

def log_event(message):
    with open(status_file, "a") as f:
        f.write(f"[!] {message} | Time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")

# Run cycles
try:
    for cycle in range(1, num_cycle + 1):
        print(f"\n\n=== Starting Cycle {cycle}/{num_cycle} ===")
        for idx, (script, delay) in enumerate(scripts):
            script_name = os.path.basename(script)
            print(f"\n--- Cycle {cycle}/{num_cycle} | Executing {script_name} ---")

            write_live_status(cycle, script)

            try:
                subprocess.run([sys.executable, script], check=True, stdout=None, stderr=None)
            except KeyboardInterrupt:
                print(f"\n[Interrupted] KeyboardInterrupt during {script}")
                log_event(f"KeyboardInterrupt during script: {os.path.basename(script)}")
                sys.exit(1)
            except subprocess.CalledProcessError as e:
                log_event(f"Error running script: {os.path.basename(script)} | Return code: {e.returncode}")

            # Determine the next script's name (if any)
            next_script_name = os.path.basename(scripts[idx + 1][0]) if idx + 1 < len(scripts) else "next cycle or end"

            if delay > 0:
                countdown(delay, os.path.basename(script), next_script_name)

except KeyboardInterrupt:
    print("\n[Interrupted] KeyboardInterrupt during main loop.")
    log_event("KeyboardInterrupt during main loop")
    sys.exit(1)

print("\nAll cycles completed.")
log_event(f"All {num_cycle} cycles completed")
