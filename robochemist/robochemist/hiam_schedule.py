import subprocess
import time
import sys

# List of (script, delay_in_seconds)
# scripts = [
#     ("/home/szhuang/autohiam_ws/src/robochemist/robochemist/hiam_infusion_schedule.py", 1 * 60 * 60),
#     ("/home/szhuang/autohiam_ws/src/robochemist/robochemist/hiam_precipitation_schedule.py", 5 * 60),
#     ("/home/szhuang/autohiam_ws/src/robochemist/robochemist/hiam_washing_schedule.py", 15 * 60),
#     ("/home/szhuang/autohiam_ws/src/robochemist/robochemist/hiam_transition_schedule.py", 6 * 60),
# ]
scripts = [
    ("/home/szhuang/autohiam_ws/src/robochemist/robochemist/hiam_infusion_schedule.py", 5*60),
    ("/home/szhuang/autohiam_ws/src/robochemist/robochemist/hiam_precipitation_schedule.py", 5*60),
    ("/home/szhuang/autohiam_ws/src/robochemist/robochemist/hiam_washing_schedule.py", 5*60),
    ("/home/szhuang/autohiam_ws/src/robochemist/robochemist/hiam_transition_schedule.py", 5*60),
]


def countdown(seconds):
    try:
        for remaining in range(seconds, 0, -1):
            hrs, rem = divmod(remaining, 3600)
            mins, secs = divmod(rem, 60)
            sys.stdout.write(f"\rWaiting: {hrs:02d}h:{mins:02d}m:{secs:02d}s remaining")
            sys.stdout.flush()
            time.sleep(1)
        print("\rWaiting complete.                          ")
    except KeyboardInterrupt:
        print("\nCountdown interrupted by user.")
        exit(1)

# Repeat 5 times
for cycle in range(1, 6):
    print(f"\n=== Starting Cycle {cycle}/5 ===")
    for script, delay in scripts:
        print(f"\nRunning {script}...")
        subprocess.run([sys.executable, script])  # <--- fix applied here
        if delay > 0:
            countdown(delay)

print("\nAll cycles completed.")
