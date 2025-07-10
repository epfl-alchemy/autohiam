import subprocess
import time
import sys

num_cycle = 10

# List of (script, delay_in_seconds)
scripts = [
    ("/home/szhuang/autohiam_ws/src/robochemist/robochemist/hiam_infusion_schedule.py", 1 * 60 * 60), # sample in iron-ion solution
    ("/home/szhuang/autohiam_ws/src/robochemist/pump/change_ammonia_schedule.py", 10), # sample still in iron-ion solution, this will pump in new ammonia for this run
    ("/home/szhuang/autohiam_ws/src/robochemist/robochemist/hiam_precipitation_schedule.py", 5 * 60), # sample in ammonia solution
    ("/home/szhuang/autohiam_ws/src/robochemist/robochemist/hiam_washing_schedule.py", 15 * 60), # sample in water
    ("/home/szhuang/autohiam_ws/src/robochemist/pump/change_water_schedule.py", 10), # sample still in water, this will pump in new water for next run (takes 7 mins)
    ("/home/szhuang/autohiam_ws/src/robochemist/robochemist/hiam_transition_schedule.py", 5 * 60),
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

def write_live_status(cycle, script):
    with open("current_status.txt", "w") as f:
        f.write(f"Cycle {cycle} | Running: {script} | Time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")

# Repeat "num_cycle" times
for cycle in range(1, num_cycle + 1):
    print(f"\n=== Starting Cycle {cycle}/{num_cycle}, Total {num_cycle} cycles ===")
    for script, delay in scripts:
        print(f"\nRunning {script}...")

        write_live_status(cycle, script)

        subprocess.run([sys.executable, script])  # <--- fix applied here
        if delay > 0:
            countdown(delay)
            print(f"Current cycle numer {cycle}")

print("\nAll cycles completed.")
