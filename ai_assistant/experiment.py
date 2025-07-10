# experiment.py
import argparse
import time

parser = argparse.ArgumentParser()
parser.add_argument("--cycles", type=int, default=9)
args = parser.parse_args()

print(f"Running HIAM experiment with {args.cycles} cycles.")
for i in range(1, args.cycles + 1):
    print(f"Cycle {i}/{args.cycles}")
    time.sleep(1)  # simulate action
