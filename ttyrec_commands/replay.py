#!/usr/bin/env python3
import time
import sys
import subprocess
from functools import partial

# Create a print function that flushes immediately
flushprint = partial(print, flush=True)

def slow_type(text, delay=0.1):
    for char in text:
        sys.stdout.write(char)
        sys.stdout.flush()  # <-- Critical!
        time.sleep(delay)
    sys.stdout.write('\n')
    sys.stdout.flush()

with open('commands.txt') as f:
    for line in f:
        line = line.strip()
        if not line: continue
        
        flushprint("$ ", end='')
        slow_type(line)
        
        # Execute with output capture
        process = subprocess.run(line, shell=True, 
                               stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT,
                               text=True)
        
        # Print captured output immediately
        flushprint(process.stdout)
        time.sleep(2)  # Extra pause after command