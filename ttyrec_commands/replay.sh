#!/bin/bash
# Adjust speed with `pv -qL [chars/sec]` and pauses with `sleep`
while IFS= read -r line; do
  echo -n "$ "       # Simulate the prompt (optional)
  echo "$line" | pv -qL 10  # 10 chars/second (adjust speed)
  eval "$line"       # Execute the command
  sleep 1            # Pause after each command (optional)
done < commands.txt
