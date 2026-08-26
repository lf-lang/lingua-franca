#!/bin/bash

# Check if a program is provided
if [ -z "$1" ]; then
  echo "Usage: $0 <command>"
  exit 1
fi

# Initialize run counter
count=0

# Run the program in a loop
while true; do
	echo "***************************************** $count"
  # Execute the program
  "$@"
   
  # Check if the program failed
  if [ $? -ne 0 ]; then
    echo "************************* Program failed after $count successful runs."
    exit 1
  fi
   
  # Increment the run counter
  ((count++))
done