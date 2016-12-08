#!/bin/bash

while true; do
  ps cax | grep QUADCOPTER > /dev/null
  if [ $? -eq 0 ]; then
    echo "Process is running."
  else
    echo "Process is not running."
    /home/pi/git/build/src/RTPiDrone 
  fi
  sleep 10
done
