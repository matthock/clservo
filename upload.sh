#!/bin/bash

ARDUINO=/opt/arduino/arduino
PORT=/dev/ttyACM0

${ARDUINO} --upload tl_feedback_motor_tester.ino --port ${PORT} --board pololu-a-star:avr:a-star32U4
echo "Connecting..."
./monitor/bin/python3 ./monitor/monitor.py
#stty -F ${PORT} 115200 raw -clocal -echo
#cat ${PORT}
