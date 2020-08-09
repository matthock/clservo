import serial
import signal
import sys

running = True

def signal_handler(sig, frame):
  global running
  print("Received Ctrl-C")
  running = False
signal.signal(signal.SIGINT, signal_handler)

with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as ser:
  while running:
    line = ser.readline()
    if (line):
      print(line)
