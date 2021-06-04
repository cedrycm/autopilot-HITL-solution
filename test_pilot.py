import os, sys, struct, math, subprocess
import numpy as np

from collections import namedtuple

from zip_sim import TELEMETRY_STRUCT
from pilots.pilot_creator import select_autopilot, select_manualpilot

# set stdin to read bytes
stdin = sys.stdin.buffer

def run_pilot():
    """autopilot selection is done with the provided function 
    
    -for autopilots use 'select_autopilot() 
    -for manual piloting use 'select_manualpilot()"""
    
    #Concrete Pilot Selection 
    pilot = select_autopilot(1)

    while True:
        try:
            # for tele_input in TELEMETRY_STRUCT.iter_unpack(sys.stdin.buffer.read()):
            tele_input = bytearray(stdin.read(TELEMETRY_STRUCT.size))
            
            pilot.interpret_telemetry(tele_input)
            pilot.send_command()
            
            if "Exit" == tele_input:
                break

        except (BrokenPipeError, IOError):
            #ignore subprocess flush command
            pass

if __name__ == "__main__":
    run_pilot()
