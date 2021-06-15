import sys


from collections import namedtuple

from zip_sim import TELEMETRY_STRUCT
from src.pilots.pilot_creator import select_autopilot, select_manualpilot

# set stdin to read bytes
stdin = sys.stdin.buffer


def run_pilot(pilot_select="AUTO"):
    """autopilot selection is done with the provided function

    -for autopilots use 'select_autopilot()
    -for manual piloting use 'select_manualpilot()"""

    if (
        pilot_select == "AUTO" or pilot_select == "UNO"
    ):  # currently only two pilot implementations
        # Concrete Pilot Selection
        pilot = select_autopilot(pilot_select)

        while True:
            try:
                # for tele_input in TELEMETRY_STRUCT.iter_unpack(sys.stdin.buffer.read()):
                tele_input = bytearray(stdin.read(TELEMETRY_STRUCT.size))
                if (len(tele_input) == 44):
                    pilot.interpret_telemetry(tele_input)
                    pilot.send_command()
                else:
                    break

            except (EOFError, BrokenPipeError, IOError, TimeoutError):
                # ignore subprocess flush command
                break


if __name__ == "__main__":
    pilot_select = None
    if len(sys.argv) > 1:
        pilot_select = sys.argv[1]
        run_pilot(pilot_select)
    else:
        run_pilot()
