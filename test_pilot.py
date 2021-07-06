import sys

from zip_sim import TELEMETRY_STRUCT
from src.pilots.pilot_creator import PilotDirector

# set stdin to read bytes
stdin = sys.stdin.buffer


def run_pilot(pilot_select="AUTO"):
    """pilot selection is done with the provided string values:
        -"AUTO"   : Default python concrete autopilot class
        -"UNO"    : uses controller as interface for embedded arduino uno solution
                    see config.py for arduino init parameters
        -"MANUAL" : Uses keyboard as controller for pilot"""

    if (
        pilot_select == "AUTO" or pilot_select == "UNO" or pilot_select == "MANUAL"
    ):  # currently only two pilot implementations
        # Concrete Pilot Selection
        pilot = PilotDirector.select_pilot(pilot_select)

        while True:
            try:
                tele_input = bytearray(stdin.read(TELEMETRY_STRUCT.size))
                if len(tele_input) == 44:
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
