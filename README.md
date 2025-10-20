# RELAVENT Ventilator – Imperial College London Project

A low-cost, open-source ventilator control and monitoring project. This repository contains embedded firmware (Arduino/ino) for the control unit and a Python GUI for plotting and supervision, along with wiring and system diagrams.

## Project Overview

The RELAVENT ventilator project explores building a reliable emergency ventilator controller using off‑the‑shelf components. It includes:

- Embedded control logic for the actuator/valves and basic modes.
- A Python-based supervision and plotting GUI for serial telemetry.
- System diagrams to assist with assembly and integration.

This repository currently contains:
- `control_command_share_test10 copy.i.ino` – Arduino/Teensy firmware sketch for the control unit.
- `control_plot_GUI_test13 copy.py` – Python plotting/GUI client for live data and supervision.
- `Control_unit_diagram.png`, `Full_wiring_diagram.png`, `Power_Supply_unit_diagram.png` – Hardware/system diagrams.
- `Viva.pptx` – Presentation slides.

> Note: Filenames include spaces and the word "copy" from iterations. Consider renaming to stable names (e.g., `controller.ino`, `gui.py`).

## Safety Disclaimer

This project is for research and educational purposes only. It is not a medical device and has not been reviewed or approved by regulatory bodies. Do not use it for patient care.

## Hardware Requirements (example/reference)

- Microcontroller: Arduino (e.g., Uno/MEGA) or Teensy compatible with the `.ino` sketch
- Actuation: Stepper/servo or solenoid valves per your mechanical design
- Sensors: Pressure sensor(s), flow sensor (optional), encoders/limit switches as needed
- Power: Regulated supplies per `Power_Supply_unit_diagram.png`
- Connectivity: USB for serial telemetry

Refer to the provided diagrams:
- `Full_wiring_diagram.png`
- `Control_unit_diagram.png`
- `Power_Supply_unit_diagram.png`

## Firmware (Arduino) – Build & Flash

1. Install Arduino IDE (or Arduino CLI) and required board packages for your target MCU.
2. Open the sketch `control_command_share_test10 copy.i.ino`.
3. Verify required libraries (if any are referenced in the sketch) are installed.
4. Select the correct Board and Port.
5. Upload the sketch.

Using Arduino CLI (example):
```bash
arduino-cli compile --fqbn arduino:avr:mega /path/to/control_command_share_test10\ copy.i.ino
arduino-cli upload  --fqbn arduino:avr:mega -p /dev/ttyACM0 /path/to/control_command_share_test10\ copy.i.ino
```
Adjust `--fqbn` and port for your board.

## Python GUI – Setup & Run

Requirements:
- Python 3.9+
- Packages: likely `pyserial`, `matplotlib`, `numpy` (adjust based on actual imports)

Create and activate a virtual environment, then install dependencies:
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install pyserial matplotlib numpy
```

Run the GUI:
```bash
python "control_plot_GUI_test13 copy.py"
```

Configure the serial port, baud rate, and any runtime parameters in the script as needed. Ensure the firmware is streaming data over serial in the expected format.

## Serial Protocol (tentative)

- Connection: USB CDC (virtual serial)
- Typical baud: 115200 (confirm in the `.ino` sketch)
- Messages: CSV or delimited key-value pairs (confirm in code)

If you share a snippet of your serial framing from the `.ino` and Python files, we can document the packet structure precisely here.

## Development Notes

- Keep logic for safety guards (pressure limits, timing, alarms) explicit and well‑tested.
- Use clear naming and avoid magic numbers; extract tunables as constants.
- Prefer robust parsing and validation on the Python side to avoid GUI lockups.

## Roadmap / TODO

- Clean up filenames and create a stable module structure.
- Pin out and BOM documentation based on current diagrams.
- Add unit tests for critical control loops and parsing.
- Package Python GUI with a `requirements.txt` and optional `pyproject.toml`.

## Contributing

Contributions are welcome. Open an issue to discuss changes or improvements.

## License

No license is granted. All rights reserved. This project and its contents belong to Imperial College London. Please contact Imperial College London for permissions or usage inquiries.

## Acknowledgements

- Project belongs to Imperial College London.
- Supervision by Dr. Joseph Van Batenburg-Sherwood, Imperial College London.
- Imperial College London project context and collaborators.

---

Repository: `STV1222/RELAVENT-ventilator-Imperial-College-London-Project-`  
GitHub: `https://github.com/STV1222/RELAVENT-ventilator-Imperial-College-London-Project-.git`
