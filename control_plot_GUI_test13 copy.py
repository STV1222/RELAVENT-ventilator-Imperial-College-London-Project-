import sys
import queue
import threading
from collections import deque
from PyQt6.QtGui import QIntValidator, QFont, QColor, QPalette
import serial
from serial.tools import list_ports
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QPushButton,
    QLabel, QWidget, QGridLayout, QGroupBox, QLineEdit, QFormLayout, QStackedWidget,
    QFrame, QSizePolicy
)
from pyqtgraph import PlotWidget
import pyqtgraph as pg
import csv
import os
import time

# Queue for sending commands to the Arduino
command_queue = queue.Queue()

def process_commands():
    """Continuously process and send commands from the command queue."""
    while True:
        command = command_queue.get()
        if ser and command:
            with serial_lock:
                ser.write((command + "\n").encode("utf-8"))
                latest_output_queue.put(f"Sent Command: {command}")
        command_queue.task_done()

# Automatically find the correct serial port
def find_serial_port():
    ports = list_ports.comports()
    for port in ports:
        print(f"Detected: {port.device} - {port.description}")
        if "Teensy" in port.description or "usbmodem" in port.device:
            return port.device
    raise RuntimeError("No suitable device found.")

# Serial port configuration
try:
    SERIAL_PORT = find_serial_port()
    BAUD_RATE = 1000000
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Connected to {SERIAL_PORT}")
except Exception as e:
    print(f"Error opening serial port: {e}")
    ser = None

# Buffers for real-time data
BUFFER_SIZE = 1000
data_buffers = {
    "time": deque(maxlen=BUFFER_SIZE),
    "reservoir_data": deque(maxlen=BUFFER_SIZE),
    "system_data": deque(maxlen=BUFFER_SIZE),
    "lung_data": deque(maxlen=BUFFER_SIZE),
    "oxygen_data": deque(maxlen=BUFFER_SIZE),
    "inh_volume_data": deque(maxlen=BUFFER_SIZE),
    "exh_volume_data": deque(maxlen=BUFFER_SIZE),
    "flow_data": deque(maxlen=BUFFER_SIZE),
}

# CSV file setup
CSV_FILE_PATH = "data_log.csv"

def create_csv_file():
    if not os.path.exists(CSV_FILE_PATH):
        with open(CSV_FILE_PATH, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time", "Reservoir Pressure", "System Pressure", "Lung Pressure", 
                            "Oxygen Level", "Inhalation Volume", "Exhalation Volume", "Flow Rate"])
        print(f"Created new CSV file: {CSV_FILE_PATH}")

csv_batch = []
def append_to_csv(data):
    global csv_batch
    csv_batch.append(data)
    if len(csv_batch) >= 10:
        with open(CSV_FILE_PATH, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(csv_batch)
        csv_batch = []

serial_lock = threading.Lock()
latest_output_queue = queue.Queue()

def read_serial():
    if ser:
        while True:
            try:
                with serial_lock:
                    line = ser.readline().decode("utf-8").strip()
                if line:
                    # print(f"DEBUG: Received Serial Data: {line}")
                    latest_output_queue.put(f"Raw: {line}")
                    if line.startswith("ReservoirPressure:"):
                        parse_data(line)
                    elif line.startswith("PRE_DPRESS:"):
                        val = line.split(":")[1]
                        dashboard.pre_dpress = float(val) if val.replace(".", "").replace("-", "").isdigit() else None
                        # print(f"DEBUG: PreDPress Updated: {dashboard.pre_dpress}")
                    elif line.startswith("POST_DPRESS:"):
                        val = line.split(":")[1]
                        dashboard.post_dpress = float(val) if val.replace(".", "").replace("-", "").isdigit() else None
                        # print(f"DEBUG: PostDPress Updated: {dashboard.post_dpress}")
                    elif line.startswith("PRE_DFLOW:"):
                        val = line.split(":")[1]
                        dashboard.pre_dflow = float(val) if val.replace(".", "").replace("-", "").isdigit() else None
                        # print(f"DEBUG: PreDFlow Updated: {dashboard.pre_dflow}")
                    elif line.startswith("ALARM:"):
                        handle_alarm_line(line)
                    else:
                        print(f"DEBUG: Unrecognized Serial Line: {line}")
            except Exception as e:
                print(f"Error reading serial: {e}")
                latest_output_queue.put(f"Serial Error: {e}")
                time.sleep(0.1)

def parse_data(line):
    """Parse incoming serial data line and update data buffers, CSV file, and dashboard states."""
    try:
        data_parts = line.split(", ")
        if len(data_parts) != 7:
            raise ValueError(f"Expected 7 data parts, got {len(data_parts)}: {line}")

        reservoir_pressure = float(data_parts[0].split(":")[1])
        system_pressure = float(data_parts[1].split(":")[1])
        oxygen_level = float(data_parts[2].split(":")[1])
        if oxygen_level < 0 or oxygen_level > 100:
            print(f"WARNING: Oxygen level out of range: {oxygen_level}. Clamping to 0-100.")
            latest_output_queue.put(f"Warning: Oxygen level anomaly: {oxygen_level}")
            oxygen_level = max(0, min(100, oxygen_level))
        inh_volume = float(data_parts[3].split(":")[1])
        exh_volume = float(data_parts[4].split(":")[1])
        flow = float(data_parts[5].split(":")[1])
        valve_d_state = data_parts[6].split(":")[1]

        if flow > 0 and dashboard.lung_resistance is not None:  # Inhalation
            Q_in = flow / 60.0
            lung_pressure = system_pressure - dashboard.lung_resistance * Q_in
        elif flow < 0 and dashboard.lung_resistance is not None:  # Exhalation
            Q_ex = -flow / 60.0
            lung_pressure = system_pressure + dashboard.lung_resistance * Q_ex
        else:
            lung_pressure = system_pressure

        data_buffers["reservoir_data"].append(reservoir_pressure)
        data_buffers["system_data"].append(system_pressure)
        data_buffers["lung_data"].append(lung_pressure)
        data_buffers["oxygen_data"].append(oxygen_level)
        data_buffers["inh_volume_data"].append(inh_volume)
        data_buffers["exh_volume_data"].append(exh_volume)
        data_buffers["flow_data"].append(flow)

        time_stamp = len(data_buffers["time"])
        append_to_csv([time_stamp, reservoir_pressure, system_pressure, lung_pressure,
                       oxygen_level, inh_volume, exh_volume, flow])

        # print(f"DEBUG: Parsed Values - Reservoir: {reservoir_pressure:.2f}, System: {system_pressure:.2f}, "
        #       f"Lung: {lung_pressure:.2f}, Oxygen: {oxygen_level:.2f}, Inh Vol: {inh_volume:.2f}, "
        #       f"Exh Vol: {exh_volume:.2f}, Flow: {flow:.2f}, Valve D: {valve_d_state}")

        if dashboard.prev_valve_d_state == "OPEN" and valve_d_state == "OPEN":
            dashboard.pre_dflow = flow
            dashboard.pre_dflow_label.setText(f"{dashboard.pre_dflow:.2f}")
            print(f"DEBUG: PreDFlow Updated (Valve D OPEN): {dashboard.pre_dflow:.2f}")

        if dashboard.prev_valve_d_state == "OPEN" and valve_d_state == "CLOSED" and dashboard.post_dflow is None:
            dashboard.post_dflow = flow
            dashboard.post_dflow_label.setText(f"{dashboard.post_dflow:.2f}")
            print(f"DEBUG: PostDFlow Captured (Valve D first CLOSED): {dashboard.post_dflow:.2f}")

        if dashboard.prev_valve_d_state == "OPEN" and valve_d_state == "CLOSED":
            print("DEBUG: Valve D State Changed to CLOSED. Starting 1-second pressure collection.")
            dashboard.start_pressure_collection()

        if dashboard.collecting_pressure:
            dashboard.pressure_collection.append((system_pressure, flow))

        dashboard.prev_valve_d_state = valve_d_state
        dashboard.detect_end_of_breath()

        if not data_buffers["time"]:
            data_buffers["time"].append(0)
        else:
            data_buffers["time"].append(data_buffers["time"][-1] + 1)

        if len(data_buffers["system_data"]) > 1:
            prev_system_pressure = data_buffers["system_data"][-2]
            pressure_change = abs(system_pressure - prev_system_pressure)
            if pressure_change > 50:
                print(f"WARNING: Sudden system pressure change detected: {system_pressure:.2f} "
                      f"from {prev_system_pressure:.2f}")
                latest_output_queue.put(f"Warning: Sudden pressure change: {pressure_change:.2f}")

    except ValueError as ve:
        print(f"Error parsing data: {ve}")
        latest_output_queue.put(f"Parse Error: {ve}")
    except Exception as e:
        print(f"Unexpected error in parse_data: {e}")
        latest_output_queue.put(f"Unexpected Error: {e}")

def handle_alarm_line(line):
    if "High Pressure" in line:
        dashboard.trigger_alarm("High Pressure detected! Valve C close, Valve D open")
    elif "Low Pressure" in line:
        dashboard.trigger_alarm("Low Pressure detected!")
    else:
        dashboard.trigger_alarm(line.replace("ALARM: ", ""))

create_csv_file()

# Define global styles
BUTTON_STYLE = """
    QPushButton {
        background-color: #2c3e50;
        color: white;
        border: none;
        border-radius: 4px;
        padding: 8px;
        font-weight: bold;
        min-height: 30px;
    }
    QPushButton:hover {
        background-color: #34495e;
    }
    QPushButton:pressed {
        background-color: #1abc9c;
    }
"""

GROUPBOX_STYLE = """
    QGroupBox {
        border: 1px solid #3498db;
        border-radius: 5px;
        margin-top: 10px;
        font-weight: bold;
        color: #2c3e50;
    }
    QGroupBox::title {
        subcontrol-origin: margin;
        subcontrol-position: top center;
        padding: 0 5px;
        color: #3498db;
    }
"""

VALUE_DISPLAY_STYLE = """
    QLabel {
        background-color: #1e2330;
        color: white;
        border: 1px solid #3498db;
        border-radius: 4px;
        padding: 5px;
        font-weight: bold;
        qproperty-alignment: AlignCenter;
    }
"""

class VentilatorDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Ventilator Monitoring Dashboard")
        self.setGeometry(100, 100, 1400, 900)
        self.setMinimumSize(1400, 900)
        self.setStyleSheet("background-color: #f0f0f0;")

        self.collecting_pressure = False
        self.pressure_collection = []
        self.collection_timer = None

        # PEEP control variables
        self.ep_eep = 1.5
        self.delta_ep_eep = 0.1
        self.last_peep_measured = 0.0
        self.last_valve_d_close_time = 0
        self.hardware_lag = 50
        self.flow_zero_threshold = 0.10  # L/min

        self.high_pressure_alarm_threshold = 35
        self.low_pressure_alarm_threshold = -5
        self.blockage_alarm_threshold = 5
        self.hypervent_threshold = 30
        self.hypovent_threshold = -30

        self.breath_count = 0
        self.last_user_setting_breath_count = 0
        self.recent_inh_volumes = []
        self.alarm_active = False
        self.exh_pressure_values = []
        self.min_exh_pressure = None
        self.prev_valve_d_state = None
        self.pre_dpress = None
        self.post_dpress = None
        self.pre_dflow = None
        self.post_dflow = None
        self.lung_resistance = None
        self.exit_confirmed = False
        self.stop_confirmed = False
        self.stored_settings = {}

        self.pre_dpress_label = QLabel("N/A")
        self.post_dpress_label = QLabel("N/A")
        self.pre_dflow_label = QLabel("N/A")
        self.post_dflow_label = QLabel("N/A")
        self.lung_resistance_label = QLabel("N/A")
        self.min_exh_pressure_label = QLabel("N/A")

        for label in [self.pre_dpress_label, self.post_dpress_label, self.pre_dflow_label,
                      self.post_dflow_label, self.lung_resistance_label, self.min_exh_pressure_label]:
            label.setStyleSheet("font-size: 12pt; font-weight: bold; color: white; background-color: #1e2330; padding: 3px; border-radius: 3px;")

        self.central_widget = QStackedWidget()
        self.setCentralWidget(self.central_widget)
        self.dashboard_page = self.create_dashboard_page()
        self.settings_page = self.create_settings_page()
        self.advanced_page = self.create_advanced_page()
        self.central_widget.addWidget(self.dashboard_page)
        self.central_widget.addWidget(self.settings_page)
        self.central_widget.addWidget(self.advanced_page)
        self.central_widget.setCurrentWidget(self.dashboard_page)

        self.initialize_buffers()
        self.debug_timer = QTimer()
        self.debug_timer.timeout.connect(self.print_current_values)
        self.debug_timer.start(1000)

    def start_pressure_collection(self):
        self.collecting_pressure = True
        self.pressure_collection = []
        self.collection_timer = threading.Timer(1.0, self.stop_pressure_collection)
        self.collection_timer.start()
        print("DEBUG: Pressure and flow collection started for 1 second.")

    def stop_pressure_collection(self):
        self.collecting_pressure = False
        if self.pressure_collection:
            min_pressure = min(self.pressure_collection, key=lambda x: x[0])[0]
            max_pressure = max(self.pressure_collection, key=lambda x: x[0])[0]
            
            self.pre_dpress = min_pressure
            self.post_dpress = max_pressure

            if self.pre_dpress is not None and self.post_dpress is not None and self.pre_dflow is not None:
                dp = self.post_dpress - self.pre_dpress
                dq = -self.pre_dflow / 60.0
                if abs(dq) > 1e-4:
                    self.lung_resistance = dp / dq
                    print(f"DEBUG: Calculated Lung Resistance: {self.lung_resistance:.2f}")
                else:
                    self.lung_resistance = None
                    print("DEBUG: Lung Resistance not calculated (negligible flow).")
            else:
                self.lung_resistance = None
                print("DEBUG: Lung Resistance not calculated (missing pressure or flow data).")

            self.pre_dpress_label.setText(f"{self.pre_dpress:.2f}")
            self.post_dpress_label.setText(f"{self.post_dpress:.2f}")
            self.lung_resistance_label.setText(f"{self.lung_resistance:.2f}" if self.lung_resistance is not None else "N/A")

            print(f"DEBUG: Pressure collection ended. PreDPress: {self.pre_dpress:.2f}, "
                  f"PostDPress: {self.post_dpress:.2f}, Lung Resistance: {self.lung_resistance:.2f}")
        else:
            self.pre_dpress_label.setText("N/A")
            self.post_dpress_label.setText("N/A")
            self.lung_resistance_label.setText("N/A")
            print("DEBUG: No pressure data collected during the 1-second window.")

    def print_current_values(self):
        res = data_buffers["reservoir_data"][-1] if data_buffers["reservoir_data"] else 'N/A'
        sys = data_buffers["system_data"][-1] if data_buffers["system_data"] else 'N/A'
        lung = data_buffers["lung_data"][-1] if data_buffers["lung_data"] else 'N/A'
        oxy = data_buffers["oxygen_data"][-1] if data_buffers["oxygen_data"] else 'N/A'
        inh = data_buffers["inh_volume_data"][-1] if data_buffers["inh_volume_data"] else 'N/A'
        exh = data_buffers["exh_volume_data"][-1] if data_buffers["exh_volume_data"] else 'N/A'
        flow = data_buffers["flow_data"][-1] if data_buffers["flow_data"] else 'N/A'
        valve_d = self.prev_valve_d_state if self.prev_valve_d_state else 'N/A'
        min_exh = self.min_exh_pressure if self.min_exh_pressure is not None else 'N/A'
        post_d = self.post_dpress if self.post_dpress is not None else 'N/A'
        pre_d = self.pre_dpress if self.pre_dpress is not None else 'N/A'
        pre_dflow = self.pre_dflow if self.pre_dflow is not None else "N/A"
        post_dflow = self.post_dflow if self.post_dflow is not None else "N/A"
        lung_res = self.lung_resistance if self.lung_resistance is not None else 'N/A'
        print(f"DEBUG: Current State - Res: {res}, Sys: {sys}, Lung: {lung}, Oxy: {oxy}, "
              f"Inh: {inh}, Exh: {exh}, Flow: {flow}, Valve D: {valve_d}, "
              f"Min Exh: {min_exh}, PostDPress: {post_d}, PreDPress: {pre_d}, LungRes: {lung_res}, "
              f"PreDFlow: {pre_dflow}, PostDFlow: {post_dflow}")

    def initialize_buffers(self):
        for _ in range(100):
            data_buffers["time"].append(len(data_buffers["time"]))
            data_buffers["reservoir_data"].append(0.0)
            data_buffers["system_data"].append(0.0)
            data_buffers["lung_data"].append(0.0)
            data_buffers["oxygen_data"].append(0.0)
            data_buffers["inh_volume_data"].append(0.0)
            data_buffers["exh_volume_data"].append(0.0)
            data_buffers["flow_data"].append(0.0)

    def toggle_mode(self):
        self.is_manual_mode = not getattr(self, 'is_manual_mode', False)
        if self.is_manual_mode:
            self.mode_button.setText("Switch to Automatic Mode")
            self.mode_label.setText("Current Mode: Manual")
            self.auto_manual_stack.setCurrentIndex(1)
            self.send_command("MODE_MANUAL")
        else:
            self.mode_button.setText("Switch to Manual Mode")
            self.mode_label.setText("Current Mode: Automatic")
            self.auto_manual_stack.setCurrentIndex(0)
            self.send_command("MODE_AUTO")

    def send_command(self, command):
        command_queue.put(command)

    def create_valve_controls(self):
        valve_group = QGroupBox("Valve Controls")
        valve_group.setStyleSheet(GROUPBOX_STYLE)
        valve_layout = QGridLayout()
        valve_layout.setSpacing(10)
        
        open_label = QLabel("Open")
        close_label = QLabel("Close")
        open_label.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        close_label.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        open_label.setStyleSheet("font-weight: bold; color: #3498db;")
        close_label.setStyleSheet("font-weight: bold; color: #3498db;")
        
        valve_layout.addWidget(open_label, 0, 0)
        valve_layout.addWidget(close_label, 0, 1)
        
        valves = ["A", "B", "C", "D"]
        for i, valve in enumerate(valves):
            open_button = QPushButton(f"Valve {valve}")
            close_button = QPushButton(f"Valve {valve}")
            open_button.setStyleSheet(BUTTON_STYLE)
            close_button.setStyleSheet(BUTTON_STYLE)
            open_button.setFixedHeight(40)
            close_button.setFixedHeight(40)
            open_button.clicked.connect(lambda _, v=valve: self.send_command(f"VALVE_{v}_OPEN"))
            close_button.clicked.connect(lambda _, v=valve: self.send_command(f"VALVE_{v}_CLOSE"))
            valve_layout.addWidget(open_button, i + 1, 0)
            valve_layout.addWidget(close_button, i + 1, 1)
        
        valve_group.setLayout(valve_layout)
        return valve_group

    def create_plots(self, layout, plot_type="main"):
        if not hasattr(self, 'plot_widgets'):
            self.plot_widgets = {}
        if not hasattr(self, 'plot_curves'):
            self.plot_curves = {}
            
        pg.setConfigOption('background', '#1e2330')
        pg.setConfigOption('foreground', 'w')
        
        if plot_type == "main":
            self.pressure_plot_widget = PlotWidget(title="System & Lung Pressure")
            self.pressure_plot_widget.setLabel('left', 'Pressure', units='cmH₂O')
            self.pressure_plot_widget.setLabel('bottom', 'Time', units='s')
            self.pressure_plot_widget.showGrid(x=True, y=True, alpha=0.3)
            layout.addWidget(self.pressure_plot_widget, 0, 0, 1, 2)
            
            self.system_curve = self.pressure_plot_widget.plot(pen=pg.mkPen(color="r", width=3), name="System")
            self.lung_curve = self.pressure_plot_widget.plot(pen=pg.mkPen(color="#FFA500", width=3), name="Lung")
            self.system_curve.setDownsampling(ds=10, auto=True, method='peak')
            self.lung_curve.setDownsampling(ds=10, auto=True, method='peak')
            self.pressure_plot_widget.setClipToView(True)
            self.pressure_plot_widget.setAntialiasing(False)
            
            flow_widget = PlotWidget(title="Flow Rate")
            flow_widget.setLabel('left', 'Flow', units='L/min')
            flow_widget.setLabel('bottom', 'Time', units='s')
            flow_widget.showGrid(x=True, y=True, alpha=0.3)
            layout.addWidget(flow_widget, 1, 0, 1, 2)
            self.plot_widgets["flow_data"] = flow_widget
            self.plot_curves["flow_data"] = flow_widget.plot(pen=pg.mkPen(color="#FFFF00", width=3))
            
        elif plot_type == "advanced":
            advanced_plot_info = [
                ("Reservoir Pressure", "reservoir_data", "#3498db"),
                ("Oxygen Level", "oxygen_data", "#2ecc71"),
                ("Inhalation Volume", "inh_volume_data", "#00FFFF"),
                ("Exhalation Volume", "exh_volume_data", "#FF00FF"),
            ]
            for i, (title, tag, color) in enumerate(advanced_plot_info):
                plot_widget = PlotWidget(title=title)
                plot_widget.showGrid(x=True, y=True, alpha=0.3)
                layout.addWidget(plot_widget, i // 2, i % 2)
                self.plot_widgets[tag] = plot_widget
                self.plot_curves[tag] = plot_widget.plot(pen=pg.mkPen(color=color, width=3))

    def update_plots(self):
        if not data_buffers["time"]:
            return
        times_list = list(data_buffers["time"])[-500:]
        sys_list = list(data_buffers["system_data"])[-500:]
        lung_list = list(data_buffers["lung_data"])[-500:]
        self.system_curve.setData(times_list, sys_list)
        self.lung_curve.setData(times_list, lung_list)
        for tag in ["reservoir_data", "oxygen_data", "inh_volume_data", "exh_volume_data", "flow_data"]:
            if data_buffers[tag]:
                self.plot_curves[tag].setData(times_list, list(data_buffers[tag])[-500:])
        self.update_realtime_labels()
        self.check_alarms()
        self.maybe_resume_after_blockage()
        self.send_peep_control_command()

    def send_peep_control_command(self):
        """Monitor lung pressure and send command to close Valve D based on target PEEP and ep_eep."""
        if not data_buffers["lung_data"] or not data_buffers["flow_data"]:
            return

        current_lung_pressure = data_buffers["lung_data"][-1]
        current_flow = data_buffers["flow_data"][-1]
        target_peep = float(self.ppeep_input.text() or "0")
        current_time = time.time() * 1000  # in milliseconds

        # Log all relevant values for debugging
        print(f"DEBUG: Current Lung Pressure: {current_lung_pressure:.2f} cmH₂O, "
              f"Flow: {current_flow:.2f} L/min, Target PEEP: {target_peep:.2f} cmH₂O, "
              f"ep_eep: {self.ep_eep:.2f} cmH₂O, "
              f"Time since last close: {current_time - self.last_valve_d_close_time:.0f} ms")

        if (abs(current_flow) <= self.flow_zero_threshold and
            (target_peep - self.ep_eep) <= current_lung_pressure <= (target_peep + self.ep_eep) and
            current_time - self.last_valve_d_close_time >= self.hardware_lag):
            delta = current_time - self.last_valve_d_close_time
            self.send_command("VALVE_D_CLOSE")
            self.last_valve_d_close_time = current_time
            print(
                f"DEBUG: Condition met - Closing Valve D. "
                f"Lung Pressure: {current_lung_pressure:.2f} cmH₂O within "
                f"[{(target_peep - self.ep_eep):.2f}, {(target_peep + self.ep_eep):.2f}] cmH₂O, "
                f"Flow: {current_flow:.2f} L/min near zero, "
                f"Time since last close: {delta:.0f} ms >= {self.hardware_lag} ms"
            )
            latest_output_queue.put(f"Valve D Closed at Lung Pressure: {current_lung_pressure:.2f}, ep_eep: {self.ep_eep:.2f}")
        else:
            # Log why the condition was not met
            reasons = []
            if abs(current_flow) > self.flow_zero_threshold:
                reasons.append(f"Flow: {current_flow:.2f} L/min not near zero (>{self.flow_zero_threshold} L/min)")
            if current_lung_pressure < (target_peep - self.ep_eep):
                reasons.append(f"Lung Pressure {current_lung_pressure:.2f} < lower bound {(target_peep - self.ep_eep):.2f}")
            elif current_lung_pressure > (target_peep + self.ep_eep):
                reasons.append(f"Lung Pressure {current_lung_pressure:.2f} > upper bound {(target_peep + self.ep_eep):.2f}")
            if current_time - self.last_valve_d_close_time < self.hardware_lag:
                reasons.append(f"Time since last close: {current_time - self.last_valve_d_close_time:.0f} ms < {self.hardware_lag} ms")
            print(f"DEBUG: Condition not met - Valve D not closed. Reasons: {', '.join(reasons)}")

    def create_auto_control_panel(self):
        auto_control_group = QGroupBox("Automatic Control Panel")
        auto_control_group.setStyleSheet(GROUPBOX_STYLE)
        auto_control_group.setFixedWidth(260)
        
        main_layout = QVBoxLayout()
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(15, 15, 15, 15)
        
        self.tidal_volume_input = QLineEdit("400")
        self.ppeep_input = QLineEdit("15.0")
        self.resp_rate_input = QLineEdit("20")
        self.fio2_input = QLineEdit("95")
        self.ie_ratio_input = QLineEdit("1:2")
        
        input_style = "QLineEdit { background-color: #1e2330; color: white; border: 2px solid #3498db; border-radius: 6px; padding: 6px; font-size: 14pt; margin: 3px; } QLineEdit:focus { border-color: #2ecc71; background-color: #2c3e50; }"
        for input_field in [self.tidal_volume_input, self.ppeep_input, self.resp_rate_input, self.fio2_input, self.ie_ratio_input]:
            input_field.setStyleSheet(input_style)
            input_field.setFixedHeight(40)
            input_field.setFixedWidth(100)
        
        label_style = "QLabel { color: #2c3e50; font-weight: bold; font-size: 12pt; margin-bottom: 3px; }"
        tidal_label = QLabel("Target Tidal Volume (mL):")
        ppeep_label = QLabel("PEEP (cmH₂O):")
        resp_rate_label = QLabel("Respiratory Rate (BPM):")
        fio2_label = QLabel("FiO₂ (%):")
        ie_ratio_label = QLabel("I:E Ratio:")
        
        for label in [tidal_label, ppeep_label, resp_rate_label, fio2_label, ie_ratio_label]:
            label.setStyleSheet(label_style)
        
        input_groups = [
            (tidal_label, self.tidal_volume_input),
            (ppeep_label, self.ppeep_input),
            (resp_rate_label, self.resp_rate_input),
            (fio2_label, self.fio2_input),
            (ie_ratio_label, self.ie_ratio_input)
        ]
        
        for label, input_field in input_groups:
            group_layout = QVBoxLayout()
            group_layout.addWidget(label)
            group_layout.addWidget(input_field)
            group_layout.setSpacing(2)
            main_layout.addLayout(group_layout)
        
        apply_button = QPushButton("Apply Settings")
        apply_button.setStyleSheet(BUTTON_STYLE)
        apply_button.clicked.connect(self.apply_auto_settings)
        apply_button.setFixedHeight(40)
        apply_button.setFixedWidth(160)
        
        button_container = QWidget()
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        button_layout.addWidget(apply_button)
        button_layout.addStretch()
        button_container.setLayout(button_layout)
        
        main_layout.addSpacing(5)
        main_layout.addWidget(button_container)
        
        auto_control_group.setLayout(main_layout)
        return auto_control_group

    def apply_auto_settings(self):
        self.stored_settings = {
            "TIDAL_VOLUME": self.tidal_volume_input.text(),
            "PPEEP": self.ppeep_input.text(),
            "RESP_RATE": self.resp_rate_input.text(),
            "FIO2": self.fio2_input.text(),
            "IE_RATIO": self.ie_ratio_input.text()
        }
        target_peep = float(self.ppeep_input.text() or "0")
        self.ep_eep = 0.1 * target_peep + 1.5
        print(f"Initial ep_eep set to {self.ep_eep:.2f} based on target PEEP {target_peep:.2f}")
        # Immediately apply settings to the controller so UI changes take effect without pressing Start
        for key, value in self.stored_settings.items():
            command_queue.put(f"{key}:{value}")
        latest_output_queue.put("Settings applied to controller.")
        self.last_user_setting_breath_count = self.breath_count

    def create_action_buttons(self):
        action_group = QGroupBox("Action Buttons")
        action_group.setStyleSheet(GROUPBOX_STYLE)
        button_layout = QHBoxLayout()
        button_layout.setSpacing(5)
        button_layout.setContentsMargins(10, 10, 10, 10)
        
        self.start_button = QPushButton("Start")
        self.confirm_exit_button = QPushButton("Confirm Exit")
        self.exit_button = QPushButton("Exit")
        self.confirm_stop_button = QPushButton("Confirm Stop")
        self.stop_button = QPushButton("Stop")
        
        for button in [self.start_button, self.confirm_exit_button, self.exit_button, 
                       self.confirm_stop_button, self.stop_button]:
            button.setStyleSheet(BUTTON_STYLE)
            button.setFixedHeight(35)
        
        self.start_button.setStyleSheet(BUTTON_STYLE.replace("#2c3e50", "#27ae60"))
        self.stop_button.setStyleSheet(BUTTON_STYLE.replace("#2c3e50", "#e74c3c"))
        self.confirm_stop_button.setStyleSheet(BUTTON_STYLE.replace("#2c3e50", "#f39c12"))
        
        self.start_button.clicked.connect(self.start_ventilator)
        self.confirm_exit_button.clicked.connect(self.confirm_exit)
        self.exit_button.clicked.connect(self.exit_application)
        self.confirm_stop_button.clicked.connect(self.confirm_stop)
        self.stop_button.clicked.connect(self.stop_ventilator)
        
        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.confirm_exit_button)
        button_layout.addWidget(self.exit_button)
        button_layout.addWidget(self.confirm_stop_button)
        button_layout.addWidget(self.stop_button)
        
        action_group.setLayout(button_layout)
        return action_group

    def start_ventilator(self):
        if not self.stored_settings:
            latest_output_queue.put("No settings applied. Please apply settings first.")
            return
        for key, value in self.stored_settings.items():
            command_queue.put(f"{key}:{value}")
        command_queue.put("START")
        latest_output_queue.put("Settings applied. Starting ventilator...")

    def confirm_exit(self):
        self.exit_confirmed = True
        latest_output_queue.put("Exit confirmed. Click 'Exit' to close.")

    def exit_application(self):
        if not self.exit_confirmed:
            latest_output_queue.put("Exit not confirmed. Click 'Confirm Exit' first.")
            return
        self.close()

    def update_latest_output(self):
        while not latest_output_queue.empty():
            line = latest_output_queue.get()
            self.latest_output_label.setText(f"Latest Output: {line}")

    def confirm_stop(self):
        self.stop_confirmed = True
        latest_output_queue.put("Stop confirmed. Click 'Stop' to halt.")

    def stop_ventilator(self):
        if not self.stop_confirmed:
            latest_output_queue.put("Stop not confirmed. Click 'Confirm Stop' first.")
            return
        command_queue.put("STOP")
        latest_output_queue.put("System stopped.")
        self.stop_confirmed = False

    def create_dashboard_page(self):
        dashboard_page = QWidget()
        main_layout = QVBoxLayout(dashboard_page)
        main_layout.setSpacing(5)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        header_layout = QHBoxLayout()
        header_layout.setSpacing(5)
        title_label = QLabel("JAMVENT")
        title_label.setStyleSheet("font-size: 20pt; font-weight: bold; color: #3498db;")
        header_layout.addWidget(title_label)
        header_layout.addStretch(1)
        main_layout.addLayout(header_layout)
        
        control_layout = QHBoxLayout()
        control_layout.setSpacing(5)
        button_layout = QHBoxLayout()
        button_layout.setSpacing(5)
        self.mode_button = QPushButton("Switch to Manual Mode")
        self.settings_button = QPushButton("Settings")
        self.advanced_button = QPushButton("Advanced")
        for button in [self.mode_button, self.settings_button, self.advanced_button]:
            button.setStyleSheet(BUTTON_STYLE)
            button.setFixedHeight(35)
            button_layout.addWidget(button)
        self.mode_button.clicked.connect(self.toggle_mode)
        self.settings_button.clicked.connect(self.show_settings_page)
        self.advanced_button.clicked.connect(self.show_advanced_page)
        control_layout.addLayout(button_layout)
        control_layout.addStretch(1)
        
        self.alarm_light = QLabel()
        self.alarm_light.setFixedSize(25, 25)
        self.alarm_light.setStyleSheet("background-color: green; border-radius: 12px;")
        control_layout.addWidget(self.alarm_light)
        self.alarm_label = QLabel("No Alarm")
        self.alarm_label.setStyleSheet("font-size: 12pt; font-weight: bold; color: green;")
        control_layout.addWidget(self.alarm_label)
        self.mode_label = QLabel("Current Mode: Automatic")
        self.mode_label.setStyleSheet("font-size: 12pt; font-weight: bold; color: #3498db;")
        control_layout.addWidget(self.mode_label)
        main_layout.addLayout(control_layout)
    
        middle_layout = QHBoxLayout()
        middle_layout.setSpacing(5)
        
        left_layout = QVBoxLayout()
        left_layout.setSpacing(5)
        self.auto_manual_stack = QStackedWidget()
        auto_page = QWidget()
        auto_layout = QVBoxLayout(auto_page)
        self.auto_control_group = self.create_auto_control_panel()
        auto_layout.addWidget(self.auto_control_group)
        self.auto_manual_stack.addWidget(auto_page)
        manual_page = QWidget()
        manual_layout = QVBoxLayout(manual_page)
        self.valve_controls_group = self.create_valve_controls()
        manual_layout.addWidget(self.valve_controls_group)
        self.auto_manual_stack.addWidget(manual_page)
        left_layout.addWidget(self.auto_manual_stack)
        left_widget = QWidget()
        left_widget.setLayout(left_layout)
        left_widget.setFixedWidth(300)
        middle_layout.addWidget(left_widget)
        
        plot_layout = QGridLayout()
        plot_layout.setSpacing(5)
        self.create_plots(plot_layout, plot_type="main")
        plots_widget = QWidget()
        plots_widget.setLayout(plot_layout)
        middle_layout.addWidget(plots_widget, 1)
        
        self.realtime_panel = self.create_realtime_panel()
        middle_layout.addWidget(self.realtime_panel)
        
        main_layout.addLayout(middle_layout)
        
        self.action_buttons = self.create_action_buttons()
        main_layout.addWidget(self.action_buttons)
        
        self.latest_output_label = QLabel("Latest Output:")
        self.latest_output_label.setStyleSheet("font-size: 12pt; color: #2c3e50;")
        main_layout.addWidget(self.latest_output_label)
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(20)
        
        self.output_timer = QTimer()
        self.output_timer.timeout.connect(self.update_latest_output)
        self.output_timer.start(200)
        
        return dashboard_page

    def create_realtime_panel(self):
        box = QGroupBox("Real-Time Values")
        box.setStyleSheet("QGroupBox { font-weight: bold; }")
        box.setFixedWidth(300)
        layout = QVBoxLayout(box)
        layout.setSpacing(10)
        layout.setContentsMargins(15, 20, 15, 20)
        
        self.res_pressure_label = QLabel("0.00")
        self.sys_pressure_label = QLabel("0.00")
        self.lung_pressure_label = QLabel("0.00")
        self.oxy_label = QLabel("0.00")
        self.inh_label = QLabel("0.00")
        self.exh_label = QLabel("0.00")
        self.flow_label = QLabel("0.00")

        big_style = "font-size: 28pt; font-weight: bold; color: white; background-color: #1e2330; border-radius: 5px; padding: 5px;"
        param_style = "font-size: 14pt; font-weight: bold; color: #3498db;"
        
        for lbl in (self.res_pressure_label, self.sys_pressure_label, self.lung_pressure_label,
                    self.oxy_label, self.inh_label, self.exh_label, self.flow_label):
            lbl.setStyleSheet(big_style)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)

        res_label = QLabel("Reservoir Pressure:")
        sys_label = QLabel("System Pressure:")
        lung_label = QLabel("Lung Pressure:")
        oxy_label = QLabel("Oxygen Level:")
        inh_label = QLabel("Inhalation Volume:")
        exh_label = QLabel("Exhalation Volume:")
        flow_label = QLabel("Flow Rate:")
        
        for lbl in (res_label, sys_label, lung_label, oxy_label, inh_label, exh_label, flow_label):
            lbl.setStyleSheet(param_style)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)

        layout.addWidget(res_label)
        layout.addWidget(self.res_pressure_label)
        layout.addSpacing(5)
        
        layout.addWidget(sys_label)
        layout.addWidget(self.sys_pressure_label)
        layout.addSpacing(5)

        layout.addWidget(lung_label)
        layout.addWidget(self.lung_pressure_label)
        layout.addSpacing(5)
        
        layout.addWidget(oxy_label)
        layout.addWidget(self.oxy_label)
        layout.addSpacing(5)
        
        layout.addWidget(inh_label)
        layout.addWidget(self.inh_label)
        layout.addSpacing(5)
        
        layout.addWidget(exh_label)
        layout.addWidget(self.exh_label)
        layout.addSpacing(5)
        
        layout.addWidget(flow_label)
        layout.addWidget(self.flow_label)
        
        return box

    def update_realtime_labels(self):
        if len(data_buffers["reservoir_data"]) > 0:
            self.res_pressure_label.setText(f"{data_buffers['reservoir_data'][-1]:.2f}")
        if len(data_buffers["system_data"]) > 0:
            self.sys_pressure_label.setText(f"{data_buffers['system_data'][-1]:.2f}")
        if len(data_buffers["lung_data"]) > 0:
            self.lung_pressure_label.setText(f"{data_buffers['lung_data'][-1]:.2f}")
        if len(data_buffers["oxygen_data"]) > 0:
            self.oxy_label.setText(f"{data_buffers['oxygen_data'][-1]:.2f}")
        if len(data_buffers["inh_volume_data"]) > 0:
            self.inh_label.setText(f"{data_buffers['inh_volume_data'][-1]:.2f}")
        if len(data_buffers["exh_volume_data"]) > 0:
            self.exh_label.setText(f"{data_buffers['exh_volume_data'][-1]:.2f}")
        if len(data_buffers["flow_data"]) > 0:
            self.flow_label.setText(f"{data_buffers['flow_data'][-1]:.2f}")
        if self.pre_dpress is not None:
            self.pre_dpress_label.setText(f"{self.pre_dpress:.2f}")
        if self.post_dpress is not None:
            self.post_dpress_label.setText(f"{self.post_dpress:.2f}")
        if self.lung_resistance is not None:
            self.lung_resistance_label.setText(f"{self.lung_resistance:.2f}")
        if self.min_exh_pressure is not None:
            self.min_exh_pressure_label.setText(f"{self.min_exh_pressure:.2f}")

    def create_settings_page(self):
        settings_page = QWidget()
        layout = QVBoxLayout(settings_page)
        layout.setSpacing(15)
        layout.setContentsMargins(20, 20, 20, 20)
        
        title = QLabel("Alarm Settings")
        title.setStyleSheet("font-size: 24pt; font-weight: bold; color: #3498db;")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)
        
        alarms_group = QGroupBox("Alarm Thresholds")
        alarms_group.setStyleSheet(GROUPBOX_STYLE)
        alarms_layout = QFormLayout()
        alarms_layout.setSpacing(15)
        alarms_layout.setContentsMargins(15, 20, 15, 20)
        
        input_style = "QLineEdit { background-color: #1e2330; color: white; border: 1px solid #3498db; border-radius: 4px; padding: 8px; font-size: 14pt; }"
        label_style = "QLabel { color: #2c3e50; font-weight: bold; font-size: 14pt; }"
        
        self.high_pressure_input = QLineEdit(str(self.high_pressure_alarm_threshold))
        self.high_pressure_input.setValidator(QIntValidator(30, 70))
        self.high_pressure_input.setStyleSheet(input_style)
        self.high_pressure_input.setFixedHeight(40)
        
        self.low_pressure_input = QLineEdit(str(self.low_pressure_alarm_threshold))
        self.low_pressure_input.setValidator(QIntValidator(-10, -2))
        self.low_pressure_input.setStyleSheet(input_style)
        self.low_pressure_input.setFixedHeight(40)
        
        self.blockage_input = QLineEdit(str(self.blockage_alarm_threshold))
        self.blockage_input.setValidator(QIntValidator(2, 10))
        self.blockage_input.setStyleSheet(input_style)
        self.blockage_input.setFixedHeight(40)
        
        self.hypervent_input = QLineEdit(str(self.hypervent_threshold))
        self.hypervent_input.setValidator(QIntValidator(15, 50))
        self.hypervent_input.setStyleSheet(input_style)
        self.hypervent_input.setFixedHeight(40)
        
        self.hypovent_input = QLineEdit(str(self.hypovent_threshold))
        self.hypovent_input.setValidator(QIntValidator(-50, -15))
        self.hypovent_input.setStyleSheet(input_style)
        self.hypovent_input.setFixedHeight(40)
        
        high_pressure_label = QLabel("High Pressure Alarm (cmH₂O):")
        high_pressure_label.setStyleSheet(label_style)
        low_pressure_label = QLabel("Low Pressure Alarm (cmH₂O):")
        low_pressure_label.setStyleSheet(label_style)
        blockage_label = QLabel("Blockage Alarm (cmH₂O over PEEP):")
        blockage_label.setStyleSheet(label_style)
        hypervent_label = QLabel("Hypervent Alarm (% above Vt):")
        hypervent_label.setStyleSheet(label_style)
        hypovent_label = QLabel("Hypovent Alarm (% below Vt):")
        hypovent_label.setStyleSheet(label_style)
        
        alarms_layout.addRow(high_pressure_label, self.high_pressure_input)
        alarms_layout.addRow(low_pressure_label, self.low_pressure_input)
        alarms_layout.addRow(blockage_label, self.blockage_input)
        alarms_layout.addRow(hypervent_label, self.hypervent_input)
        alarms_layout.addRow(hypovent_label, self.hypovent_input)
        
        alarms_group.setLayout(alarms_layout)
        layout.addWidget(alarms_group)
        
        button_container = QWidget()
        button_layout = QHBoxLayout(button_container)
        button_layout.setSpacing(20)
        
        apply_button = QPushButton("Apply Alarms")
        apply_button.setStyleSheet(BUTTON_STYLE.replace("#2c3e50", "#27ae60"))
        apply_button.clicked.connect(self.apply_alarm_settings)
        apply_button.setFixedHeight(50)
        apply_button.setFixedWidth(200)
        button_layout.addWidget(apply_button)
        
        back_button = QPushButton("Back to Dashboard")
        back_button.setStyleSheet(BUTTON_STYLE)
        back_button.clicked.connect(self.show_dashboard_page)
        back_button.setFixedHeight(50)
        back_button.setFixedWidth(200)
        button_layout.addWidget(back_button)
        
        layout.addWidget(button_container)
        layout.addStretch(1)
        
        return settings_page

    def apply_alarm_settings(self):
        try:
            self.high_pressure_alarm_threshold = int(self.high_pressure_input.text())
            self.low_pressure_alarm_threshold = int(self.low_pressure_input.text())
            self.blockage_alarm_threshold = int(self.blockage_input.text())
            self.hypervent_threshold = int(self.hypervent_input.text())
            self.hypovent_threshold = int(self.hypovent_input.text())
            command_queue.put(f"HIGH_PRESSURE_ALARM:{self.high_pressure_alarm_threshold}")
            command_queue.put(f"LOW_PRESSURE_ALARM:{self.low_pressure_alarm_threshold}")
            command_queue.put(f"BLOCKAGE_ALARM:{self.blockage_alarm_threshold}")
            command_queue.put(f"HYPERVENTILATION_ALARM:{self.hypervent_threshold}")
            command_queue.put(f"HYPOVENTILATION_ALARM:{self.hypovent_threshold}")
            latest_output_queue.put("Alarm thresholds applied successfully!")
        except ValueError:
            latest_output_queue.put("Error: Invalid alarm input(s).")

    def create_advanced_page(self):
        advanced_page = QWidget()
        layout = QVBoxLayout(advanced_page)
        layout.setSpacing(15)
        layout.setContentsMargins(20, 20, 20, 20)
        
        title = QLabel("Advanced Monitoring")
        title.setStyleSheet("font-size: 24pt; font-weight: bold; color: #3498db;")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)
        
        plots_group = QGroupBox("Additional Plots")
        plots_group.setStyleSheet(GROUPBOX_STYLE)
        plots_layout = QGridLayout()
        plots_layout.setSpacing(15)
        self.create_plots(plots_layout, plot_type="advanced")
        plots_group.setLayout(plots_layout)
        layout.addWidget(plots_group)
        
        tech_group = QGroupBox("Technical Parameters")
        tech_group.setStyleSheet(GROUPBOX_STYLE)
        tech_layout = QGridLayout()
        tech_layout.setSpacing(10)
        
        param_style = "font-size: 12pt; font-weight: bold; color: #2c3e50;"
        value_style = "font-size: 14pt; font-weight: bold; color: white; background-color: #1e2330; border-radius: 4px; padding: 5px;"
        
        params = [
            ("Pre-D Pressure:", self.pre_dpress_label, 0, 0),
            ("Post-D Pressure:", self.post_dpress_label, 0, 1),
            ("Pre-D Flow:", self.pre_dflow_label, 1, 0),
            ("Post-D Flow:", self.post_dflow_label, 1, 1),
            ("Lung Resistance:", self.lung_resistance_label, 2, 0),
            ("Min Exh Pressure:", self.min_exh_pressure_label, 2, 1)
        ]
        
        for label_text, value_label, row, col in params:
            param_label = QLabel(label_text)
            param_label.setStyleSheet(param_style)
            value_label.setStyleSheet(value_style)
            value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            value_label.setFixedHeight(40)
            tech_layout.addWidget(param_label, row, col*2)
            tech_layout.addWidget(value_label, row, col*2+1)
        
        tech_group.setLayout(tech_layout)
        layout.addWidget(tech_group)
        
        back_button = QPushButton("Back to Dashboard")
        back_button.setStyleSheet(BUTTON_STYLE)
        back_button.clicked.connect(self.show_dashboard_page)
        back_button.setFixedHeight(50)
        back_button.setFixedWidth(200)
        
        button_container = QWidget()
        button_layout = QHBoxLayout(button_container)
        button_layout.addStretch(1)
        button_layout.addWidget(back_button)
        button_layout.addStretch(1)
        
        layout.addWidget(button_container)
        layout.addStretch(1)
        
        return advanced_page

    def show_dashboard_page(self):
        self.central_widget.setCurrentWidget(self.dashboard_page)

    def show_settings_page(self):
        self.central_widget.setCurrentWidget(self.settings_page)
        
    def show_advanced_page(self):
        self.central_widget.setCurrentWidget(self.advanced_page)

    def check_alarms(self):
        if not data_buffers["system_data"]:
            return
        system_pressure = data_buffers["system_data"][-1]
        if system_pressure > self.high_pressure_alarm_threshold:
            self.handle_high_pressure_alarm(system_pressure)
        elif system_pressure < self.low_pressure_alarm_threshold:
            self.handle_low_pressure_alarm(system_pressure)
        else:
            self.clear_alarm()

    def trigger_alarm(self, msg):
        self.alarm_light.setStyleSheet("background-color: #e74c3c; border-radius: 15px;")
        self.alarm_label.setText(msg)
        self.alarm_label.setStyleSheet("font-size: 14pt; font-weight: bold; color: #e74c3c;")
        self.alarm_active = True

    def clear_alarm(self):
        self.alarm_light.setStyleSheet("background-color: #27ae60; border-radius: 15px;")
        self.alarm_label.setText("No Alarm")
        self.alarm_label.setStyleSheet("font-size: 14pt; font-weight: bold; color: #27ae60;")
        self.alarm_active = False

    def handle_high_pressure_alarm(self, pressure):
        msg = f"High Pressure: {pressure:.1f} cmH₂O"
        self.trigger_alarm(msg)
        self.send_command("VALVE_C_CLOSE")
        self.send_command("VALVE_D_OPEN")
        latest_output_queue.put(f"{msg} - Closed Valve C, Opened Valve D")

    def handle_low_pressure_alarm(self, pressure):
        msg = f"Low Pressure: {pressure:.1f} cmH₂O"
        self.trigger_alarm(msg)
        self.send_command("VALVE_C_OPEN")
        self.send_command("VALVE_D_CLOSE")
        latest_output_queue.put(f"{msg} - Opened Valve C, Closed Valve D")

    def detect_end_of_breath(self):
        if len(data_buffers["exh_volume_data"]) < 2:
            return
        prev_exh = data_buffers["exh_volume_data"][-2]
        curr_exh = data_buffers["exh_volume_data"][-1]
        if prev_exh > 1.0 and curr_exh < 0.5:
            self.breath_count += 1
            if data_buffers["inh_volume_data"]:
                last_inh = data_buffers["inh_volume_data"][-1]
                self.recent_inh_volumes.append(last_inh)
                if len(self.recent_inh_volumes) > 3:
                    self.recent_inh_volumes.pop(0)
            peep_measured = self.estimate_peep_from_data()
            
            target_peep = float(self.ppeep_input.text() or "0")
            peep_error = target_peep - peep_measured
            if abs(peep_error) > 0.01:
                adjustment = self.delta_ep_eep * (peep_error / abs(peep_error))
            else:
                adjustment = 0
            self.ep_eep = self.ep_eep + adjustment
            self.ep_eep = max(0.0, min(5.0, self.ep_eep))
            self.last_peep_measured = peep_measured
            
            print(f"DEBUG: PEEP Measured: {peep_measured:.2f}, Target PEEP: {target_peep:.2f}, "
                  f"PEEP Error: {peep_error:.2f}, Updated ep_eep: {self.ep_eep:.2f}")
            latest_output_queue.put(f"PEEP Measured: {peep_measured:.2f}, Error: {peep_error:.2f}, ep_eep: {self.ep_eep:.2f}")
            
            self.check_blockage_alarm(peep_measured)
            self.check_hypervent_alarm()
            self.check_hypovent_alarm()

    def estimate_peep_from_data(self):
        return min(data_buffers["system_data"][-10:]) if data_buffers["system_data"] else 0

    def check_blockage_alarm(self, actual_peep):
        if self.breath_count - self.last_user_setting_breath_count < 3:
            return
        desired_peep = float(self.ppeep_input.text() or "0")
        if actual_peep > (desired_peep + self.blockage_alarm_threshold):
            self.handle_blockage_alarm(actual_peep, desired_peep)

    def handle_blockage_alarm(self, actual_peep, desired_peep):
        msg = f"BLOCKAGE ALARM: PEEP {actual_peep:.1f} > {desired_peep:.1f}+{self.blockage_alarm_threshold}"
        self.trigger_alarm(msg)
        self.send_command("STOP")
        latest_output_queue.put(f"{msg} - Ventilation paused")

    def maybe_resume_after_blockage(self):
        if not self.alarm_active or not data_buffers["system_data"]:
            return
        current_pressure = data_buffers["system_data"][-1]
        desired_peep = float(self.ppeep_input.text() or "0")
        if current_pressure <= desired_peep:
            self.clear_alarm()
            latest_output_queue.put("Blockage resolved. Resuming normal ventilation")

    def check_hypervent_alarm(self):
        if (self.breath_count - self.last_user_setting_breath_count) < 3 or len(self.recent_inh_volumes) < 3:
            return
        avg_inh = sum(self.recent_inh_volumes) / 3.0
        desired_tidal = float(self.tidal_volume_input.text() or "400")
        ratio = (avg_inh / desired_tidal) * 100.0
        if ratio > (100 + self.hypervent_threshold):
            self.handle_hypervent_alarm(ratio)

    def handle_hypervent_alarm(self, ratio):
        msg = f"Hyperventilation Alarm: InhVol is {ratio:.1f}% of set Vt"
        self.trigger_alarm(msg)
        latest_output_queue.put(msg)

    def check_hypovent_alarm(self):
        if (self.breath_count - self.last_user_setting_breath_count) < 3 or len(self.recent_inh_volumes) < 3:
            return
        avg_inh = sum(self.recent_inh_volumes) / 3.0
        desired_tidal = float(self.tidal_volume_input.text() or "400")
        ratio = (avg_inh / desired_tidal) * 100.0
        limit = 100 + self.hypovent_threshold
        if ratio < limit:
            self.handle_hypovent_alarm(ratio, limit)

    def handle_hypovent_alarm(self, ratio, limit):
        msg = f"Hypoventilation Alarm: InhVol is {ratio:.1f}% (Limit: {limit:.1f}%)"
        self.trigger_alarm(msg)
        latest_output_queue.put(msg)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    dashboard = VentilatorDashboard()
    if ser:
        threading.Thread(target=read_serial, daemon=True).start()
        threading.Thread(target=process_commands, daemon=True).start()
    dashboard.show()
    sys.exit(app.exec())