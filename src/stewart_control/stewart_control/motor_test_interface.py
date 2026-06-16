#!/usr/bin/env python3

import sys
import time

from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QHeaderView,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)
import serial

from stewart_control.actuator_calibration import ActuatorCalibration
from stewart_control.config_loader import get_config, get_config_path, save_config
from stewart_control.runtime_state import RuntimeStateManager


TICKS_PER_TURN = 960.0
PULLEY_DIAMETER_CM = 2.0
PULLEY_CIRCUMFERENCE_CM = 3.1416 * PULLEY_DIAMETER_CM

BG_DARK = "#171922"
BG_CARD = "#232635"
BG_INPUT = "#10131b"
BG_PANEL = "#1d2130"
ACCENT = "#1976d2"
ACCENT_HOVER = "#2d8ae0"
SUCCESS = "#2e9b66"
SUCCESS_HOVER = "#39b97b"
DANGER = "#c84b53"
DANGER_HOVER = "#dd6168"
TEXT_PRIMARY = "#f3f1e8"
TEXT_SECONDARY = "#b8b4a8"
BORDER = "#35394b"


def make_button(text, color, hover):
    btn = QPushButton(text)
    btn.setCursor(Qt.PointingHandCursor)
    btn.setStyleSheet(
        f"""
        QPushButton {{
            background-color: {color};
            color: white;
            border: none;
            border-radius: 8px;
            padding: 8px 14px;
            min-height: 34px;
            font-size: 12px;
            font-weight: 700;
        }}
        QPushButton:hover {{
            background-color: {hover};
        }}
        QPushButton:disabled {{
            background-color: #4b5064;
            color: #d0d4df;
        }}
        """
    )
    return btn


def card(title):
    box = QGroupBox(title)
    box.setStyleSheet(
        f"""
        QGroupBox {{
            background-color: {BG_CARD};
            color: {TEXT_PRIMARY};
            border: 1px solid {BORDER};
            border-radius: 10px;
            margin-top: 12px;
            font-size: 12px;
            font-weight: 700;
            padding: 12px 10px 10px 10px;
        }}
        QGroupBox::title {{
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 4px;
        }}
        """
    )
    return box


class MotorTestInterface(QWidget):
    FREE_TARGET_MIN_CM = -9999.0
    FREE_TARGET_MAX_CM = 9999.0
    JOG_STEP_MIN_CM = 0.1
    JOG_STEP_MAX_CM = 5.0
    JOG_STEP_DEFAULT_CM = 0.1
    HOMING_SPAN_CM = 10.0

    def __init__(self):
        super().__init__()
        self.cfg = get_config()
        self.config_path = get_config_path()
        self.calibration = ActuatorCalibration.from_config(self.cfg["actuators"])
        self.runtime_state = RuntimeStateManager()
        self.ser = None
        self.last_feedback = [0.0] * 6
        self.last_command = [0.0] * 6
        self.session_min = [None] * 6
        self.session_max = [None] * 6
        self.selected_motor_index = 0
        self.homing_min = [None] * 6
        self.homing_max = [None] * 6
        self.feedback_ready = False

        self.setWindowTitle("Manual Homing")
        self.resize(980, 700)
        self.setMinimumSize(520, 520)
        self.setStyleSheet(
            f"""
            QWidget {{
                background-color: {BG_DARK};
                color: {TEXT_PRIMARY};
                font-family: 'Segoe UI', 'Helvetica Neue', Arial, sans-serif;
                font-size: 12px;
            }}
            QLabel {{
                color: {TEXT_PRIMARY};
            }}
            QComboBox, QDoubleSpinBox {{
                background-color: {BG_INPUT};
                color: {TEXT_PRIMARY};
                border: 1px solid {BORDER};
                border-radius: 6px;
                padding: 6px 8px;
            }}
            QComboBox:focus, QDoubleSpinBox:focus {{
                border: 1px solid {ACCENT};
            }}
            """
        )

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        root.addWidget(scroll)

        content = QWidget()
        scroll.setWidget(content)

        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(12)

        hero = QFrame()
        hero.setStyleSheet(
            f"""
            QFrame {{
                background-color: {BG_PANEL};
                border: 1px solid {BORDER};
                border-radius: 14px;
            }}
            """
        )
        hero_layout = QVBoxLayout(hero)
        hero_layout.setContentsMargins(18, 18, 18, 18)
        hero_layout.setSpacing(4)

        title = QLabel("Manual Homing")
        title.setStyleSheet("font-size: 24px; font-weight: 800;")
        subtitle = QLabel("Home each actuator and apply the calibration when all six are saved.")
        subtitle.setWordWrap(True)
        subtitle.setStyleSheet(f"color: {TEXT_SECONDARY}; font-size: 12px;")
        hero_layout.addWidget(title)
        hero_layout.addWidget(subtitle)
        content_layout.addWidget(hero)

        notification_bar = QFrame()
        notification_bar.setStyleSheet(
            f"""
            QFrame {{
                background-color: {BG_PANEL};
                border: 1px solid {BORDER};
                border-radius: 10px;
            }}
            """
        )
        notification_layout = QHBoxLayout(notification_bar)
        notification_layout.setContentsMargins(14, 10, 14, 10)
        notification_layout.setSpacing(10)
        notification_tag = QLabel("Notification")
        notification_tag.setStyleSheet(
            f"color: {ACCENT}; font-size: 11px; font-weight: 800; text-transform: uppercase;"
        )
        self.notification_label = QLabel("Waiting for controller connection.")
        self.notification_label.setWordWrap(True)
        self.notification_label.setStyleSheet("font-size: 13px; font-weight: 700;")
        notification_layout.addWidget(notification_tag)
        notification_layout.addWidget(self.notification_label, 1)
        content_layout.addWidget(notification_bar)

        top_layout = QHBoxLayout()
        top_layout.setSpacing(12)
        content_layout.addLayout(top_layout)

        connection_box = card("Connection")
        connection_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        connection_form = QFormLayout(connection_box)
        connection_form.setContentsMargins(12, 18, 12, 12)
        connection_form.setSpacing(10)
        connection_form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)

        self.port_label = QLabel(self.cfg["serial"]["port"])
        self.port_label.setStyleSheet("font-weight: 700;")
        self.port_label.setWordWrap(True)
        self.baud_label = QLabel(str(self.cfg["serial"]["baudrate"]))
        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet(f"color: {DANGER}; font-weight: 700;")

        self.connect_btn = make_button("Connect", SUCCESS, SUCCESS_HOVER)
        self.connect_btn.clicked.connect(self.connect_serial)
        self.disconnect_btn = make_button("Disconnect", DANGER, DANGER_HOVER)
        self.disconnect_btn.clicked.connect(self.disconnect_serial)
        self.disconnect_btn.setEnabled(False)

        buttons_row = QHBoxLayout()
        buttons_row.addWidget(self.connect_btn)
        buttons_row.addWidget(self.disconnect_btn)

        connection_form.addRow("Port", self.port_label)
        connection_form.addRow("Baudrate", self.baud_label)
        connection_form.addRow("Status", self.status_label)
        connection_form.addRow(buttons_row)
        top_layout.addWidget(connection_box, 1)

        command_box = card("Selected Motor Control")
        command_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        command_form = QFormLayout(command_box)
        command_form.setContentsMargins(12, 18, 12, 12)
        command_form.setSpacing(10)
        command_form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)

        self.motor_selector = QComboBox()
        self.motor_selector.addItems([f"Motor {i}" for i in range(1, 7)])
        self.motor_selector.currentIndexChanged.connect(self.on_motor_changed)

        self.target_spin = QDoubleSpinBox()
        self.target_spin.setRange(self.FREE_TARGET_MIN_CM, self.FREE_TARGET_MAX_CM)
        self.target_spin.setDecimals(2)
        self.target_spin.setSingleStep(0.1)
        self.target_spin.setValue(0.0)

        self.step_spin = QDoubleSpinBox()
        self.step_spin.setRange(self.JOG_STEP_MIN_CM, self.JOG_STEP_MAX_CM)
        self.step_spin.setDecimals(2)
        self.step_spin.setSingleStep(self.JOG_STEP_MIN_CM)
        self.step_spin.setValue(self.JOG_STEP_DEFAULT_CM)

        self.send_btn = make_button("Move To Target", ACCENT, ACCENT_HOVER)
        self.send_btn.clicked.connect(self.send_target)
        self.send_btn.setEnabled(False)

        self.jog_minus_btn = make_button("Jog Down", "#8b5e34", "#a06d3d")
        self.jog_minus_btn.clicked.connect(self.jog_negative)
        self.jog_minus_btn.setEnabled(False)
        self.jog_plus_btn = make_button("Jog Up", "#8b5e34", "#a06d3d")
        self.jog_plus_btn.clicked.connect(self.jog_positive)
        self.jog_plus_btn.setEnabled(False)
        self.zero_btn = make_button("Send Home Vector", "#5e6a82", "#6c7892")
        self.zero_btn.clicked.connect(self.send_selected_zero)
        self.zero_btn.setEnabled(False)
        self.reverse_travel_btn = make_button("Flip Travel +/-10", "#5e6a82", "#6c7892")
        self.reverse_travel_btn.clicked.connect(self.toggle_selected_travel_direction)
        self.reverse_travel_btn.setEnabled(False)

        jog_row = QHBoxLayout()
        jog_row.addWidget(self.jog_minus_btn)
        jog_row.addWidget(self.jog_plus_btn)

        command_form.addRow("Motor", self.motor_selector)
        command_form.addRow("Target (cm)", self.target_spin)
        command_form.addRow("Jog step (cm)", self.step_spin)
        command_form.addRow(self.send_btn)
        command_form.addRow(jog_row)
        command_form.addRow(self.zero_btn)
        command_form.addRow(self.reverse_travel_btn)
        top_layout.addWidget(command_box, 1)

        homing_box = card("Calibration Status")
        homing_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        homing_layout = QVBoxLayout(homing_box)
        homing_layout.setContentsMargins(12, 18, 12, 12)
        homing_layout.setSpacing(10)

        homing_help = QLabel(
            "Save the selected motor at home, then flip travel if its full stroke "
            "is home - 10 cm instead of home + 10 cm."
        )
        homing_help.setWordWrap(True)
        homing_help.setStyleSheet(f"color: {TEXT_SECONDARY};")
        homing_layout.addWidget(homing_help)

        self.progress_label = QLabel("0 of 6 motors saved")
        self.progress_label.setStyleSheet("font-size: 18px; font-weight: 800;")
        homing_layout.addWidget(self.progress_label)

        self.homing_table = QTableWidget(6, 4)
        self.homing_table.setHorizontalHeaderLabels(
            ["Motor", "Home Min", "Travel Max", "Status"]
        )
        self.homing_table.verticalHeader().setVisible(False)
        self.homing_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.homing_table.setSelectionMode(QTableWidget.NoSelection)
        self.homing_table.setFocusPolicy(Qt.NoFocus)
        self.homing_table.setMinimumHeight(220)
        self.homing_table.setStyleSheet(
            f"""
            QTableWidget {{
                background-color: {BG_INPUT};
                color: {TEXT_PRIMARY};
                border: 1px solid {BORDER};
                border-radius: 8px;
                gridline-color: {BORDER};
            }}
            QHeaderView::section {{
                background-color: {BG_CARD};
                color: {TEXT_PRIMARY};
                border: none;
                border-bottom: 1px solid {BORDER};
                padding: 6px;
                font-weight: 700;
            }}
            """
        )
        self.homing_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        homing_layout.addWidget(self.homing_table)

        homing_buttons = QHBoxLayout()
        self.capture_min_btn = make_button("Save Selected Home", "#7a5c2e", "#90703a")
        self.capture_min_btn.clicked.connect(self.capture_current_minimum)
        self.apply_homing_btn = make_button(
            "Apply Calibration", SUCCESS, SUCCESS_HOVER
        )
        self.apply_homing_btn.clicked.connect(self.apply_homing_calibration)
        self.apply_homing_btn.setEnabled(False)
        homing_buttons.addWidget(self.capture_min_btn)
        homing_buttons.addWidget(self.apply_homing_btn)
        homing_layout.addLayout(homing_buttons)

        self.homing_status_label = QLabel("Ready to start.")
        self.homing_status_label.setWordWrap(True)
        self.homing_status_label.setStyleSheet(f"color: {TEXT_SECONDARY};")
        homing_layout.addWidget(self.homing_status_label)
        top_layout.addWidget(homing_box, 1)

        stats_box = card("Live Motor Status")
        stats_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        stats_layout = QGridLayout(stats_box)
        stats_layout.setContentsMargins(12, 18, 12, 12)
        stats_layout.setHorizontalSpacing(12)
        stats_layout.setVerticalSpacing(8)
        content_layout.addWidget(stats_box)

        self.value_labels = {}
        fields = [
            "Selected motor",
            "Calibration state",
            "Last command (cm)",
            "Last feedback (cm)",
            "Command error (cm)",
            "Estimated ticks",
            "Session min (cm)",
            "Session max (cm)",
            "Session span (cm)",
        ]
        for row, field in enumerate(fields):
            name = QLabel(field)
            name.setStyleSheet(f"color: {TEXT_SECONDARY}; font-weight: 700;")
            value = QLabel("-")
            value.setStyleSheet("font-size: 16px; font-weight: 800;")
            stats_layout.addWidget(name, row, 0)
            stats_layout.addWidget(value, row, 1)
            self.value_labels[field] = value

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.poll_serial)
        self.poll_timer.start(50)

        self.on_motor_changed(0)
        self.load_homing_values_from_config()
        self.refresh_homing_table()

    def append_log(self, text):
        self.notification_label.setText(text)

    def load_homing_values_from_config(self):
        self.homing_min = list(self.cfg["actuators"].get("motor_min_cm", [None] * 6))
        self.homing_max = list(self.cfg["actuators"].get("motor_max_cm", [None] * 6))

    def refresh_homing_table(self):
        for idx in range(6):
            motor_item = QTableWidgetItem(f"Motor {idx + 1}")
            min_value = self.homing_min[idx]
            max_value = self.homing_max[idx]
            min_item = QTableWidgetItem(
                "-" if min_value is None else f"{float(min_value):.2f} cm"
            )
            max_item = QTableWidgetItem(
                "-" if max_value is None else f"{float(max_value):.2f} cm"
            )
            done = min_value is not None and max_value is not None
            if done:
                span = float(max_value) - float(min_value)
                direction = "+10" if span >= 0.0 else "-10"
                status_text = f"Saved ({direction})"
            else:
                status_text = "Pending"
            status_item = QTableWidgetItem(status_text)
            status_item.setForeground(Qt.white if done else Qt.lightGray)
            self.homing_table.setItem(idx, 0, motor_item)
            self.homing_table.setItem(idx, 1, min_item)
            self.homing_table.setItem(idx, 2, max_item)
            self.homing_table.setItem(idx, 3, status_item)

        complete = all(
            min_value is not None and max_value is not None
            for min_value, max_value in zip(self.homing_min, self.homing_max)
        )
        self.apply_homing_btn.setEnabled(complete)
        saved_count = sum(value is not None for value in self.homing_min)
        self.progress_label.setText(f"{saved_count} of 6 motors saved")
        if complete:
            self.homing_status_label.setText(
                "All motors saved. Apply calibration to finish."
            )
        else:
            self.homing_status_label.setText(
                f"{saved_count}/6 motors saved."
            )

    def reset_session_bounds(self):
        self.session_min = [None] * 6
        self.session_max = [None] * 6
        self.refresh_live_values()

    def sync_command_to_feedback(self):
        self.last_command = [round(value, 2) for value in self.last_feedback]
        self.refresh_live_values()
        self.target_spin.blockSignals(True)
        self.target_spin.setValue(self.last_feedback[self.selected_motor_index])
        self.target_spin.blockSignals(False)

    def set_motion_controls_enabled(self, enabled):
        self.send_btn.setEnabled(enabled)
        self.jog_minus_btn.setEnabled(enabled)
        self.jog_plus_btn.setEnabled(enabled)
        self.zero_btn.setEnabled(enabled)
        self.capture_min_btn.setEnabled(enabled)
        self.reverse_travel_btn.setEnabled(enabled)

    def set_connection_state(self, connected):
        self.connect_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)
        self.set_motion_controls_enabled(connected and self.feedback_ready)
        self.status_label.setText("Connected" if connected else "Disconnected")
        self.status_label.setStyleSheet(
            f"color: {SUCCESS if connected else DANGER}; font-weight: 700;"
        )

    def connect_serial(self):
        if self.ser is not None:
            return
        try:
            ser_cfg = self.cfg["serial"]
            self.ser = serial.Serial(
                ser_cfg["port"], ser_cfg["baudrate"], timeout=ser_cfg["timeout"]
            )
            time.sleep(ser_cfg["startup_delay"])
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.feedback_ready = False
            self.reset_session_bounds()
            self.set_connection_state(True)
            self.append_log("Serial link opened. Waiting for first valid feedback frame.")
        except Exception as exc:
            self.ser = None
            QMessageBox.critical(self, "Serial Error", str(exc))
            self.append_log(f"Failed to open serial: {exc}")

    def disconnect_serial(self):
        if self.ser is not None:
            try:
                self.ser.close()
            finally:
                self.ser = None
        self.feedback_ready = False
        self.set_connection_state(False)
        self.reset_session_bounds()
        self.append_log("Serial link closed.")

    def on_motor_changed(self, index):
        self.selected_motor_index = index
        self._apply_selected_motor_range()
        self.refresh_live_values()
        self.target_spin.blockSignals(True)
        self.target_spin.setValue(self.last_feedback[index] if self.feedback_ready else 0.0)
        self.target_spin.blockSignals(False)

    def _apply_selected_motor_range(self):
        self.target_spin.setRange(self.FREE_TARGET_MIN_CM, self.FREE_TARGET_MAX_CM)

    def build_vector(self, selected_value):
        # During homing, freeze every non-selected motor at its latest measured
        # position so only the selected motor receives a new target.
        vector = [round(float(value), 2) for value in self.last_feedback]
        vector[self.selected_motor_index] = round(float(selected_value), 2)
        return vector

    def send_vector(self, vector, label):
        if self.ser is None:
            return
        if not self.feedback_ready:
            QMessageBox.warning(
                self,
                "Feedback Required",
                "Wait for the first valid feedback frame before sending motion commands.",
            )
            return
        message = ",".join(f"{value:.2f}" for value in vector) + "\n"
        try:
            self.ser.write(message.encode())
            self.last_command = vector[:]
            self.refresh_live_values()
            self.append_log(f"{label}: {message.strip()}")
        except Exception as exc:
            self.append_log(f"Write error: {exc}")
            QMessageBox.critical(self, "Write Error", str(exc))

    def send_target(self):
        self.send_vector(self.build_vector(self.target_spin.value()), "Target sent")

    def jog_negative(self):
        new_value = self.target_spin.value() - self.step_spin.value()
        self.target_spin.setValue(new_value)
        self.send_target()

    def jog_positive(self):
        new_value = self.target_spin.value() + self.step_spin.value()
        self.target_spin.setValue(new_value)
        self.send_target()

    def send_selected_zero(self):
        self.send_vector(self.build_vector(0.0), "Selected motor zero sent")

    def toggle_selected_travel_direction(self):
        idx = self.selected_motor_index
        home_value = self.homing_min[idx]
        if home_value is None:
            QMessageBox.warning(
                self,
                "Home Required",
                "Save the selected motor home position before flipping travel.",
            )
            return

        home_value = float(home_value)
        current_max = self.homing_max[idx]
        if current_max is not None and float(current_max) < home_value:
            self.homing_max[idx] = round(home_value + self.HOMING_SPAN_CM, 2)
            direction = "+10 cm"
        else:
            self.homing_max[idx] = round(home_value - self.HOMING_SPAN_CM, 2)
            direction = "-10 cm"

        self.refresh_homing_table()
        self.append_log(
            f"Motor {idx + 1} travel direction set to {direction} from home."
        )

    def capture_current_minimum(self):
        if self.ser is None:
            QMessageBox.warning(self, "Serial Required", "Connect to the controller first.")
            return
        if not self.feedback_ready:
            QMessageBox.warning(
                self,
                "Feedback Required",
                "Wait for the first valid feedback frame before saving a minimum.",
            )
            return

        idx = self.selected_motor_index
        current_feedback = round(float(self.last_feedback[idx]), 2)
        current_max = self.homing_max[idx]
        direction = (
            -1.0
            if current_max is not None and float(current_max) < current_feedback
            else 1.0
        )
        self.homing_min[idx] = current_feedback
        self.homing_max[idx] = round(
            current_feedback + direction * self.HOMING_SPAN_CM,
            2,
        )
        self.refresh_homing_table()
        self.append_log(
            f"Saved Motor {idx + 1} minimum at {current_feedback:.2f} cm "
            f"-> travel limit set to {self.homing_max[idx]:.2f} cm"
        )

    def apply_homing_calibration(self):
        if not all(
            min_value is not None and max_value is not None
            for min_value, max_value in zip(self.homing_min, self.homing_max)
        ):
            QMessageBox.warning(
                self,
                "Incomplete Homing",
                "Save a home value and travel limit for all six motors first.",
            )
            return

        invalid_spans = []
        for idx, (min_value, max_value) in enumerate(
            zip(self.homing_min, self.homing_max)
        ):
            span = float(max_value) - float(min_value)
            if abs(abs(span) - self.HOMING_SPAN_CM) > 1e-6:
                invalid_spans.append(f"Motor {idx + 1}: {span:+.2f} cm")

        if invalid_spans:
            QMessageBox.warning(
                self,
                "Invalid Travel",
                "Each motor travel must be exactly +10.00 cm or -10.00 cm.\n"
                + "\n".join(invalid_spans),
            )
            return

        cfg = get_config(self.config_path)
        cfg["actuators"]["motor_min_cm"] = [round(float(v), 2) for v in self.homing_min]
        cfg["actuators"]["motor_max_cm"] = [round(float(v), 2) for v in self.homing_max]
        cfg["actuators"]["logical_min_cm"] = 0.0
        cfg["actuators"]["logical_max_cm"] = self.HOMING_SPAN_CM
        cfg["actuators"]["home_vector"] = [0.0] * 6
        save_config(cfg, self.config_path)

        self.cfg = cfg
        self.calibration = ActuatorCalibration.from_config(self.cfg["actuators"])
        self.runtime_state.mark_manual_recovery_complete()
        self.append_log(
            "Homing calibration saved to YAML. Startup recovery marked as complete."
        )
        QMessageBox.information(
            self,
            "Homing Saved",
            "Motor home values were saved. Each travel limit is home +/-10.00 cm.\n"
            "Restart the motion nodes so they reload the updated calibration.",
        )

    def update_session_bounds(self, feedback):
        for idx, value in enumerate(feedback):
            if self.session_min[idx] is None or value < self.session_min[idx]:
                self.session_min[idx] = value
            if self.session_max[idx] is None or value > self.session_max[idx]:
                self.session_max[idx] = value

    def refresh_live_values(self):
        idx = self.selected_motor_index
        command = self.last_command[idx]
        feedback = self.last_feedback[idx]
        error = command - feedback
        ticks = feedback * TICKS_PER_TURN / PULLEY_CIRCUMFERENCE_CM
        min_seen = self.session_min[idx]
        max_seen = self.session_max[idx]
        span = None
        if min_seen is not None and max_seen is not None:
            span = max_seen - min_seen
        homed = self.homing_min[idx] is not None

        self.value_labels["Selected motor"].setText(f"Motor {idx + 1}")
        self.value_labels["Calibration state"].setText("Saved" if homed else "Not saved")
        self.value_labels["Last command (cm)"].setText(f"{command:.2f}")
        self.value_labels["Last feedback (cm)"].setText(f"{feedback:.2f}")
        self.value_labels["Command error (cm)"].setText(f"{error:+.2f}")
        self.value_labels["Estimated ticks"].setText(f"{ticks:.0f}")
        self.value_labels["Session min (cm)"].setText(
            "-" if min_seen is None else f"{min_seen:.2f}"
        )
        self.value_labels["Session max (cm)"].setText(
            "-" if max_seen is None else f"{max_seen:.2f}"
        )
        self.value_labels["Session span (cm)"].setText(
            "-" if span is None else f"{span:.2f}"
        )

    def poll_serial(self):
        if self.ser is None:
            return

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode(errors="ignore").strip()
                if not line or "," not in line:
                    continue

                try:
                    feedback = [float(x) for x in line.split(",") if x.strip()]
                except ValueError:
                    self.append_log(f"Ignored invalid feedback: {line}")
                    continue

                if len(feedback) != 6:
                    self.append_log(f"Ignored incomplete feedback: {line}")
                    continue

                self.last_feedback = feedback
                if not self.feedback_ready:
                    self.feedback_ready = True
                    self.set_connection_state(True)
                    self.append_log("First valid feedback received. Motion controls enabled.")
                if self.last_command == [0.0] * 6:
                    self.sync_command_to_feedback()
                self.update_session_bounds(feedback)
                self.refresh_live_values()
        except Exception as exc:
            self.append_log(f"Read error: {exc}")

    def closeEvent(self, event):
        self.disconnect_serial()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    gui = MotorTestInterface()
    gui.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
