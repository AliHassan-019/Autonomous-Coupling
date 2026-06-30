import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import serial
import serial.tools.list_ports


BAUD_RATE = 115200
ACTUATOR_MIN_CM = 0.0
ACTUATOR_MAX_CM = 81.0
MIN_SPEED =100
MAX_SPEED = 300
DEFAULT_SPEED = 150


class StepperMotorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Stepper Motor Control")
        self.root.geometry("620x390")
        self.root.minsize(580, 360)

        self.ser = None
        self.connected = False

        self.port_var = tk.StringVar()
        self.connection_var = tk.StringVar(value="Disconnected")
        self.speed_var = tk.IntVar(value=DEFAULT_SPEED)
        self.speed_display_var = tk.StringVar(value=f"{DEFAULT_SPEED} steps/s")
        self.distance_var = tk.StringVar(value="5.0")
        self.notification_var = tk.StringVar(value="Ready")

        self.configure_style()
        self.build_layout()
        self.refresh_ports()
        self.set_controls_enabled(False)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def configure_style(self):
        self.root.configure(bg="#f5f7fa")
        style = ttk.Style()
        style.theme_use("clam")

        style.configure(".", font=("Segoe UI", 10), background="#f5f7fa")
        style.configure("TFrame", background="#f5f7fa")
        style.configure("Panel.TFrame", background="#ffffff")
        style.configure("Title.TLabel", font=("Segoe UI", 20, "bold"), foreground="#111827", background="#f5f7fa")
        style.configure("Muted.TLabel", foreground="#667085", background="#f5f7fa")
        style.configure("PanelTitle.TLabel", font=("Segoe UI", 11, "bold"), foreground="#111827", background="#ffffff")
        style.configure("PanelText.TLabel", foreground="#344054", background="#ffffff")
        style.configure("Metric.TLabel", font=("Segoe UI", 22, "bold"), foreground="#111827", background="#ffffff")
        style.configure("Status.TLabel", font=("Segoe UI", 10, "bold"), foreground="#b42318", background="#ffffff")

        style.configure("TButton", padding=(14, 8), background="#eef2f6", foreground="#111827")
        style.map("TButton", background=[("active", "#e3e8ef"), ("disabled", "#f2f4f7")])
        style.configure("Primary.TButton", background="#2563eb", foreground="#ffffff")
        style.map("Primary.TButton", background=[("active", "#1d4ed8"), ("disabled", "#c7d2fe")])
        style.configure("Danger.TButton", background="#dc2626", foreground="#ffffff")
        style.map("Danger.TButton", background=[("active", "#b91c1c"), ("disabled", "#fecaca")])
        style.configure("TEntry", fieldbackground="#ffffff", padding=8)
        style.configure("TCombobox", fieldbackground="#ffffff", padding=6)

    def build_layout(self):
        outer = ttk.Frame(self.root, padding=22)
        outer.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(3, weight=1)

        ttk.Label(outer, text="Stepper Motor Control", style="Title.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(outer, text="Speed, distance, homing, and stop controls.", style="Muted.TLabel").grid(
            row=1, column=0, sticky="w", pady=(3, 18)
        )

        connection = self.panel(outer)
        connection.grid(row=2, column=0, sticky="ew", pady=(0, 14))
        connection.columnconfigure(1, weight=1)

        ttk.Label(connection, text="Connection", style="PanelTitle.TLabel").grid(row=0, column=0, sticky="w", padx=16, pady=(14, 10))
        ttk.Label(connection, textvariable=self.connection_var, style="Status.TLabel").grid(row=0, column=3, sticky="e", padx=16, pady=(14, 10))

        ttk.Label(connection, text="Port", style="PanelText.TLabel").grid(row=1, column=0, sticky="w", padx=(16, 8), pady=(0, 16))
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, state="readonly", width=18)
        self.port_combo.grid(row=1, column=1, sticky="ew", padx=(0, 8), pady=(0, 16))
        ttk.Button(connection, text="Refresh", command=self.refresh_ports).grid(row=1, column=2, sticky="e", padx=(0, 8), pady=(0, 16))
        self.connect_button = ttk.Button(connection, text="Connect", command=self.toggle_connection, style="Primary.TButton")
        self.connect_button.grid(row=1, column=3, sticky="e", padx=(0, 16), pady=(0, 16))

        notification = self.panel(outer)
        notification.grid(row=3, column=0, sticky="ew", pady=(0, 14))
        ttk.Label(notification, textvariable=self.notification_var, style="PanelText.TLabel").grid(
            row=0, column=0, sticky="w", padx=16, pady=12
        )

        controls = self.panel(outer)
        controls.grid(row=4, column=0, sticky="nsew")
        controls.columnconfigure(1, weight=1)

        ttk.Label(controls, text="Motion Control", style="PanelTitle.TLabel").grid(row=0, column=0, sticky="w", padx=18, pady=(18, 14))

        ttk.Label(controls, text="Motor Speed", style="PanelText.TLabel").grid(row=1, column=0, sticky="w", padx=18, pady=8)
        self.speed_slider = ttk.Scale(
            controls,
            from_=MIN_SPEED,
            to=MAX_SPEED,
            orient=tk.HORIZONTAL,
            variable=self.speed_var,
            command=self.update_speed_label,
        )
        self.speed_slider.grid(row=1, column=1, sticky="ew", pady=8)
        ttk.Label(controls, textvariable=self.speed_display_var, style="PanelText.TLabel", width=13).grid(
            row=1, column=2, sticky="e", padx=(18, 8), pady=8
        )
        self.speed_button = ttk.Button(controls, text="Set Speed", command=self.apply_speed)
        self.speed_button.grid(row=1, column=3, sticky="e", padx=(0, 18), pady=8)

        ttk.Label(controls, text="Distance", style="PanelText.TLabel").grid(row=2, column=0, sticky="w", padx=18, pady=8)
        self.distance_entry = ttk.Entry(controls, textvariable=self.distance_var, width=12)
        self.distance_entry.grid(row=2, column=1, sticky="ew", pady=8)
        self.distance_entry.bind("<Return>", lambda _event: self.move_to_distance())
        ttk.Label(controls, text="cm", style="PanelText.TLabel").grid(row=2, column=2, sticky="e", padx=18, pady=8)

        buttons = ttk.Frame(controls, style="Panel.TFrame")
        buttons.grid(row=3, column=0, columnspan=4, sticky="ew", padx=18, pady=(22, 18))
        buttons.columnconfigure(0, weight=1)
        buttons.columnconfigure(1, weight=1)
        buttons.columnconfigure(2, weight=1)

        self.home_button = ttk.Button(buttons, text="Home", command=self.home_position)
        self.home_button.grid(row=0, column=0, sticky="ew", padx=(0, 8))
        self.goto_button = ttk.Button(buttons, text="Move to Distance", command=self.move_to_distance, style="Primary.TButton")
        self.goto_button.grid(row=0, column=1, sticky="ew", padx=8)
        self.stop_button = ttk.Button(buttons, text="Stop", command=self.stop_motor, style="Danger.TButton")
        self.stop_button.grid(row=0, column=2, sticky="ew", padx=(8, 0))

    def panel(self, parent):
        return ttk.Frame(parent, style="Panel.TFrame")

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports if ports else ["No ports found"]
        if ports and (not self.port_var.get() or self.port_var.get() == "No ports found"):
            self.port_var.set(ports[0])
        elif not ports:
            self.port_var.set("No ports found")

    def toggle_connection(self):
        if self.connected:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        port = self.port_var.get()
        if not port or port == "No ports found":
            messagebox.showerror("Connection Error", "No serial port selected.")
            return

        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=0.2)
            time.sleep(2)
            self.connected = True
            self.connection_var.set(f"Connected: {port}")
            self.notify(f"Connected to {port}")
            self.connect_button.configure(text="Disconnect")
            self.set_controls_enabled(True)
            threading.Thread(target=self.read_serial, daemon=True).start()
            self.apply_speed()
        except Exception as exc:
            self.connected = False
            self.ser = None
            self.notify("Connection failed")
            messagebox.showerror("Connection Error", str(exc))

    def disconnect_serial(self):
        self.connected = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connection_var.set("Disconnected")
        self.connect_button.configure(text="Connect")
        self.set_controls_enabled(False)
        self.notify("Disconnected")

    def read_serial(self):
        while self.connected and self.ser:
            try:
                self.ser.readline()
            except Exception:
                break

    def update_speed_label(self, _value=None):
        self.speed_display_var.set(f"{int(self.speed_var.get())} steps/s")

    def set_controls_enabled(self, enabled):
        state = "normal" if enabled else "disabled"
        for widget in (self.speed_slider, self.speed_button, self.distance_entry, self.home_button, self.goto_button, self.stop_button):
            widget.configure(state=state)
        self.port_combo.configure(state="disabled" if self.connected else "readonly")

    def send_command(self, command):
        if not self.connected or not self.ser:
            self.notify("Connect to the Arduino first")
            messagebox.showerror("Not Connected", "Connect to the Arduino first.")
            return False
        try:
            self.ser.write((command + "\n").encode("utf-8"))
            self.ser.flush()
            return True
        except Exception as exc:
            self.notify("Send failed")
            messagebox.showerror("Send Error", str(exc))
            return False

    def validate_distance(self):
        try:
            distance = float(self.distance_var.get())
        except ValueError:
            raise ValueError("Enter a numeric distance in centimeters.")
        if not ACTUATOR_MIN_CM <= distance <= ACTUATOR_MAX_CM:
            raise ValueError(f"Distance must be between {ACTUATOR_MIN_CM:g} and {ACTUATOR_MAX_CM:g} cm.")
        return distance

    def apply_speed(self):
        speed = max(MIN_SPEED, min(MAX_SPEED, int(self.speed_var.get())))
        self.speed_var.set(speed)
        self.speed_display_var.set(f"{speed} steps/s")
        if self.connected and self.send_command(f"SPEED {speed}"):
            self.notify(f"Speed set to {speed} steps/s")
            return True
        return False

    def move_to_distance(self):
        try:
            distance = self.validate_distance()
        except ValueError as exc:
            self.notify(str(exc))
            messagebox.showerror("Invalid Distance", str(exc))
            return
        if self.send_command(f"GOTO {distance:.3f}"):
            self.notify(f"Moving to {distance:.3f} cm")

    def home_position(self):
        if self.send_command("HOME"):
            self.notify("Home set to 0 cm")

    def stop_motor(self):
        if self.send_command("STOP"):
            self.notify("Stop command sent")

    def notify(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.notification_var.set(f"{timestamp}  {message}")

    def on_close(self):
        self.disconnect_serial()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    StepperMotorGUI(root)
    root.mainloop()
