import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time

class StepperMotorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Stepper Motor Control")
        self.root.geometry("600x500")
        
        self.ser = None
        self.connected = False
        
        # Create main frame
        main_frame = ttk.Frame(root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # ===== Serial Connection Section =====
        connection_frame = ttk.LabelFrame(main_frame, text="Serial Connection", padding="10")
        connection_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(connection_frame, text="COM Port:").grid(row=0, column=0, sticky=tk.W)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(connection_frame, textvariable=self.port_var, width=15, state='readonly')
        self.port_combo.grid(row=0, column=1, sticky=tk.W, padx=5)
        self.refresh_ports()
        
        ttk.Button(connection_frame, text="Refresh Ports", command=self.refresh_ports).grid(row=0, column=2, padx=5)
        ttk.Button(connection_frame, text="Connect", command=self.connect_serial).grid(row=0, column=3, padx=5)
        
        self.status_label = ttk.Label(connection_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=0, column=4, padx=10)
        
        # ===== Motor Speed Section =====
        speed_frame = ttk.LabelFrame(main_frame, text="Motor Speed Control", padding="10")
        speed_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        ttk.Label(speed_frame, text="Speed (50-500 μs):").grid(row=0, column=0, sticky=tk.W)
        self.speed_var = tk.IntVar(value=150)
        
        self.speed_slider = ttk.Scale(speed_frame, from_=50, to=500, orient=tk.HORIZONTAL, 
                                       variable=self.speed_var, command=self.update_speed_label)
        self.speed_slider.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=10)
        
        self.speed_label = ttk.Label(speed_frame, text="150 μs (Faster)", width=20)
        self.speed_label.grid(row=0, column=2, sticky=tk.W)
        
        speed_frame.columnconfigure(1, weight=1)
        
        # ===== Distance/Steps Input Section =====
        input_frame = ttk.LabelFrame(main_frame, text="Movement Input", padding="10")
        input_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        # Distance in cm
        ttk.Label(input_frame, text="Distance (cm):").grid(row=0, column=0, sticky=tk.W)
        self.distance_var = tk.DoubleVar(value=10.0)
        distance_entry = ttk.Entry(input_frame, textvariable=self.distance_var, width=15)
        distance_entry.grid(row=0, column=1, sticky=tk.W, padx=5)
        
        # OR Steps
        ttk.Label(input_frame, text="OR Steps:").grid(row=0, column=2, sticky=tk.W, padx=(20, 0))
        self.steps_var = tk.IntVar(value=4250)
        steps_entry = ttk.Entry(input_frame, textvariable=self.steps_var, width=15)
        steps_entry.grid(row=0, column=3, sticky=tk.W, padx=5)
        
        # ===== Direction Selection =====
        direction_frame = ttk.LabelFrame(main_frame, text="Direction", padding="10")
        direction_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        self.direction_var = tk.IntVar(value=1)
        ttk.Radiobutton(direction_frame, text="Forward", variable=self.direction_var, value=1).pack(side=tk.LEFT, padx=20)
        ttk.Radiobutton(direction_frame, text="Backward", variable=self.direction_var, value=0).pack(side=tk.LEFT, padx=20)
        
        # ===== Control Buttons =====
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=15)
        
        ttk.Button(button_frame, text="Move by Distance", command=self.move_distance).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Move by Steps", command=self.move_steps).pack(side=tk.LEFT, padx=5)
        tk.Button(button_frame, text="Stop", command=self.stop_motor, foreground="red", bg="white").pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Status", command=self.get_status).pack(side=tk.LEFT, padx=5)
        
        # ===== Output Console =====
        console_frame = ttk.LabelFrame(main_frame, text="Serial Output", padding="10")
        console_frame.grid(row=5, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=10)
        
        scrollbar = ttk.Scrollbar(console_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.console = tk.Text(console_frame, height=8, width=60, yscrollcommand=scrollbar.set)
        self.console.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.console.yview)
        
        # Configure grid weights
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(5, weight=1)
    
    def refresh_ports(self):
        """Refresh available COM ports"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports if ports else ['No ports found']
        if ports:
            self.port_combo.current(0)
    
    def connect_serial(self):
        """Connect to Arduino via serial"""
        if self.connected:
            self.disconnect_serial()
            return
        
        port = self.port_var.get()
        if not port or port == 'No ports found':
            messagebox.showerror("Error", "No COM port selected")
            return
        
        try:
            self.ser = serial.Serial(port, 9600, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            self.connected = True
            self.status_label.config(text="Connected ✓", foreground="green")
            self.log_message(f"Connected to {port}")
            
            # Start reading thread
            thread = threading.Thread(target=self.read_serial, daemon=True)
            thread.start()
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))
            self.log_message(f"Failed to connect: {str(e)}")
    
    def disconnect_serial(self):
        """Disconnect from Arduino"""
        if self.ser:
            self.ser.close()
        self.connected = False
        self.status_label.config(text="Disconnected", foreground="red")
        self.log_message("Disconnected from Arduino")
    
    def read_serial(self):
        """Read incoming serial messages"""
        while self.connected:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    message = self.ser.readline().decode('utf-8').strip()
                    if message:
                        self.log_message(f">> {message}")
            except Exception as e:
                self.log_message(f"Read error: {str(e)}")
                break
    
    def send_command(self, command):
        """Send command to Arduino"""
        if not self.connected or not self.ser:
            messagebox.showerror("Error", "Not connected to Arduino")
            return False
        
        try:
            self.ser.write((command + '\n').encode())
            self.log_message(f"<< {command}")
            return True
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send command: {str(e)}")
            self.log_message(f"Send error: {str(e)}")
            return False
    
    def move_distance(self):
        """Move motor by distance in cm"""
        try:
            distance = float(self.distance_var.get())
            speed = self.speed_var.get()
            direction = self.direction_var.get()
            
            if distance <= 0:
                messagebox.showerror("Error", "Distance must be positive")
                return
            
            command = f"DIST {distance} {speed} {direction}"
            self.send_command(command)
        except ValueError:
            messagebox.showerror("Error", "Invalid distance value")
    
    def move_steps(self):
        """Move motor by number of steps"""
        try:
            steps = int(self.steps_var.get())
            speed = self.speed_var.get()
            direction = self.direction_var.get()
            
            if steps <= 0:
                messagebox.showerror("Error", "Steps must be positive")
                return
            
            command = f"MOVE {steps} {speed} {direction}"
            self.send_command(command)
        except ValueError:
            messagebox.showerror("Error", "Invalid steps value")
    
    def stop_motor(self):
        """Send stop command"""
        self.send_command("STOP")
    
    def get_status(self):
        """Get motor status"""
        self.send_command("STATUS")
    
    def update_speed_label(self, value):
        """Update speed label when slider moves"""
        speed = int(float(value))
        speed_text = "Slower" if speed > 250 else "Faster" if speed < 150 else "Medium"
        self.speed_label.config(text=f"{speed} μs ({speed_text})")
    
    def log_message(self, message):
        """Log message to console"""
        self.console.insert(tk.END, message + '\n')
        self.console.see(tk.END)
        self.root.update()

if __name__ == "__main__":
    root = tk.Tk()
    gui = StepperMotorGUI(root)
    root.mainloop()
