import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from tkinter import *
from tkinter import ttk
import serial
import time

class ArduinoCommunication:
    def __init__(self, port="COM7", baudrate=115200, timeout=0.1):
        try:
            self.arduino = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(2)  # Wait for connection
            self.arduino.reset_input_buffer()
            self.connected = True
            print("Arduino connected")
        except Exception as e:
            print(f"Failed to connect to Arduino: {str(e)}")
            self.connected = False
    
    def reconnect(self, port):
        """Try to reconnect to Arduino"""
        try:
            if self.connected and self.arduino.is_open:
                self.arduino.close()
            
            self.arduino = serial.Serial(port, 115200, timeout=0.1)
            time.sleep(2)  # Wait for connection
            self.arduino.reset_input_buffer()
            self.connected = True
            print(f"Reconnected to Arduino ({port})")
            return True
        except Exception as e:
            self.connected = False
            print(f"Failed to reconnect: {str(e)}")
            return False

    def send_data(self, data):
        if self.connected and self.arduino.is_open:
            formatted_data = str(data) + "\n"
            self.arduino.write(formatted_data.encode('utf-8'))
            self.arduino.flush()
            return True
        return False

    def read_data(self):
        if self.connected and self.arduino.is_open and self.arduino.in_waiting > 0:
            return self.arduino.readline().decode('utf-8', errors='ignore').strip()
        return None

    def close(self):
        if self.connected and self.arduino.is_open:
            self.arduino.close()
            print("Arduino connection closed")

def dual_arm_forward_kinematics(l0, l1, l2, theta_a1, theta_a2):
    """
    Calculate forward kinematics for dual arm plotter
    
    Parameters:
    - l0: Distance from base center to active joint
    - l1: Length of first link
    - l2: Length of second link
    - theta_a1: Left active joint angle (radians)
    - theta_a2: Right active joint angle (radians)
    
    Returns:
    - end_x, end_y: End effector position
    - theta_p1, theta_p2: Passive joint angles
    """
    # Calculate left passive joint position
    x1 = -l0 + l1 * np.cos(theta_a1)
    y1 = l1 * np.sin(theta_a1)
    
    # Calculate right passive joint position
    x2 = l0 + l1 * np.cos(theta_a2)
    y2 = l1 * np.sin(theta_a2)
    
    # Calculate distance between passive joints
    d = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    # Check if the structure is valid
    if d > 2 * l2:
        return None, None, None, None
    
    # Calculate end effector position
    cos_angle = (2 * l2**2 - d**2) / (2 * l2**2)
    h = l2 * np.sqrt(1 - (d/(2*l2))**2)  # height
    
    # Calculate midpoint
    mid_x = (x1 + x2) / 2
    mid_y = (y1 + y2) / 2
    
    # Vector from midpoint to end effector
    dx = x2 - x1
    dy = y2 - y1
    
    # Rotate vector 90 degrees
    end_x = mid_x - dy * h / d
    end_y = mid_y + dx * h / d
    
    # Calculate passive joint angles
    theta_p1 = np.arctan2(end_y - y1, end_x - x1) - theta_a1
    theta_p2 = np.arctan2(end_y - y2, end_x - x2) - theta_a2
    
    return end_x, end_y, theta_p1, theta_p2

class DualArmPlotterVisualizer:
    def __init__(self, root, arduino_comm=None):
        self.root = root
        self.root.title("CNC Dual Arm Plotter")
        self.root.resizable(False, False)  # Fix window size
        
        # Robot parameters
        self.l0 = 5.0  # Base to joint distance
        self.l1 = 8.0  # First link length
        self.l2 = 9.0  # Second link length
        
        # Arduino communication
        self.arduino_comm = arduino_comm
        self.com_ports = ["COM1", "COM2", "COM3", "COM4", "COM5", "COM6", "COM7"]
        
        # Create GUI elements
        self.create_widgets()
        
        # Start polling if Arduino is connected
        if self.arduino_comm and self.arduino_comm.connected:
            self.poll_arduino()
    
    def create_widgets(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.grid(row=0, column=0, sticky=(N, W, E, S))
        
        # Control panel
        control_frame = ttk.LabelFrame(main_frame, padding=10)
        control_frame.grid(row=0, column=0, padx=10, pady=10, sticky=(N, W))
        
        # Joint angle controls
        ttk.Label(control_frame, text="Theta A1:").grid(row=0, column=0, sticky=W, pady=5)
        self.theta_a1_var = DoubleVar(value=100) 
        theta_a1_slider = ttk.Scale(control_frame, from_=-360, to=360, orient=HORIZONTAL, 
                         variable=self.theta_a1_var, length=200,
                         command=lambda _: self.update_display())
        theta_a1_slider.grid(row=0, column=1, padx=5, pady=5)
        self.a1_label = ttk.Label(control_frame, text="100.0") 
        self.a1_label.grid(row=0, column=2)

        ttk.Label(control_frame, text="Theta A2:").grid(row=1, column=0, sticky=W, pady=5)
        self.theta_a2_var = DoubleVar(value=80)  
        theta_a2_slider = ttk.Scale(control_frame, from_=-360, to=360, orient=HORIZONTAL, 
                         variable=self.theta_a2_var, length=200,
                         command=lambda _: self.update_display())
        theta_a2_slider.grid(row=1, column=1, padx=5, pady=5)
        self.a2_label = ttk.Label(control_frame, text="80.0")  
        self.a2_label.grid(row=1, column=2)
        # Position display
        ttk.Label(control_frame, text="Position:").grid(row=2, column=0, sticky=W, pady=5)
        self.position_label = ttk.Label(control_frame, text="X: 0.0, Y: 0.0")
        self.position_label.grid(row=2, column=1, columnspan=2, sticky=W, pady=5)
        
        ttk.Label(control_frame, text="P angle:").grid(row=3, column=0, sticky=W, pady=5)
        self.passive_label = ttk.Label(control_frame, text="P1: 0.0°, P2: 0.0°")
        self.passive_label.grid(row=3, column=1, columnspan=2, sticky=W, pady=5)
        
        # Workspace button
        ttk.Button(control_frame, text="Show Workspace", 
                  command=self.plot_workspace).grid(row=4, column=0, columnspan=3, pady=10)
        
        # Arduino control
        arduino_frame = ttk.LabelFrame(control_frame, text="Arduino Control", padding=5)
        arduino_frame.grid(row=5, column=0, columnspan=3, pady=10, sticky=(W, E))
        
        # COM port selection
        ttk.Label(arduino_frame, text="COM Port:").grid(row=0, column=0, sticky=W, pady=5)
        self.port_var = StringVar(value="COM7")
        port_combo = ttk.Combobox(arduino_frame, textvariable=self.port_var, values=self.com_ports)
        port_combo.grid(row=0, column=1, sticky=W, pady=5)
        
        # Connect button
        ttk.Button(arduino_frame, text="Connect", 
                  command=self.reconnect_arduino).grid(row=0, column=2, padx=5, pady=5)
        
        # Send button
        ttk.Button(arduino_frame, text="Send to Arduino", 
                  command=self.send_to_arduino).grid(row=1, column=0, columnspan=3, pady=5)
        
        ttk.Label(arduino_frame, text="Status:").grid(row=2, column=0, sticky=W, pady=5)
        status = "Connected" if self.arduino_comm and self.arduino_comm.connected else "Not Connected"
        self.arduino_status = ttk.Label(arduino_frame, text=status)
        self.arduino_status.grid(row=2, column=1, columnspan=2, sticky=W, pady=5)
        
        ttk.Label(arduino_frame, text="Feedback:").grid(row=3, column=0, sticky=W, pady=5)
        self.arduino_feedback = ttk.Label(arduino_frame, text="No data")
        self.arduino_feedback.grid(row=3, column=1, columnspan=2, sticky=W, pady=5)
        
        # Visualization panel
        plot_frame = ttk.LabelFrame(main_frame, text="Simulation", padding=10)
        plot_frame.grid(row=0, column=1, rowspan=2, padx=10, pady=10, sticky=(N, E))
        
        # Create plot
        self.fig = plt.Figure(figsize=(8, 8), dpi=100)
        self.ax = self.fig.add_subplot(111)
        
        # Set plot properties
        max_reach = self.l0 + self.l1 + self.l2
        self.ax.set_xlim(-max_reach * 1.2, max_reach * 1.2)
        self.ax.set_ylim(-max_reach * 0.2, max_reach * 1.2)
        self.ax.grid(True)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        
        # Draw robot base
        self.base, = self.ax.plot([-self.l0, self.l0], [0, 0], 'k-', linewidth=3)
        self.ax.scatter([-self.l0, self.l0], [0, 0], color='black', s=100, marker='o', zorder=5)
        
        # Create plot elements
        self.left_arm, = self.ax.plot([], [], 'b-', linewidth=2, label='Left arm')
        self.right_arm, = self.ax.plot([], [], 'g-', linewidth=2, label='Right arm')
        self.end_effector, = self.ax.plot([], [], 'ro', markersize=10, label='Position')
        
        # Add legend
        self.ax.legend()
        
        # Embed in Tkinter
        canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(side=TOP, fill=BOTH, expand=1)
        
        # Initial display update
        self.update_display()
    
    def update_display(self):
        """Update display"""
        # Get angles
        theta_a1_deg = self.theta_a1_var.get()
        theta_a2_deg = self.theta_a2_var.get()
        
        # Update angle labels
        self.a1_label.config(text=f"{theta_a1_deg:.1f}")
        self.a2_label.config(text=f"{theta_a2_deg:.1f}")
        
        # Convert to radians
        theta_a1 = np.radians(theta_a1_deg)
        theta_a2 = np.radians(theta_a2_deg)
        
        # Calculate forward kinematics
        end_x, end_y, theta_p1, theta_p2 = dual_arm_forward_kinematics(
            self.l0, self.l1, self.l2, theta_a1, theta_a2)
        
        # If structure is invalid
        if end_x is None:
            self.position_label.config(text="Impossible position")
            self.passive_label.config(text="Invalid angles")
            
            # Clear plots
            self.left_arm.set_data([], [])
            self.right_arm.set_data([], [])
            self.end_effector.set_data([], [])
            self.fig.canvas.draw()
            return
        
        # Update position display
        self.position_label.config(text=f"X: {end_x:.1f}, Y: {end_y:.1f}")
        
        # Update passive joint angle display
        self.passive_label.config(
            text=f"P1: {np.degrees(theta_p1):.1f}°, P2: {np.degrees(theta_p2):.1f}°")
        
        # Calculate joint positions
        a1_x, a1_y = -self.l0, 0
        a2_x, a2_y = self.l0, 0
        
        p1_x = a1_x + self.l1 * np.cos(theta_a1)
        p1_y = a1_y + self.l1 * np.sin(theta_a1)
        
        p2_x = a2_x + self.l1 * np.cos(theta_a2)
        p2_y = a2_y + self.l1 * np.sin(theta_a2)
        
        # Update robot arm plots
        self.left_arm.set_data([a1_x, p1_x, end_x], [a1_y, p1_y, end_y])
        self.right_arm.set_data([a2_x, p2_x, end_x], [a2_y, p2_y, end_y])
        self.end_effector.set_data([end_x], [end_y])
        
        self.fig.canvas.draw()
    
    def plot_workspace(self):
        """Plot robot workspace"""
        workspace_window = Toplevel(self.root)
        workspace_window.title("Robot Workspace")
        workspace_window.resizable(False, False)
        
        ws_fig = plt.Figure(figsize=(8, 8), dpi=100)
        ws_ax = ws_fig.add_subplot(111)
        
        max_reach = self.l0 + self.l1 + self.l2
        ws_ax.set_xlim(-max_reach * 1.2, max_reach * 1.2)
        ws_ax.set_ylim(-max_reach * 0.2, max_reach * 1.2)
        ws_ax.grid(True)
        ws_ax.set_aspect('equal')
        ws_ax.set_title('Robot Workspace')
        
        ws_ax.plot([-self.l0, self.l0], [0, 0], 'k-', linewidth=3)
        ws_ax.scatter([-self.l0, self.l0], [0, 0], color='black', s=100, marker='o')
        
        positions_x = []
        positions_y = []
        
        for a1 in range(0, 361, 15):  
            for a2 in range(0, 361, 15):
                theta_a1 = np.radians(a1)
                theta_a2 = np.radians(a2)
                
                end_x, end_y, _, _ = dual_arm_forward_kinematics(
                    self.l0, self.l1, self.l2, theta_a1, theta_a2)
                
                if end_x is not None:
                    positions_x.append(end_x)
                    positions_y.append(end_y)
        
        ws_ax.scatter(positions_x, positions_y, s=1, color='blue', alpha=0.5)
        
        canvas = FigureCanvasTkAgg(ws_fig, master=workspace_window)
        canvas.draw()
        canvas.get_tk_widget().pack(side=TOP, fill=BOTH, expand=1)
    
    def reconnect_arduino(self):
        """Reconnect to Arduino"""
        port = self.port_var.get()
        
        if not self.arduino_comm:
            # Create new communication object
            try:
                self.arduino_comm = ArduinoCommunication(port=port)
                if self.arduino_comm.connected:
                    self.arduino_status.config(text="Connected")
                    self.poll_arduino()
                else:
                    self.arduino_status.config(text="Connection Failed")
            except:
                self.arduino_status.config(text="Connection Error")
        else:
            # Reconnect using existing object
            if self.arduino_comm.reconnect(port):
                self.arduino_status.config(text="Connected")
                self.poll_arduino()
            else:
                self.arduino_status.config(text="Reconnection Failed")
    
    def send_to_arduino(self):
        """Send current angles to Arduino"""
        if not (self.arduino_comm and self.arduino_comm.connected):
            self.arduino_feedback.config(text="Arduino not connected")
            return
        
        # Get current angles
        theta_a1_deg = self.theta_a1_var.get()
        theta_a2_deg = self.theta_a2_var.get()
        
        data = f"{theta_a1_deg:.1f},{theta_a2_deg:.1f}"
        if self.arduino_comm.send_data(data):
           self.arduino_feedback.config(text=f"Sent: {data}")
        else:
           self.arduino_feedback.config(text="Send failed")
    
    def poll_arduino(self):
        """Periodically check Arduino data"""
        if self.arduino_comm and self.arduino_comm.connected:
            data = self.arduino_comm.read_data()
            if data:
                self.arduino_feedback.config(text=f"Received: {data}")
            
            self.root.after(100, self.poll_arduino)
        else:
            self.arduino_status.config(text="Disconnected")

# Start program
if __name__ == "__main__":
    root = Tk()
    
    # Try to connect to Arduino
    arduino_comm = None
    try:
        arduino_comm = ArduinoCommunication(port="COM7", baudrate=115200)
    except:
        print("Failed to connect to Arduino")
    
    app = DualArmPlotterVisualizer(root, arduino_comm)
    
    try:
        root.mainloop()
    finally:
        if arduino_comm and hasattr(arduino_comm, 'connected') and arduino_comm.connected:
            arduino_comm.close()
