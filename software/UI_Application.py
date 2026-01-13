import os
import math
import time
import numpy as np
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from Back_End import (generate_drawing_from_image, path_to_joint_angles_deg,path_to_joint_angles,send_trajectory_to_arduino_streaming,inside_workspace,forward_kinematics, mid_distance, back_length,fore_length, HOME_X, HOME_Y, inverse_kinematics, request_current_angles, set_calibration, map_ik_to_hardware,ArduinoCommunication)
from Ai_Image import generate_ai_image

class PlotterUI:
    """
    GUI system for the Robot actions
    Functions are imported from the other helper modules

    Features:
    - Load or AI-generate image.
    - Convert to single-stroke path using IK-based constraints.
    - Visualise reachable workspace.
    - Simulate robot arm movement for the path.
    - Send trajectory to Arduino with blocking handshake (perfromed via back_end).
    """

    def debug_draw_square(self):
        """
        Simple fixed square in drawing space to test stability.
        Uses same coordinate system as generate_drawing_from_image.
        """
        # sqaure coordinates
        x_min, x_max = -20, 20
        y_min, y_max = 80, 120

        square_path = [(x_min, y_min),(x_max, y_min),(x_max, y_max),(x_min, y_max),(x_min, y_min),]

        # update preview
        self.path = square_path
        self.angles_deg = path_to_joint_angles_deg(square_path)

        xs = [p[0] for p in square_path]
        ys = [p[1] for p in square_path]

        self.ax.clear()
        self.ax.plot(xs, ys, "-o", linewidth=0.8)
        self.ax.set_aspect("equal", "box")
        self.ax.set_xlabel("x (mm)")
        self.ax.set_ylabel("y (mm)")
        self.ax.set_title("Draw a Square")
        self.ax.grid(True)
        self.canvas.draw()

        self.points_label.config(text=f"Points: {len(square_path)}")
        self.time_label.config(text="Estimated time: (square)")
        self.status_label.config(text="Status:square ready")


    def __init__(self, root):
        self.root = root
        self.root.title("The Boss Robot Plotter UI")

        # Default starting state
        self.image_path = None          # chosen or generated image
        self.path = []                  # (x, y) path
        self.angles_deg = []            # joint angles in degrees
        self.estimated_time = 0.0       # time estimate

        #set default calubration state
        self.cal_index = 0
        self.cal_ideal = []
        self.cal_measured = []
        self.cal_in_progress = False

        # sets the deafult port and the resolution to medium and COM13, works on my laptop
        self.port_var = tk.StringVar(value="COM13")
        self.resolution_var = tk.StringVar(value="medium")
        self.click_move_enabled = tk.BooleanVar(value=False)

        self.arduino = None

        # Jogging state 
        self.jog_t1 = 0.0
        self.jog_t2 = 0.0


        # UI vuilding bloack
        self.build_layout()

    def get_arduino(self):
        """
        Ensure we have a persistent ArduinoCommunication instance.
        """
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("No port", "Please enter a valid COM port.")
            return None
        # if we already have an open connection, reuse it
        if self.arduino is not None and self.arduino.connected:
            return self.arduino

        # Otherwise, open a new one (this will reset the Arduino)
        self.status_label.config(text=f"Status: Connecting to {port}...")
        self.root.update_idletasks()

        self.arduino = ArduinoCommunication(port=port, baudrate=115200, timeout=0.1)
        if not self.arduino.connected:
            messagebox.showerror("Arduino", f"Could not open {port}")
            self.arduino = None
            self.status_label.config(text="Status: Arduino not connected")
            return None

        self.status_label.config(text=f"Status: Connected to {port}")
        return self.arduino

    def on_close(self):
        # Cleanly close Arduino connection on app exit
        if self.arduino is not None and self.arduino.connected:
            # Relax motors before closing
            try:
                self.arduino.send_data("FREE")
                time.sleep(0.05)
            except Exception:
                pass
            self.arduino.close()
        self.root.destroy()



    def build_layout(self):
        #Creates all the UI elements and places them

        # allow window to resize nicely
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.grid(row=0, column=0, sticky="nsew")
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)

        # LEFT: Controls
        control_frame = ttk.LabelFrame(main_frame, text="Controls", padding=10)
        control_frame.grid(row=0, column=0, sticky="nw")
        row = 0

        #  Load image 
        self.load_button = ttk.Button(control_frame,text="Load Image",command=self.load_image)
        self.load_button.grid(row=row, column=0, columnspan=2, sticky="ew", pady=5)
        row += 1

        # Selected file label
        self.image_label = ttk.Label(control_frame, text="No image selected")
        self.image_label.grid(row=row, column=0, columnspan=2, sticky="w")
        row += 1

        #  Resolution selection 
        ttk.Label(control_frame, text="Resolution:").grid(row=2, column=0, sticky="w", pady=(10, 2))
        self.res_combo = ttk.Combobox(control_frame,textvariable=self.resolution_var,values=["low", "medium", "high"],state="readonly",width=10,)
        self.res_combo.grid(row=row, column=1, sticky="w")
        self.res_combo.current(1)  # set to medium
        row += 1

        #  Generate path 
        self.gen_button = ttk.Button(control_frame,text="Generate Path",command=self.generate_path)
        self.gen_button.grid(row=3, column=0, columnspan=2, sticky="ew", pady=8)
        row += 1

        self.debug_square_button = ttk.Button(control_frame,text="Draw a square",command=self.debug_draw_square)
        self.debug_square_button.grid(row=4, column=0, columnspan=2, sticky="ew", pady=4)
        row += 1


        #  Show workspace 
        self.workspace_button = ttk.Button(control_frame,text="Show Workspace",command=self.show_workspace)
        self.workspace_button.grid(row=row, column=0, columnspan=2, sticky="ew", pady=4)
        row += 1

        #  Simulate robot movement (virtual only) 
        self.sim_button = ttk.Button(control_frame,text="Simulate Movement",command=self.simulate_movement)
        self.sim_button.grid(row=row, column=0, columnspan=2, sticky="ew", pady=4)
        row += 1

        #Calibration controls 
        calib_frame = ttk.LabelFrame(control_frame, text="Calibration (3 points)", padding=5)
        calib_frame.grid(row=row, column=0, columnspan=2, sticky="ew", pady=(6, 4))

        self.cal_status_label = ttk.Label(calib_frame, text="0 / 3 points captured")
        self.cal_status_label.grid(row=0, column=0, columnspan=2, sticky="w")

        self.cal_button = ttk.Button(calib_frame,text="Capture calibration point",command=self.capture_calibration_point)
        self.cal_button.grid(row=1, column=0, columnspan=2, sticky="ew", pady=3)

        # Home + click-to-move
        ttk.Button(control_frame, text="Go to Home", command=self.home_robot).grid(row=row, column=0, columnspan=2, sticky="ew", pady=4)
        row += 1

        ttk.Checkbutton(control_frame,text="Click-to-move active",variable=self.click_move_enabled,).grid(row=row, column=0, columnspan=2, sticky="w")
        row += 1

        #  Path info 
        self.points_label = ttk.Label(control_frame, text="Points: -")
        self.points_label.grid(row=row, column=0, columnspan=2, sticky="w")
        row += 1

        self.time_label = ttk.Label(control_frame, text="Estimated time: - s")
        self.time_label.grid(row=row, column=0, columnspan=2, sticky="w")
        row += 1

        #  AI prompt 
        ttk.Label(control_frame, text="AI prompt:").grid(row=row, column=0, sticky="w", pady=(10, 2))
        self.ai_prompt_var = tk.StringVar()
        self.ai_entry = ttk.Entry(control_frame,textvariable=self.ai_prompt_var,width=25)
        self.ai_entry.grid(row=row, column=1, sticky="w")
        row += 1

        self.ai_button = ttk.Button(control_frame,text="Generate AI Image",command=self.generate_ai_image_ui)
        self.ai_button.grid(row=row, column=0, columnspan=2, sticky="ew", pady=5)
        row += 1

        #  Arduino controls 
        arduino_frame = ttk.LabelFrame(control_frame, text="Arduino", padding=10)
        arduino_frame.grid(row=row, column=0, columnspan=2, sticky="ew", pady=(10, 0))

        ttk.Label(arduino_frame, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_entry = ttk.Entry(arduino_frame,textvariable=self.port_var,width=10)
        self.port_entry.grid(row=0, column=1, sticky="w")

        self.status_label = ttk.Label(arduino_frame, text="Status: Resting")
        self.status_label.grid(row=1, column=0, columnspan=2, sticky="w", pady=2)

        self.send_button = ttk.Button(arduino_frame,text="Send to Arduino",command=self.send_to_arduino)
        self.send_button.grid(row=2, column=0, columnspan=2, sticky="ew", pady=5)
        self.send_button = ttk.Button(arduino_frame,text="Send to Arduino",command=self.send_to_arduino)
        self.send_button.grid(row=2, column=0, columnspan=2, sticky="ew", pady=5)

        # Relax / Hold buttons
        self.free_button = ttk.Button(arduino_frame,text="Relax motors",command=self.free_motors)
        self.free_button.grid(row=3, column=0, columnspan=2, sticky="ew", pady=3)

        self.hold_button = ttk.Button(arduino_frame,text="Hold motors",command=self.hold_motors)
        self.hold_button.grid(row=4, column=0, columnspan=2, sticky="ew", pady=3)

        # Jog controls (direct joint-space jogging)
        jog_frame = ttk.LabelFrame(arduino_frame, text="Jog motors", padding=5)
        jog_frame.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(5, 0))

        # Labels
        ttk.Label(jog_frame, text="Motor 1 (left/right):").grid(row=0, column=0, sticky="w")
        ttk.Label(jog_frame, text="Motor 2 (left/right):").grid(row=2, column=0, sticky="w")

        ttk.Button(jog_frame,text="M1 -5 deg",command=lambda: self.jog_motor(-5.0, 0.0)).grid(row=1, column=0, sticky="ew", pady=2)

        ttk.Button(jog_frame,text="M1 +5 deg",command=lambda: self.jog_motor(+5.0, 0.0)).grid(row=1, column=1, sticky="ew", pady=2)

        ttk.Button(jog_frame,text="M2 -5 deg",command=lambda: self.jog_motor(0.0, -5.0)).grid(row=3, column=0, sticky="ew", pady=2)

        ttk.Button(jog_frame,text="M2 +5 deg",command=lambda: self.jog_motor(0.0, +5.0)).grid(row=3, column=1, sticky="ew", pady=2)

        ttk.Button(jog_frame,text="Reset jog (0 deg,0 deg)",command=self.reset_jog).grid(row=4, column=0, columnspan=2, sticky="ew", pady=2)


        # RIGHT: Plot preview and simulation
        plot_frame = ttk.LabelFrame(main_frame, text="Preview / Simulation", padding=10)
        plot_frame.grid(row=0, column=1, padx=(10, 0), sticky="nsew")

        self.fig = plt.Figure(figsize=(5, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Please generate Path")
        self.ax.set_xlabel("x (mm)")
        self.ax.set_ylabel("y (mm)")
        self.ax.grid(True)
        self.ax.set_aspect("equal", "box")

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        #mouse clicks on the plot
        self.canvas.mpl_connect("button_press_event", self.on_canvas_click)




    # activate callbacks
    def free_motors(self):
        # Send FREE to Arduino so the motors relax
        ard = self.get_arduino()
        if ard is None:
            return
        ard.send_data("FREE")
        self.status_label.config(text="Status: Motors relaxed (FREE)")
        print("[ARD] Sent FREE")

    def hold_motors(self):
        # Send HOLD to Arduino to actiavte motors PID
        ard = self.get_arduino()
        if ard is None:
            return
        ard.send_data("HOLD")
        self.status_label.config(text="Status: Motors holding (HOLD)")
        print("[ARD] Sent HOLD")

    def jog_motor(self, d1_deg, d2_deg):
        # used to manually jog the motors to desired poisition without IK
        ard = self.get_arduino()
        if ard is None:
            return
        # update local state
        self.jog_t1 += d1_deg
        self.jog_t2 += d2_deg

        # absolute angles in degrees
        line = f"{self.jog_t1:.2f},{self.jog_t2:.2f}"
        print(f"[JOG] Sending: {line}")
        ard.send_data(line)

        self.status_label.config(
        text=f"Status: Jog -> M1={self.jog_t1:.1f} deg, M2={self.jog_t2:.1f} deg")


    def reset_jog(self):
        # Reset the jog state to (0 deg,0 deg) and send that as a demand.
       
        ard = self.get_arduino()
        if ard is None:
            return

        self.jog_t1 = 0.0
        self.jog_t2 = 0.0

        line = "0.00,0.00"
        print("[JOG] Reset jog to 0,0")
        ard.send_data(line)

        self.status_label.config(text="Status: Jog reset to (0 deg,0 deg)")


    def load_image(self):
        # Let the user pick an image file
        file_path = filedialog.askopenfilename(title="Select image",filetypes=[("Image files", "*.png;*.jpg;*.jpeg;*.bmp;*.gif"),("All files", "*.*"),],)
        if not file_path:
            return

        self.image_path = file_path
        file_name = os.path.basename(file_path)
        self.image_label.config(text=f"Selected: {file_name}")
        self.status_label.config(text="Status: Image loaded")
        print(f"[INFO] Image selected: {file_path}")

    def generate_ai_image_ui(self):
        """Generate an AI image using OpenAI and feed it directly into the pipeline."""
        prompt = self.ai_prompt_var.get().strip()
        if not prompt:
            messagebox.showwarning("No prompt", "Please enter a prompt first.")
            return

        self.status_label.config(text="Status: Generating AI image, please be patient as this could take a while...")
        self.root.update_idletasks()
        print(f"[AI] Generating image for prompt: {prompt!r}")

        try:
            img_path = generate_ai_image(prompt, output_path="ai_output.png")

            # Update GUI state
            self.image_path = img_path
            file_name = os.path.basename(img_path)
            self.image_label.config(text=f"AI image: {file_name}")
            self.status_label.config(text="Status: AI image ready (click Generate Path)")
            print(f"[AI] Image saved as {img_path}")

        except Exception as e:
            self.status_label.config(text="Status: AI generation failed")
            messagebox.showerror("AI Error", f"Failed to generate AI image:\n{e}")
            print(f"[ERROR] {e}")

    def generate_path(self):
        # Run the image -> path -> joint angles pipeline and update the UI.
        
        if not self.image_path:
            messagebox.showwarning("No image","Please load or generate an image first.")
            return

        self.status_label.config(text="Status: Generating path...")
        self.root.update_idletasks()

        res = self.resolution_var.get()
        path, t_est = generate_drawing_from_image(self.image_path,resolution=res,)

        #if no path  adedd
        if not path:
            self.path = []
            self.angles_deg = []
            self.points_label.config(text="Points: 0")
            self.time_label.config(text="Estimated time: - s")
            self.status_label.config(text="Status: No valid path")
            self.ax.clear()
            self.ax.set_title("No valid path (check image / workspace)")
            self.ax.grid(True)
            self.canvas.draw()
            print("[WARN] No valid path generated from image.")
            return

        #reassign the matrices and variables
        self.path = path
        self.estimated_time = t_est
        self.angles_deg = path_to_joint_angles_deg(path)

        self.points_label.config(text=f"Points: {len(path)}")
        self.time_label.config(text=f"Estimated time: {t_est:.1f} s")
        self.status_label.config(text="Status: Path generated")
        print(f"[INFO] Path generated with {len(path)} points.")

        # Basic preview: just the stroke
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]

        self.ax.clear()
        self.ax.plot(xs, ys, linewidth=0.6)
        self.ax.set_aspect("equal", "box")
        self.ax.set_title("Predicted Drawing")
        self.ax.set_xlabel("x (mm)")
        self.ax.set_ylabel("y (mm)")
        self.ax.grid(True)
        self.canvas.draw()

    def show_workspace(self):
        """
        Visualise reachable workspace in the preview panel,
        using the same inside_workspace() check as the backend.
        """
        self.status_label.config(text="Status: Computing workspace...")
        self.root.update_idletasks()

        xs, ys = [], []
        step = 4  # coarse enough to be quick, fine enough to see shape

        for x in np.arange(-100, 100 + step, step):
            for y in np.arange(0, 150 + step, step):
                if inside_workspace(x, y):
                    xs.append(x)
                    ys.append(y)

        self.ax.clear()
        if xs:
            self.ax.scatter(xs, ys, s=5)
            self.ax.set_title("Reachable Workspace of the Robot")
        else:
            self.ax.set_title("Workspace empty (check kinematics params)")

        self.ax.set_xlabel("x (mm)")
        self.ax.set_ylabel("y (mm)")
        self.ax.set_aspect("equal", "box")
        self.ax.grid(True)
        self.canvas.draw()

        self.status_label.config(text="Status: Workspace displayed")
        print(f"[INFO] Workspace plotted with {len(xs)} points.")

    def simulate_movement(self):
        """
        show the robot arms moving along the generated path using the same IK/FK as the backend.
        This is purely virtual, useful to prove maths/theory even if hardware breaks.
        """
        if not self.path:
            messagebox.showwarning("No path","Generate a path before running the simulation.")
            return

        # Use continuous IK along the path for smoother arm animation
        angles = path_to_joint_angles(self.path)
        if not angles:
            messagebox.showwarning("No IK","Could not compute joint angles for this path.")
            return

        l0 = mid_distance / 2.0
        base_left = (-l0, 0.0)
        base_right = (l0, 0.0)

        self.status_label.config(text="Status: Simulating movement...")
        self.root.update_idletasks()

        drawn_x = []
        drawn_y = []

        # animation loop 
        for t1, t2 in angles:
            px, py, _, _ = forward_kinematics(t1, t2)
            if px is None:
                continue

            # elbow positions
            x1 = -l0 + back_length * math.cos(t1)
            y1 = back_length * math.sin(t1)
            x2 = l0 + back_length * math.cos(t2)
            y2 = back_length * math.sin(t2)

            drawn_x.append(px)
            drawn_y.append(py)

            self.ax.clear()
            self.ax.set_xlim(-100, 100)
            self.ax.set_ylim(150, 0)
            self.ax.set_aspect("equal", "box")
            self.ax.grid(True)
            self.ax.set_xlabel("x (mm)")
            self.ax.set_ylabel("y (mm)")
            self.ax.set_title("Simulation: Robot Motion")

            # plot bases
            self.ax.plot([base_left[0], x1, px], [base_left[1], y1, py], "-o", linewidth=1)
            self.ax.plot([base_right[0], x2, px], [base_right[1], y2, py], "-o", linewidth=1)

            # traced stroke
            if len(drawn_x) > 1:
                self.ax.plot(drawn_x, drawn_y, "k-", linewidth=0.7) #change as required

            self.canvas.draw()
            self.root.update_idletasks()
            self.root.update()

            time.sleep(0.02)  # <-change for speed of simulation

        self.status_label.config(text="Status: Simulation complete")
        print("[INFO] Simulation finished.")

    def send_to_arduino(self):
        """
        Send the generated joint trajectory to Arduino via serial.

        Uses back_end.upload_trajectory_and_run(), which:
        - connects
        - sends TRAJ N + N angle pairs
        - waits for READY
        - sends RUN
        - waits for DONE
        """
        if not self.angles_deg:
            messagebox.showwarning("No trajectory","Generate a path before sending to Arduino.",)
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("No port","Please enter a valid COM port (e.g. COM7 / COM13).",)
            return

        ard = self.get_arduino()
        if ard is None:
            return

        self.status_label.config(text=f"Status: Sending to {port}...")
        self.root.update_idletasks()
        print(f"[INFO] Uploading trajectory to Arduino on {port}..")

        try:
            send_trajectory_to_arduino_streaming(self.angles_deg,ard=ard,dt=1.0,)

            self.status_label.config(text="Status: Trajectory finished")
            print("[INFO] Trajectory upload & run complete.")
        except Exception as e:
            self.status_label.config(text="Status: Error ")
            print(f"[ERROR] Failed to send trajectory: {e}")
            messagebox.showerror("Arduino Error",f"Could not send trajectory:\n{e}",)


    def home_robot(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("No port", "Please enter a valid COM port.")
            return

        # Compute IK for home point
        t1, t2 = inverse_kinematics(HOME_X, HOME_Y)
        if t1 is None or t2 is None:
            messagebox.showerror("IK error", "Home position is not reachable.")
            return

        # motor mapping 
        t1_deg = math.degrees(t1)
        t2_deg = math.degrees(t2)
        deg_m1, deg_m2 = map_ik_to_hardware(t1_deg, t2_deg)
        angles_deg = [(deg_m1, deg_m2)]

        ard = self.get_arduino()
        if ard is None:
            return

        self.status_label.config(text="Status: Moving to Home...")
        self.root.update_idletasks()
        print(f"[INFO] Homing to ({HOME_X}, {HOME_Y}) -> M1={deg_m1:.2f}, M2={deg_m2:.2f}")

        send_trajectory_to_arduino_streaming(angles_deg,port=port,baudrate=115200,dt=1.0,ard=ard)

        self.status_label.config(text="Status: At Home")


    def on_canvas_click(self, event):
        """
        If click-to-move is enabled, clicking on the plot
        sends the robot to that (x, y) point.
        """
        if not self.click_move_enabled.get():
            return  # feature turned off

        if event.inaxes != self.ax:
            return  # click outside plot

        x = event.xdata
        y = event.ydata
        if x is None or y is None:
            return

        print(f"[CLICK] ({x:.1f}, {y:.1f})")

        if not inside_workspace(x, y):
            print("[WARN] Click outside reachable workspace")
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("No port", "Please enter a valid COM port.")
            return

        t1, t2 = inverse_kinematics(x, y)
        if t1 is None or t2 is None:
            print("[WARN] IK failed for clicked point")
            return

        t1_deg = math.degrees(t1)
        t2_deg = math.degrees(t2)
        deg_m1, deg_m2 = map_ik_to_hardware(t1_deg, t2_deg)
        angles_deg = [(deg_m1, deg_m2)]

        ard = self.get_arduino()
        if ard is None:
                return

        self.status_label.config(text=f"Status: Moving to ({x:.1f}, {y:.1f})")
        self.root.update_idletasks()

        send_trajectory_to_arduino_streaming(angles_deg,port=port,baudrate=115200,dt=1.0,ard = ard)

        self.status_label.config(text="Status: Idle (click-to-move)")

    def capture_calibration_point(self):
        """
        Calibration procedure (multi-press):

        FIRST PRESS:
          - Start calibration
          - Relax motors (FREE) so you can move arms by hand
          - Reset stored points

        NEXT 3 PRESSES:
          - Briefly HOLD to stabilise
          - Query current angles (Q)
          - Convert to (x_meas, y_meas) via FK
          - Store (ideal -> measured) pair
          - FREE again so you can move to the next corner

        Ideal points (must match generate_drawing_from_image()):
          1: (x_min, y_min)  bottom-left
          2: (x_max, y_min)  bottom-right
          3: (x_min, y_max)  top-left
        """
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("No port", "Set the COM port first.")
            return

        ard = self.get_arduino()
        if ard is None:
            return

        # First press: start calibration, relax motors
        if not self.cal_in_progress:
            self.cal_in_progress = True
            self.cal_index = 0
            self.cal_ideal = []
            self.cal_measured = []

            # These must match generate_drawing_from_image
            self.cal_x_min, self.cal_x_max = -40, 40
            self.cal_y_min, self.cal_y_max = 60, 140

            # Relax motors so you can move the arms
            ard.send_data("FREE")
            self.cal_status_label.config(text="Calibration started: 0 / 3 points")
            self.status_label.config(
            text="Status: Motors FREE - move to bottom-left corner, then press again."
            )

            print("[CAL] Calibration started, motors FREE")
            return

        # If we are here, calibration is already in progress
        if self.cal_index >= 3:
            messagebox.showinfo("Calibration", "Already captured 3 points.")
            return

        # Briefly HOLD to stabilise pose for measurement
        ard.send_data("HOLD")
        time.sleep(0.2)

        # Ideal points, in order
        ideal_points = [
            (self.cal_x_min, self.cal_y_min),  # bottom-left
            (self.cal_x_max, self.cal_y_min),  # bottom-right
            (self.cal_x_min, self.cal_y_max),  # top-left
        ]

        self.status_label.config(text=f"Status: Reading pose for calibration point {self.cal_index+1}...")
        self.root.update_idletasks()

        # Ask Arduino for current angles
        res = request_current_angles(port=port, baudrate=115200, ard=ard)
        if res is None:
            self.status_label.config(text="Status: Calibration failed (no pose)")
            messagebox.showerror("Calibration", "Could not read pose from Arduino.")
            return

        t1_deg, t2_deg = res
        t1 = math.radians(t1_deg)
        t2 = math.radians(t2_deg)

        # Compute robot Cartesian coordinates from FK
        x_meas, y_meas, _, _ = forward_kinematics(t1, t2)
        if x_meas is None:
            self.status_label.config(text="Status: Calibration FK failed")
            messagebox.showerror("Calibration", "Forward kinematics failed.")
            return

        # Store point pair
        self.cal_ideal.append(ideal_points[self.cal_index])
        self.cal_measured.append((x_meas, y_meas))
        self.cal_index += 1

        self.cal_status_label.config(text=f"{self.cal_index} / 3 points captured")
        print(f"[CAL] Captured point {self.cal_index}: ideal={self.cal_ideal[-1]}, measured=({x_meas:.2f},{y_meas:.2f})")

        # If not done yet: FREE again to move to next corner
        if self.cal_index < 3:
            ard.send_data("FREE")
            self.status_label.config(
            text=f"Status: Point {self.cal_index} captured. Motors FREE - move to next corner and press again.")   

            return

        # After 3 points, compute matrix and finish
        set_calibration(self.cal_ideal, self.cal_measured)
        self.cal_in_progress = False

        # You can choose whether to keep HOLD or FREE here; I'll set HOLD
        ard.send_data("HOLD")

        self.status_label.config(text="Status: Calibration complete (HOLD)")
        messagebox.showinfo(
            "Calibration",
            "3 points captured. Calibration matrix computed.\n"
            "New drawings will now use this calibration."
        )






if __name__ == "__main__":
    root = tk.Tk()
    app = PlotterUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
