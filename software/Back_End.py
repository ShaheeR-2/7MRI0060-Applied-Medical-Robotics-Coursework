import math
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import time
import serial

CAL_MATRIX = None

DRAW_X_MIN = -40
DRAW_X_MAX =  40
DRAW_Y_MIN =  70
DRAW_Y_MAX = 150


SWAP_MOTORS = False   # True if Arduino motor 1 is physically on the RIGHT
M1_SIGN = 1.0        # flip to -1.0 if motor 1 moves opposite 
M2_SIGN = 1.0        # flip to -1.0 if motor 2 moves opposite
M1_OFFSET_DEG = -139.25  # changed according to offset calculations
M2_OFFSET_DEG = -40.75


def map_ik_to_hardware(t1_deg, t2_deg):
    """
    t1_deg, t2_deg are angles from inverse_kinematics (LEFT, RIGHT).
    Returns (deg_m1, deg_m2) for Arduino.
    """
    # Optionally swap left/right
    if SWAP_MOTORS:
        t1_deg, t2_deg = t2_deg, t1_deg

    # Apply sign and offsets for each motor
    deg_m1 = M1_SIGN * t1_deg + M1_OFFSET_DEG
    deg_m2 = M2_SIGN * t2_deg + M2_OFFSET_DEG
    return deg_m1, deg_m2


class ArduinoCommunication:
    """
    Seriial Communication with the arduino
    Format of communicaton: "<theta1_deg>,<theta2_deg>\\n"
    """

    def __init__(self, port="COM13", baudrate=115200, timeout=0.1):
        try:
            self.arduino = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(2)  # Allow for connection (Arduino auto-reset)
            self.arduino.reset_input_buffer()
            self.connected = True
            print(f"Arduino connected on {port} at {baudrate} baud")
        except Exception as e:
            print(f"Failed to connect to Arduino: {str(e)}")
            self.connected = False
            self.arduino = None

    def send_data(self, data: str) -> bool:
     
        #function to send data to the arduino in correct format: "100.0,80.0"
   
        if self.connected and self.arduino and self.arduino.is_open: #ensure connection
            try:
                formatted_data = str(data) + "\n"
                self.arduino.write(formatted_data.encode("utf-8"))
                self.arduino.flush()
                return True
            except Exception as e:
                print(f"Send failed: {e}")
                return False
        return False

    def read_data(self):
        """Read one line from Arduino (non-blocking)."""
        if not (self.connected and self.arduino and self.arduino.is_open):
            return None

        try:
            if self.arduino.in_waiting > 0:
                line = self.arduino.readline()
                return line.decode("utf-8", errors="ignore").strip()
        except Exception as e:
            print(f"Read failed: {e}")
            return None

        return None

    def close(self):
        #Closes the connection
        if self.connected and self.arduino and self.arduino.is_open:
            self.arduino.close()
            print("Arduino connection closed")
            self.connected = False


# Robot parameters  (change according to the build)
mid_distance = 50           # distancce between the 2 motor anchors
back_length = 80           # lenght of each base arm (connected to motor)
fore_length = 90            # length of each fore arm (connected to pen)

# Shows the midpoint base point of the base
HOME_X = 0.0   #change to DRAW_X_MIN 
HOME_Y = 80.0 #change to DRAW_Y_MIN 


def forward_kinematics(theta1, theta2, mid_dist=mid_distance, l1=back_length, l2=fore_length):
    """
    Parameters:
    theta1: left active joint angle (radians)
    theta2: right active joint angle (radians)
    l0: Distance between the two motor anchors (mm)
    l1: length of each base arm (mm)
    l2: length of each fore arm (mm)

    Returns:
    (px, py): end effector coordinates (mm)
    (theta_p1, theta_p2): joint angles (radians) for left and right forearms
    or (None, None, None, None) if impossible

    Method:
    1. Compute elbow positions from base + L1 + angle
    2. Intersect two circles of radius L2 around each elbow
    3. Choose the intersection that represents the drawing pen

    Maths is show in the documentation
    """

    #distance from origin to each motor
    l0 = mid_dist / 2.0

    #Left elbow position
    x1 = -l0 + l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)

    #Right elbow position
    x2 =  l0 + l1 * np.cos(theta2)
    y2 =  l1 * np.sin(theta2)

    #Distance between elbows
    dx = x2 - x1
    dy = y2 - y1
    d = np.hypot(dx,dy)

    # Check if forearms can join
    if d == 0 or d > 2.0 * l2:
        return None, None, None, None

    # Midpoint between elbows
    mid_x = (x1 + x2) / 2.0
    mid_y = (y1 + y2) / 2.0

    #Distance from midpoint to pen (circle-circle intersection geometry)
    h = np.sqrt(l2**2 - (d / 2.0)**2)

    #Unit perpendicular to elbow-elbow line
    ux = -dy / d
    uy =  dx / d

    #Choose one of the two possible intersections.
    end_x = mid_x + ux * h
    end_y = mid_y + uy * h

    #Passive joint directions (elbow -> pen) minus active arm directions
    theta_p1 = np.arctan2(end_y - y1, end_x - x1) - theta1
    theta_p2 = np.arctan2(end_y - y2, end_x - x2) - theta2

    return end_x, end_y, theta_p1, theta_p2


def _ik_single_side(base_x, base_y, px, py,
                    l1=back_length,
                    l2=fore_length,
                    elbow_up=True):
    """
    Inverse kinematics for ONE side of the five-bar.

    Find the active joint angle theta such that:
      - distance(base -> elbow) = l1
      - distance(elbow -> pen) = l2

    This is solved as a circle-circle intersection:
      circle 1: center at base, radius l1
      circle 2: center at pen,  radius l2
    """

    dx = px - base_x
    dy = py - base_y
    d = math.hypot(dx, dy)

    # Triangle inequality for existence
    if d == 0 or d > (l1 + l2) or d < abs(l1 - l2):
        return None

    # Distance from base to pen 
    a = (l1**2 - l2**2 + d**2) / (2.0 * d)

    # Height from that line to the actual elbow
    h_sq = l1**2 - a**2
    if h_sq < 0:
        return None
    h = math.sqrt(h_sq)

    # Point along the line from base to pen
    mx = base_x + (a / d) * dx
    my = base_y + (a / d) * dy

    # Perpendicular direction
    px_off = -dy / d
    py_off =  dx / d

    if elbow_up:
        ex = mx + px_off * h
        ey = my + py_off * h
    else:
        ex = mx - px_off * h
        ey = my - py_off * h

    # Active joint angle is direction from base to elbow
    theta = math.atan2(ey - base_y, ex - base_x)
    return theta


def inverse_kinematics(px, py, mid_dist=mid_distance, l1=back_length,l2=fore_length, elbow_up=True):
    """
    Inverse kinematics for the dual-arm (five-bar) planar robot.

    Given a desired pen position (px, py), compute the two active
    joint angles (theta1, theta2).

    Each side is solved as a circle-circle intersection between:
      - base (radius l1)
      - pen position (radius l2)

    UPDATED RULE:
      - We ONLY allow configurations where one elbow is "up" and
        the other is "down" (opposite), to avoid the arms folding
        through each other.
      - We test:
            Case A: left elbow up,   right elbow down
            Case B: left elbow down, right elbow up
      - Any candidate must also pass is_valid_configuration()
        (no base collision, no elbow overlap, etc).
      - If both are valid, we pick the one with smaller
        |theta1| + |theta2| (a "simpler" posture).

    Returns:
        theta1, theta2 (radians), or (None, None) if unreachable
        or if both opposite-elbow configs are invalid.
    """

    l0 = mid_dist / 2.0
    candidates = []

    def check_candidate(t1, t2):
        if t1 is None or t2 is None:
            return None

        # Check geometry via FK
        px2, py2, _, _ = forward_kinematics(t1, t2, mid_dist=mid_dist, l1=l1, l2=l2)
        if px2 is None or py2 is None:
            return None

        err = math.hypot(px2 - px, py2 - py)
        if err > 1e-3:   # tolerance in mm
            return None

        # run  through safety constraints
        if not is_valid_configuration(t1, t2, mid_dist=mid_dist, l1=l1, l2=l2):
            return None

        return (t1, t2)

    # Case A: left elbow up, right elbow down
    theta1_A = _ik_single_side(-l0, 0.0, px, py, l1=l1, l2=l2, elbow_up=True)
    theta2_A = _ik_single_side( l0, 0.0, px, py, l1=l1, l2=l2, elbow_up=False)
    candA = check_candidate(theta1_A, theta2_A)
    if candA: candidates.append(candA)

    # Case B: left elbow down, right elbow up
    theta1_B = _ik_single_side(-l0, 0.0, px, py, l1=l1, l2=l2, elbow_up=False)
    theta2_B = _ik_single_side( l0, 0.0, px, py, l1=l1, l2=l2, elbow_up=True)
    candB = check_candidate(theta1_B, theta2_B)
    if candB: candidates.append(candB)

    if not candidates:
        return None, None

    best_theta1, best_theta2 = min(candidates, key=lambda th: (abs(th[0]) + abs(th[1]))) #finds minimum distance to next clostest angle from valid configs
    return best_theta1, best_theta2

def is_valid_configuration(theta1,theta2,mid_dist=mid_distance,l1=back_length,l2=fore_length,base_clear_y=5.0,pen_clear_y=10.0,elbow_clear_x=5.0,elbow_min_dist=10.0):
    """
    safety filter for the 5-bar:

    Reject any configuration that:
      - has elbows or pen too close to or below the base line
      - has the two elbows cross each other
      - has the elbows swapped crossing (left elbow must stay left-ish,
        right elbow right-ish)
    Tune the magic numbers (clearances) empirically on the real hardware.
    """

    # half distance between bases
    l0 = mid_dist / 2.0

    # base positions
    baseL = (-l0, 0.0)
    baseR = ( l0, 0.0)

    # elbow positions from active joints
    e1_x = baseL[0] + l1 * math.cos(theta1)
    e1_y = baseL[1] + l1 * math.sin(theta1)

    e2_x = baseR[0] + l1 * math.cos(theta2)
    e2_y = baseR[1] + l1 * math.sin(theta2)

    # uses FK to check reachability
    px, py, _, _ = forward_kinematics(theta1, theta2, mid_dist=mid_dist, l1=l1,l2=l2)

    if px is None or py is None:
        # FK says "nope"
        return False

    #1) keep everything above the base
    # elbows must be above some clearance above y=0 (base line)
    if e1_y < base_clear_y or e2_y < base_clear_y:
        return False

    # pen should also not scrape the base
    if py < pen_clear_y:
        return False

    #2) avoid elbow overlap 
    dx_e = e2_x - e1_x
    dy_e = e2_y - e1_y
    dist_elbows = math.hypot(dx_e, dy_e)
    if dist_elbows < elbow_min_dist:
        return False

    #3) avoid arms crossing through each other
    # left elbow should stay to the left of right elbow with some margin.
    if not (e1_x < e2_x - elbow_clear_x):
        return False

    return True

def inside_workspace(px, py):
    #checks if the point lies within the robot's reachable workspace and inside contrraints
    theta1, theta2 = inverse_kinematics(px, py)
    return theta1 is not None and theta2 is not None


def test_ik_fk():
    """
    To validate the inverse kinematics for the points on the workspace,
    the obtained joint angles are substituted back into the forward kinematics function.
    This confirms the reconstruction of the target position with numerical validation
    """
    # pick a few sample target points inside the workspace
    test_points = [(0, 40),(10, 50),(-10, 50),(0, 60),]

    for (px, py) in test_points:
        theta_l, theta_r = inverse_kinematics(px, py)
        if theta_l is None:
            print(f"Target ({px}, {py}) unreachable")
            continue

        end_x, end_y, _, _ = forward_kinematics(theta_l, theta_r)
        if end_x is None:
            print(f"FK failed for IK solution at ({px}, {py})")
            continue

        error = math.hypot(end_x - px, end_y - py)
        print(f"Target ({px:.1f}, {py:.1f}) -> Reconstructed ({end_x:.2f}, {end_y:.2f}), error = {error:.4f} mm")


def plot_workspace(step=2):
    """
     Plot the reachable workspace of the 2-DOF parallel arm robot.
     Parameters: The step size between sampled grid points (in mm).
     Method:
    1. Define a rectangular region to sample (x, y) points across.
    2. For each coordinate:
        - Run inverse_kinematics() to determine if both arms can reach it.
        - If valid, add the point to the list of reachable positions.
    3. Plot all valid points using matplotlib to visualise the reachable region.
    """
    xs, ys = [], [] #Lists to store reachable (x, y) points

    #Define the range of x and y coordinates to test (mm) and checks whether in workspace
    for x in np.arange(-100, 100 + step, step):
        for y in np.arange(0, 150 + step, step):
            if inside_workspace(x, y):
                xs.append(x)
                ys.append(y)

    #Plot all reachable points and annotate plot
    plt.figure()
    plt.scatter(xs, ys, s=5)
    plt.gca().set_aspect('equal', 'box')
    plt.title("Reachable Workspace of the 2-DOF Parallel Plotter")
    plt.xlabel("x (mm)")
    plt.ylabel("y (mm)")
    plt.show()


def generate_drawing_from_image(image_path, resolution="medium", threshold=128, x_min=-40, x_max=40, y_min=60, y_max=140):
    """
    Complete pipeline for converting an input image into a continuous
    single-stroke path suitable for the 2-DOF parallel arm plotter.

    Steps:
    1. Load the image and convert it to greyscale (0 - 255 intensity)
    2. Resize the image according to the selected resolution mode: (subject to change)
         - low   -> 40 x 40
         - medium -> 60 x 60
         - high   -> 80 x 80
    3. Threshold the greyscale image to produce a boolean mask
       (True = draw, False = skip).
    4. Map each active pixel (True) to a physical (x, y) coordinate
       in the robots workspace, scaled to 80 by 80 mm
    5. Filter out unreachable points using the inverse kinematics.
    6. Apply a nearest-neighbour algorithm to order all points 
       into a single continuous stroke (since the pen cannot lift).
    7. Return both the ordered path and the estimated drawing time.
    """

    # Step 1: load and resize
    if resolution == "low":
        size = 40
    elif resolution == "high":
        size = 80
    else:
        size = 60

    img = Image.open(image_path).convert("L")  # greyscale
    img = img.resize((size, size))
    grey = np.array(img)

    # Step 2: Threshold mask to boolean
    mask = grey < threshold

    # Step 3: Map mask pixels to workspace
    rows, cols = mask.shape
    points = []
    for r in range(rows):
        for c in range(cols):
            if not mask[r, c]:
                continue
            # Normal coordinates in [0,1]
            u = c / (cols - 1)
            v = r / (rows - 1)
            # Map to workspace (mm)
            X = x_min + u * (x_max - x_min)
            Y = y_max - v * (y_max - y_min)
            x, y = apply_calibration(X, Y)
            # Keep only valid coordinates
            if inside_workspace(x, y):
                points.append((x, y))

    # Step 4: Nearest-Neighbour single stroke

    # if no valid points return empty workspace
    if not points:
        return [], 0.0

    # Used to pick the start point, this will be the lowest and leftmost point on the image(if there are t)
    start_idx = min(range(len(points)), key=lambda i: (points[i][1], points[i][0]))

    # Initialise the path with that first point.
    path = [points[start_idx]]

    # Keep track of which points have been visited.
    used = {start_idx}
    current_idx = start_idx

    """
    Step 5:
    Loop function for all points in the matrix
    At each step, it looks for the closest unvisited point using euclidean distance
    Ensure pen always moves to the closest remaining point, minimising travel
    also keeps stroke continuous
    """

    for _ in range(len(points) - 1):
        cx, cy = points[current_idx]    # current pen position

        # min function finds min remaining point distance
        next_idx = min((i for i in range(len(points)) if i not in used),key=lambda j: (points[j][0] - cx)**2 + (points[j][1] - cy)**2)
        path.append(points[next_idx])
        used.add(next_idx)
        current_idx = next_idx

    if not path:
        return [], 0.0

    total_length = sum(math.hypot(path[i+1][0] - path[i][0],path[i+1][1] - path[i][1])for i in range(len(path) - 1))

    pen_speed = 20.0  # mm/s (measure and tune this value)
    estimated_time = total_length / pen_speed

    return path, estimated_time


def path_to_joint_angles(path):
    """
    Convert a list of (x, y) points into corresponding
    (theta1, theta2) joint angles using inverse kinematics.
    """
    
    angles = []
    for (x, y) in path:
        theta1, theta2 = inverse_kinematics(x, y)
        if theta1 is not None and theta2 is not None:
            angles.append((theta1, theta2))
    return angles

def angles_difference(path):
    angles = path_to_joint_angles(path)
    if not angles:
        return []

    angles_new = []
    # different is zero
    angles_new.append(angles[0])
    for i in range(1, len(angles)):
        d1 = angles[i][0] - angles[i - 1][0]
        d2 = angles[i][1] - angles[i - 1][1]
        angles_new.append((d1, d2))

    return angles_new 

def path_to_joint_angles_deg(path):
    """
    Convert a path into hardware joint angles (deg_m1, deg_m2)
    suitable for sending directly to arduino
    """
    angles_deg = []
    for (x, y) in path:
        theta1, theta2 = inverse_kinematics(x, y)
        if theta1 is None or theta2 is None:
            continue
        t1_deg = math.degrees(theta1)
        t2_deg = math.degrees(theta2)
        deg_m1, deg_m2 = map_ik_to_hardware(t1_deg, t2_deg)
        angles_deg.append((deg_m1, deg_m2))
    return angles_deg



def test_arduino_connection(port="COM13", baudrate=115200):
    """
    Quick check to see if Arduino is reachable on the given port.
    """
    print(f"[TEST] Checking Arduino on {port}...")
    ard = ArduinoCommunication(port=port, baudrate=baudrate)
    if ard.connected:
        print("[TEST] Arduino connection SUCCESS.")
        ard.close()
    else:
        print("[TEST] Arduino connection FAILED.")


def send_trajectory_to_arduino_streaming(angles_deg, port="COM13", baudrate=115200, dt=1.0, ard=None):
    """
    Send points one by one with handshake:
      - send 'theta1,theta2\n'
      - wait for 'OK'
      - sleep dt
    """
    owns_connection = False
    if ard is None:     
        ard = ArduinoCommunication(port=port, baudrate=baudrate, timeout=0.1)
        if not ard.connected:
            print(f"[ERROR] Could not connect to Arduino on {port}")
            return
        owns_connection = True

    print(f"[OK] Streaming {len(angles_deg)} points to {port} @ {baudrate} (dt={dt:.3f}s)")

    try:
        # wait for Arduino's READY
        start = time.time()
        while time.time() - start < 3.0:
            resp = ard.read_data()
            if resp:
                print(f"[Arduino] {resp}")
                if "READY" in resp:
                    break
            time.sleep(0.01)

        # Send points one-by-one
        for i, (t1, t2) in enumerate(angles_deg):
            line = f"{t1:.2f},{t2:.2f}"
            print(f"[TX {i}] {line}")
            ard.send_data(line)

            # Wait for OK
            t0 = time.time()
            got_ok = False
            while time.time() - t0 < 1.0:  # 1 s timeout per point
                resp = ard.read_data()
                if resp:
                    print(f"[Arduino] {resp}")
                    if "OK" in resp:
                        got_ok = True
                        break
                time.sleep(0.005)

            if not got_ok:
                print(f"[WARN] No OK for point {i}, continuing anyway")

            time.sleep(dt)

    finally:
        if owns_connection:
            ard.close()
            print("[INFO] Streaming complete (temp connection closed).")


def set_calibration(ideal_pts, measured_pts):
    """
    ideal_pts: list of 3 (X, Y) in 'drawing space' (mm)
    measured_pts: list of 3 (x, y) from FK of encoder angles

    Solves for 2x3 affine matrix A such that:
        [x]   [a11 a12 a13] [X]
        [y] = [a21 a22 a23] [Y]
      """
    global CAL_MATRIX
    if len(ideal_pts) != 3 or len(measured_pts) != 3:
        raise ValueError("Need exactly 3 calibration points")

    A = []
    b = []
    for (X, Y), (xr, yr) in zip(ideal_pts, measured_pts):
        # x equation
        A.append([X, Y, 1, 0, 0, 0])
        b.append(xr)
        # y equation
        A.append([0, 0, 0, X, Y, 1])
        b.append(yr)

    A = np.array(A, dtype=float)
    b = np.array(b, dtype=float)

    params = np.linalg.solve(A, b)  # [a11,a12,a13,a21,a22,a23]
    CAL_MATRIX = params.reshape(2, 3)
    print("[CAL] New calibration matrix:")
    print(CAL_MATRIX)

def calibrate_home_offsets():
    """
    Align IK home (HOME_X, HOME_Y) with hardware (0, 0).
     After this, map_ik_to_hardware(inverse_kinematics(HOME_X, HOME_Y)) will return (0, 0).
    """
    global M1_OFFSET_DEG, M2_OFFSET_DEG

    t1, t2 = inverse_kinematics(HOME_X, HOME_Y)
    if t1 is None or t2 is None:
        raise RuntimeError("IK failed for HOME point")

    t1_deg = math.degrees(t1)
    t2_deg = math.degrees(t2)
    print("[CAL-HOME] IK home angles (deg):", t1_deg, t2_deg)

    M1_OFFSET_DEG = -M1_SIGN * t1_deg
    M2_OFFSET_DEG = -M2_SIGN * t2_deg

    print("[CAL-HOME] New offsets:","M1_OFFSET_DEG =", M1_OFFSET_DEG,"M2_OFFSET_DEG =", M2_OFFSET_DEG)

    d1, d2 = map_ik_to_hardware(t1_deg, t2_deg)
    print("[CAL-HOME] map_ik_to_hardware(home) ->", d1, d2)


def apply_calibration(X, Y):
    """
    Map ideal drawing coords (X,Y) to robot coords (x,y)
    using the current affine calibration.
   If no calibration, return (X,Y).
    """
    if CAL_MATRIX is None:
        return X, Y

    a11, a12, a13 = CAL_MATRIX[0]
    a21, a22, a23 = CAL_MATRIX[1]
    xr = a11 * X + a12 * Y + a13
    yr = a21 * X + a22 * Y + a23
    return xr, yr


if __name__ == "__main__":

    calibrate_home_offsets()

    # 0. Test Arduino connection first
    test_arduino_connection(port="COM7", baudrate=115200)

    print("Plotting workspace...")
    plot_workspace(step=2)

    # 1. Image path 
    image_path = r"C:\Users\Ally\Downloads\akatsuki.jpg"

    # 2. Generate path + estimated drawing time
    path, t_est = generate_drawing_from_image(image_path, resolution="medium")
    print(f"Generated {len(path)} path points.")
    print(f"Estimated drawing time: {t_est:.1f} seconds")

    if not path:
        print("No valid path generated. Skipping send.")
    else:
        # 3. Plot the simulated drawing
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]

        plt.figure()
        plt.plot(xs, ys, linewidth=0.4)
        plt.gca().set_aspect('equal', 'box')
        plt.title("Predicted Single-Stroke Drawing")
        plt.xlabel("x (mm)")
        plt.ylabel("y (mm)")
        plt.grid(True)
        plt.show()

        # 4. Convert to joint angles (degrees for Arduino)
        angles_deg = path_to_joint_angles_deg(path)
        print(f"Valid angle pairs: {len(angles_deg)}")

        # 5. Upload trajectory and run it
        send_trajectory_to_arduino_streaming(angles_deg, port="COM13", baudrate=115200)

def debug_step_test(port="COM13"):
    """
    Send a few simple hardware angle commands to see
    which motor is which and which way 'positive' goes.
    Angles here are *already* motor degrees.
    """
    test_points_deg = [
        (0.0, 0.0),   # both at 0
        (20.0, 0.0),  # motor 1 +20
        (0.0, 0.0),
        (-20.0, 0.0), # motor 1 -20
        (0.0, 20.0),  # motor 2 +20
        (0.0, 0.0),
        (0.0, -20.0), # motor 2 -20
        (0.0, 0.0),
    ]

    send_trajectory_to_arduino_streaming(
        test_points_deg,
        port=port,
        baudrate=115200,
        dt=1.0
    )

#put arduino in Free mode (no PID)
def send_free(ard: ArduinoCommunication):
    if ard is not None and ard.connected:
        ard.send_data("FREE")

#put arduino in Hold mode (turn on PID)
def send_hold(ard: ArduinoCommunication):
    if ard is not None and ard.connected:
        ard.send_data("HOLD")


def request_current_angles(port="COM13", baudrate=115200, timeout=0.3, ard=None):
    """
    Send 'Q' to Arduino and read back: POS,<theta1_deg>,<theta2_deg>
    Returns (theta1_deg, theta2_deg) or None if failed
    """
    owns_connection = False

    if ard is None:
        ard = ArduinoCommunication(port=port, baudrate=baudrate, timeout=timeout)
        if not ard.connected:
            print(f"[CAL] Could not connect to Arduino on {port}")
            return None
        owns_connection = True

    try:
        # Remove any old data
        while True:
            line = ard.read_data()
            if not line:
                break

        # Ask for pose
        ard.send_data("Q")

        start = time.time()
        while time.time() - start < 1.0:
            line = ard.read_data()
            if not line:
                continue
            print(f"[ARD] {line}")
            if line.startswith("POS"):
                try:
                    _, t1_str, t2_str = line.split(",")
                    t1 = float(t1_str)
                    t2 = float(t2_str)
                    return t1, t2
                except ValueError:
                    continue

        print("[CAL] Timed out waiting for POS")
        return None

    finally:
        if owns_connection:
            ard.close()


"""
if __name__ == "__main__":
    # Simple 5-point step test
    test_points_deg = [
        (0.0, 0.0),
        (30.0, -30.0),
        (60.0, -60.0),
        (90.0, -90.0),
        (0.0, 0.0),
    ]

    send_trajectory_to_arduino_streaming(
        test_points_deg,
        port="COM13",   # adjust if different
        baudrate=115200,
        dt=0.5          # 0.5s between commands so you can see movement
    )
"""