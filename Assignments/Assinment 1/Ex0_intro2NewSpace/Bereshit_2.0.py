import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- Simulation Constants and Functions ---

class Moon:
    # Moon characteristics (from Wikipedia)
    RADIUS = 3475 * 1000  # meters
    ACC = 1.622         # m/s², gravitational acceleration on the Moon
    EQ_SPEED = 1700     # m/s, reference speed

    @staticmethod
    def get_acc(speed):
        """Compute effective lunar gravitational acceleration based on horizontal speed."""
        n = abs(speed) / Moon.EQ_SPEED
        return (1 - n) * Moon.ACC

# Constants
WEIGHT_EMP = 165       # kg
WEIGHT_FULE = 420      # kg
MAIN_ENG_F = 430       # N
SECOND_ENG_F = 25      # N
MAIN_BURN = 0.15       # liter per sec
SECOND_BURN = 0.009    # liter per sec
ALL_BURN = MAIN_BURN + 8 * SECOND_BURN

def acc(weight, main, seconds):
    """Calculate acceleration based on engine usage."""
    thrust = 0
    if main:
        thrust += MAIN_ENG_F
    thrust += seconds * SECOND_ENG_F
    return thrust / weight

def acc_max(weight):
    """Maximum possible acceleration given the current weight."""
    return acc(weight, True, 8)

# --- Simulation Generator ---
def simulate_landing():
    # Initial conditions
    vs = 24.8            # vertical speed (m/s)
    hs = 932             # horizontal speed (m/s)
    dist = 181 * 1000    # horizontal distance remaining (m)
    ang = 58.3           # angle in degrees
    alt = 13748          # altitude (m)
    dt = 1               # time step (sec)
    acc_val = 0          # current engine acceleration (m/s²)
    fuel = 121           # fuel amount
    weight = WEIGHT_EMP + fuel  # current weight (kg)
    NN = 0.7             # throttle (value between 0 and 1)
    
    total_distance = dist  # Save the initial total horizontal distance

    while alt > 0:
        # Compute spaceship's horizontal position (progress traveled)
        x_pos = total_distance - dist

        # Yield current state (x position, altitude, simulation time) for plotting
        yield (x_pos, alt)

        # --- Update simulation state as per your original logic ---
        if alt > 2000:
            if vs > 25:
                NN += 0.003 * dt
            if vs < 20:
                NN -= 0.003 * dt
        else:
            if ang > 3:
                ang -= 3
            else:
                ang = 0
            NN = 0.5
            if hs < 2:
                hs = 0
            if alt < 125:
                NN = 1
                if vs < 5:
                    NN = 0.7
        if alt < 5:
            NN = 0.4

        ang_rad = math.radians(ang)
        h_acc = math.sin(ang_rad) * acc_val
        v_acc = math.cos(ang_rad) * acc_val

        # Adjust vertical acceleration by lunar gravity (reduced by horizontal speed)
        vacc = Moon.get_acc(hs)

        dw = dt * ALL_BURN * NN  # fuel consumption for this step

        if fuel > 0:
            fuel -= dw
            weight = WEIGHT_EMP + fuel
            acc_val = NN * acc_max(weight)
        else:
            acc_val = 0  # no fuel, no thrust

        v_acc -= vacc

        if hs > 0:
            hs -= h_acc * dt
        dist -= hs * dt
        vs -= v_acc * dt
        alt -= dt * vs

# --- Set Up Matplotlib Animation ---

# Create the figure and axis
fig, ax = plt.subplots()
ax.set_title("Moon Landing Simulation")
ax.set_xlabel("Horizontal Distance (m)")
ax.set_ylabel("Altitude (m)")
ax.set_xlim(0, 190 * 1000)   # Maximum horizontal distance
ax.set_ylim(0, 13748)        # Maximum altitude

# Line object for the trajectory
line, = ax.plot([], [], marker='o', lw=2)
trajectory_x, trajectory_y = [], []

def init():
    """Initialization function for the animation."""
    line.set_data([], [])
    return line,

def update(frame):
    """Update function that is called for each frame of the animation."""
    x, y = frame
    trajectory_x.append(x)
    trajectory_y.append(y)
    line.set_data(trajectory_x, trajectory_y)
    return line,

# Create the animation using the simulation generator as frames.
ani = animation.FuncAnimation(
    fig, update, frames=simulate_landing, init_func=init,
    interval=100, repeat=False
)

plt.show()
