import math
from Moon import Moon

# Constants similar to the Java version
WEIGHT_EMP = 165       # kg (empty weight of the module, perhaps)
WEIGHT_FULE = 420      # kg (fuel weight)
# https://davidson.weizmann.ac.il/online/askexpert/%D7%90%D7%99%D7%9A-%D7%9E%D7%98%D7%99%D7%A1%D7%99%D7%9D-%D7%97%D7%9C%D7%9C%D7%99%D7%AA-%D7%9C%D7%99%D7%A8%D7%97
WEIGHT_FULL = WEIGHT_EMP + WEIGHT_FULE  # kg
MAIN_ENG_F = 430       # N (main engine thrust)
SECOND_ENG_F = 25      # N (secondary engine thrust)
MAIN_BURN = 0.15       # liter per sec (main engine fuel burn)
SECOND_BURN = 0.009    # liter per sec (secondary engine fuel burn)
ALL_BURN = MAIN_BURN + 8 * SECOND_BURN  # total burn rate

def acc(weight, main, seconds):
    """Calculate acceleration given weight, main engine flag, and time in seconds.
       This mimics the Java method: if 'main' is true then add MAIN_ENG_F, plus seconds * SECOND_ENG_F."""
    t = 0
    if main:
        t += MAIN_ENG_F
    t += seconds * SECOND_ENG_F
    return t / weight

def acc_max(weight):
    """Compute the maximum possible acceleration given weight."""
    return acc(weight, True, 8)

def simulate():
    print("Simulating Bereshit's Landing:")
    # initial conditions (as in the Java main method)
    vs = 24.8            # vertical speed (m/s)
    hs = 932             # horizontal speed (m/s)
    dist = 181 * 1000    # horizontal distance (m)
    ang = 58.3           # angle in degrees (0 is vertical)
    alt = 13748          # altitude (m) 2:25:40 (as in the simulation) // https://www.youtube.com/watch?v=JJ0VfRL9AMs
    time = 0             # simulation time (sec)
    dt = 1               # time step (sec)
    acc_val = 0          # acceleration rate (m/s²)
    fuel = 121           # fuel amount
    weight = WEIGHT_EMP + fuel  # current weight (kg)
    print("time, vs, hs, dist, alt, ang, weight, acc")
    NN = 0.7           # throttle (rate between 0 and 1)

    # main simulation loop
    while alt > 0:
        # Print simulation status every 10 sec or when very close to the ground
        if time % 10 == 0 or alt < 100:
            print(f"time={time:.3f}, vs={vs:.3f}, hs={hs:.3f}, dist={dist:.3f}, alt={alt:.3f}, ang={ang:.3f}, weight={weight:.3f}, fuel={fuel:.3f}, acc={acc_val:.3f}")

        # High altitude: maintain vertical speed between 20 and 25 m/s
        if alt > 2000:
            if vs > 25:
                NN += 0.003 * dt  # increase braking power
            if vs < 20:
                NN -= 0.003 * dt  # reduce braking power
        else:
            # Below 2 km: rotate toward vertical
            if ang > 3:
                ang -= 3
            else:
                ang = 0
            NN = 0.5  # set moderate braking
            if hs < 2:
                hs = 0
            if alt < 125:  # very close to the ground
                NN = 1  # maximum braking
                if vs < 5:
                    NN = 0.7  # if slow enough, ease up

        if alt < 5:
            NN = 0.4  # near the surface, reduce braking

        # Compute acceleration components
        ang_rad = math.radians(ang)
        h_acc = math.sin(ang_rad) * acc_val  # horizontal acceleration component
        v_acc = math.cos(ang_rad) * acc_val  # vertical acceleration component

        # Lunar gravity acceleration (reduced by horizontal speed)
        vacc = Moon.get_acc(hs)

        time += dt
        dw = dt * ALL_BURN * NN  # fuel consumption this step

        if fuel > 0:
            fuel -= dw
            weight = WEIGHT_EMP + fuel
            acc_val = NN * acc_max(weight)
        else:
            acc_val = 0  # no fuel, no thrust

        # Adjust vertical acceleration by lunar gravity
        v_acc -= vacc

        if hs > 0:
            hs -= h_acc * dt  # update horizontal speed
        dist -= hs * dt     # update horizontal distance
        vs -= v_acc * dt    # update vertical speed
        alt -= dt * vs      # update altitude

if __name__ == "__main__":
    simulate()
