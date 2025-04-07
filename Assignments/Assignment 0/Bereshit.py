import math
import pandas as pd
from Moon import Moon

class Bereshit:
    class PID:
        def __init__(self, P=0.05, I=0.00002, D=0.2, SetPoint=30):
            """
            Initialize the PID controller with given gains.
            """
            self.Kp = P  # Proportional gain
            self.Ki = I  # Integral gain
            self.Kd = D  # Derivative gain

            self.set_point = SetPoint  # Desired target value
            self.integral = 0  # Integral term accumulator
            self.error=0
            self.derivative=0
            self.previous_error = 0  # Previous error for derivative calculation
            self.pid = 0  # Output of the PID controller
            self.first_run = True  # Flag to indicate if it's the first run

        def update(self, current_value, dt=1):
            """
            Update the PID controller with the current value and time step.

            :param current_value: The current value to control.
            :param dt: Time step since the last update.
            """
            # Calculate error
            self.error = current_value - self.set_point

            # Integral term
            self.integral += self.error * dt

            # Derivative term
            if self.first_run:
                self.derivative = 0
                self.first_run = False
            else:
                self.derivative = (self.error - self.previous_error) / dt

            # Update previous error for next iteration
            self.previous_error = self.error

            # Calculate output
            self.pid = self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative

        def update_set_point(self, set_point):
            """
            Update the set point for the PID controller.

            :param set_point: The desired target value.
            """
            if set_point != self.set_point:
                self.set_point = set_point
                # self.integral = 0  # Reset integral term

    def __init__(
            self,
            wight_emp=165,  # kg (empty weight of the module, perhaps)
            max_fuel_weight=420,  # kg (max fuel tank weight)
            fuel=120.84,  # kg (initial fuel amount)
            main_eng_f=430,  # N (main engine thrust)
            second_eng_f=25,  # N (secondary engine thrust)
            main_burn=0.15,  # liter per sec (main engine fuel burn)
            second_burn=0.009,  # liter per sec (secondary engine fuel burn)
            vs=24.8,  # vertical speed (m/s)
            dvs=30,  # desired vertical speed (m/s)
            hs=932,  # horizontal speed (m/s)
            dist=181 * 1000,  # distance (m)
            alt=13722.47,  # altitude (m)
            ang=60,  # angle in degrees (0 is vertical)
            time=0,  # simulation time (sec)
            dt=1,  # time step (sec)
            acc_val=0,  # acceleration rate (m/s²)
            nn=0.7,  # throttle (rate between 0 and 1)
            Kp=0.05,  # Proportional gain
            Ki=0.00002,  # Integral gain
            Kd=0.2,  # Derivative gain
            data_log = []  # List to store simulation data

    ):
        """
        Initialize the Bereshit lander with given parameters.
        """
        self.WEIGHT_EMP = wight_emp
        self.MAX_FUEL_WEIGHT = max_fuel_weight
        self.MAX_WEIGHT = wight_emp + max_fuel_weight,
        self.fuel = fuel
        self.weight = wight_emp + fuel  # current weight (kg)
        self.MAIN_ENG_F = main_eng_f
        self.SECOND_ENG_F = second_eng_f
        self.MAIN_BURN = main_burn
        self.SECOND_BURN = second_burn
        self.ALL_BURN = main_burn + 8 * second_burn  # total burn rate (liters per sec)
        self.vertical_speed = vs
        self.desired_vertical_speed = dvs  # desired vertical speed (m/s)
        self.horizontal_speed = hs
        self.distance = dist
        self.altitude = alt
        self.angle = ang  # angle in degrees (0 is vertical)
        self.time = time  # simulation time (sec)
        self.dt = dt  # time step (sec)
        self.acc_val = acc_val  # acceleration rate (m/s²)
        self.NN = nn  # throttle (rate between 0 and 1)
        self.NN_pid = self.PID(Kp, Ki, Kd, dvs)  # PID controller for throttle adjustment
        self.data_log = data_log  # List to store simulation data

    def acc(self, weight, main, seconds):
        """Calculate acceleration given weight, main engine flag, and amount of running second engines."""
        t = 0
        if main:
            t += self.MAIN_ENG_F
        t += seconds * self.SECOND_ENG_F
        return t / weight

    def acc_max(self, weight):
        """Compute the maximum possible acceleration given weight."""
        return self.acc(weight, True, 8)

    def parabolic_curve(self, height, start=4000, rng=30):

        x = height / start  # Normalize height to a value between 0 and 1
        x = min(x, 1)  # Limit x to a maximum of 1

        y = -1 * (x - 1) ** 2 + 1

        output = y * rng  # Scale to the desired range

        return round(output) if output > 1.6 else 1.6 # Ensure a minimum output of 1.6

    def update_dvs(self):
        """
        Update the desired vertical speed for the PID controller.
        """
        self.desired_vertical_speed = self.parabolic_curve(self.altitude, 3000, 25)  # Update desired vertical speed based on altitude
            
        self.NN_pid.update_set_point(self.desired_vertical_speed)  # Update the set point for the PID controller

    def update_NN(self):
        """
        Update the PID controller with the current vertical speed and time step.
        """
        self.update_dvs()  # Update the desired vertical speed based on altitude
        self.NN_pid.update(self.vertical_speed, self.dt)
        self.NN += self.NN_pid.pid
        self.NN = max(0, min(1, self.NN))

    def update_angle(self):
        if self.horizontal_speed < 0.1:
            self.angle = 0
        elif self.horizontal_speed < 1:
            self.angle = 2

    def simulate(self):
        # Reset simulation data log
        self.data_log = []  # Initialize storage for simulation logs

        self.NN_pid.update(self.vertical_speed, self.dt)

        while self.altitude > 0.2:
            # Log simulation status
            self.data_log.append({
                'Gacc': round(moon_g_acc, 3),
                'acceleration': round(self.acc_val, 3),
                'Hass': round(h_acc, 3),
                'Vass': round(v_acc, 3),
                'fuel amount': round(self.fuel, 3),
                'throttle (NN)': round(self.NN, 3),
                'angle': round(self.angle, 3),
                'horizontal speed': round(self.horizontal_speed, 3),
                'vertical speed': round(self.vertical_speed, 3),
                'desired vertical speed': self.desired_vertical_speed,
                'distance': round(self.distance, 3),
                'P': round(self.NN_pid.error, 3),
                'I': round(self.NN_pid.integral, 3),
                'D': round(self.NN_pid.derivative, 3),
                'PID': round(self.NN_pid.pid, 3),
                'altitude': round(self.altitude, 3),
                'total weight': round(self.weight, 3),
                'time': round(self.time, 3),
            })

            self.time += self.dt
            fuel_burn = self.dt * self.ALL_BURN * self.NN  # fuel consumption this step

            if self.fuel > 0:
                self.fuel -= fuel_burn
                self.weight = self.WEIGHT_EMP + self.fuel
                self.acc_val = self.NN * self.acc_max(self.weight)
            else:
                print("RUN OUT OF FUEL!")
                self.acc_val = 0  # no fuel, no thrust

            # Compute acceleration components
            ang_rad = math.radians(self.angle)
            h_acc = math.sin(ang_rad) * self.acc_val  # horizontal acceleration component
            v_acc = math.cos(ang_rad) * self.acc_val  # vertical acceleration component

            # Lunar gravity acceleration (reduced by horizontal speed)
            moon_g_acc = Moon.gravitational_pull_acc(self.horizontal_speed)

            # Adjust vertical acceleration by lunar gravity
            v_acc -= moon_g_acc


            if self.horizontal_speed > 0:
                self.horizontal_speed -= h_acc * self.dt  # update horizontal speed
            self.vertical_speed -= v_acc * self.dt  # update vertical speed
            self.distance -= self.horizontal_speed * self.dt  # update horizontal distance
            self.altitude = max(0, self.altitude - (self.vertical_speed * self.dt))  # update altitude

            self.update_dvs()
            self.update_NN()  # Update the throttle using PID controller
            self.update_angle()  # Update the angle based on altitude
        
        self.data_log = pd.DataFrame(self.data_log)


if __name__ == "__main__":
    # Create an instance of the Bereshit class
    bereshit = Bereshit()

    # Simulate the landing
    bereshit.simulate()