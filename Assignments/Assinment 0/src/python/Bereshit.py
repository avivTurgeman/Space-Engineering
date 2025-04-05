import math
from Moon import Moon


class Bereshit:
    class PID:
        def __init__(self, P=0.5, I=0.00002, D=0.2, SetPoint=30):
            """
            Initialize the PID controller with given gains.
            """
            self.Kp = P  # Proportional gain
            self.Ki = I  # Integral gain
            self.Kd = D  # Derivative gain

            self.set_point = SetPoint  # Desired target value
            self.integral = 0  # Integral term accumulator
            self.previous_error = 0  # Previous error for derivative calculation
            self.pid = 0  # Output of the PID controller

        def update(self, current_value, dt=1):
            """
            Update the PID controller with the current value and time step.

            :param current_value: The current value to control.
            :param dt: Time step since the last update.
            """
            # Calculate error
            error = current_value - self.set_point

            # Proportional term
            proportional = self.Kp * error

            # Integral term
            self.integral += error * dt
            integral = self.Ki * self.integral

            # Derivative term
            derivative = self.Kd * (error - self.previous_error) / dt

            # Update previous error for next iteration
            self.previous_error = error

            # Calculate output
            self.pid = proportional + integral + derivative

        def update_set_point(self, set_point):
            """
            Update the set point for the PID controller.

            :param set_point: The desired target value.
            """
            self.set_point = set_point
            self.integral = 0  # Reset integral term

    def __init__(
            self,
            wight_emp=165,  # kg (empty weight of the module, perhaps)
            max_fuel_weight=420,  # kg (max fuel tank weight)
            fuel=121,  # kg (initial fuel amount)
            main_eng_f=430,  # N (main engine thrust)
            second_eng_f=25,  # N (secondary engine thrust)
            main_burn=0.15,  # liter per sec (main engine fuel burn)
            second_burn=0.009,  # liter per sec (secondary engine fuel burn)
            vs=24.8,  # vertical speed (m/s)
            dvs=30,  # desired vertical speed (m/s)
            hs=1700,  # horizontal speed (m/s)
            dist=181 * 1000,  # distance (m)
            alt=13748,  # altitude (m)
            ang=60,  # angle in degrees (0 is vertical)
            time=0,  # simulation time (sec)
            dt=1,  # time step (sec)
            acc_val=0,  # acceleration rate (m/s²)
            nn=0.7,  # throttle (rate between 0 and 1)
            p=0.5,  # Proportional gain
            i=0.00002,  # Integral gain
            d=0.2,  # Derivative gain

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
        self.NN_pid = self.PID(p, i, d, dvs)  # PID controller for throttle adjustment

    def acc(self, weight, main, seconds):
        """Calculate acceleration given weight, main engine flag, and time in seconds.
        if 'main' is true then add MAIN_ENG_F, plus seconds * SECOND_ENG_F."""
        t = 0
        if main:
            t += self.MAIN_ENG_F
        t += seconds * self.SECOND_ENG_F
        return t / weight

    def acc_max(self, weight):
        """Compute the maximum possible acceleration given weight."""
        return self.acc(weight, True, 8)

    def update_dvs(self):
        """
        Update the desired vertical speed for the PID controller.
        """
        if self.altitude < 4000:
            self.desired_vertical_speed = 24
        if self.altitude < 2000:
            self.desired_vertical_speed = 16
        if self.altitude < 500:
            self.desired_vertical_speed = 12
        if self.altitude < 100:
            self.desired_vertical_speed = 6
        if self.altitude < 20:
            self.desired_vertical_speed = 1
        if self.altitude < 5:
            self.desired_vertical_speed = 0

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
        #
        # if self.horizontal_speed <= 0:
        #     self.angle = 0
        # else:
        #     self.angle = math.degrees(math.atan(self.vertical_speed / self.horizontal_speed))

        # if self.horizontal_speed < 500:
        #     self.angle = 60
        # if self.altitude < 1000:
        #     self.angle = 0  # vertical landing

        if self.horizontal_speed < 7:
            # If horizontal speed is almost zero, go vertical
            self.angle = 10
        elif self.horizontal_speed < 3:
            self.angle = 0
        elif self.altitude > 5000:
            # High altitude: keep a steep angle
            self.angle = min(60, self.angle + 1)  # Slowly tilt more
        elif self.altitude > 900:
            # Getting lower: prepare for vertical
            self.angle = max(50, self.angle - 1)
        elif self.altitude > 400:
            # Almost landing: go near vertical
            self.angle = max(20, self.angle - 1)
        elif self.horizontal_speed < 3:
            # Final landing phase: completely vertical
            self.angle = 0

    def simulate(self):
        print("Simulating Bereshit's Landing:")

        while self.altitude > 0:
            # Print simulation status every 10 sec or when very close to the ground
            if self.time % 10 == 0 or self.altitude < 100:
                print(
                    f"time={self.time:.3f}, vs={self.vertical_speed:.3f}, dvs={self.desired_vertical_speed}, hs={self.horizontal_speed:.3f}, dist={self.distance:.3f}, alt={self.altitude:.3f}, ang={self.angle:.3f}, weight={self.weight:.3f}, fuel={self.fuel:.3f}, acc={self.acc_val:.3f}, NN ={self.NN}")
            self.update_dvs()
            self.update_NN()  # Update the throttle using PID controller
            self.update_angle()  # Update the angle based on altitude

            # Compute acceleration components
            ang_rad = math.radians(self.angle)
            h_acc = math.sin(ang_rad) * self.acc_val  # horizontal acceleration component
            v_acc = math.cos(ang_rad) * self.acc_val  # vertical acceleration component

            # Lunar gravity acceleration (reduced by horizontal speed)
            moon_g_acc = Moon.gravitational_pull_acc(self.horizontal_speed)

            self.time += self.dt
            fuel_burn = self.dt * self.ALL_BURN * self.NN  # fuel consumption this step

            if self.fuel > 0:
                self.fuel -= fuel_burn
                self.weight = self.WEIGHT_EMP + self.fuel
                self.acc_val = self.NN * self.acc_max(self.weight)
            else:
                print("RUN OUT OF FUEL!")
                self.acc_val = 0  # no fuel, no thrust

            # Adjust vertical acceleration by lunar gravity
            v_acc -= moon_g_acc

            if self.horizontal_speed > 0:  # TODO: inspect and remove after improvement
                self.horizontal_speed -= h_acc * self.dt  # update horizontal speed
            self.vertical_speed -= v_acc * self.dt  # update vertical speed
            self.distance -= self.horizontal_speed * self.dt  # update horizontal distance
            self.altitude -= self.vertical_speed * self.dt  # update altitude

        print(
            f"time={self.time:.3f}, vs={self.vertical_speed:.3f}, dvs={self.desired_vertical_speed}, hs={self.horizontal_speed:.3f}, dist={self.distance:.3f}, alt={self.altitude:.3f}, ang={self.angle:.3f}, weight={self.weight:.3f}, fuel={self.fuel:.3f}, acc={self.acc_val:.3f}, NN = {self.NN}")


if __name__ == "__main__":
    bereshit = Bereshit(hs=932)
    bereshit.simulate()
