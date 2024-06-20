import numpy as np


# import scipy.stats as stats
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# from numpy import linalg as LA

# Algorythem written by Dr. techn. Babak Salamat, OOP structure created by Tim Drouven
# Date: 08.05.2024
# Update: Class. 23.05.2024

class GMP3Config:
    def __init__(self, maxit, alpha, wdamp, delta, vx_max, vy_max, Q11, Q22, Q12, obstacles=None):
        """
        Initializes the configuration for the GMP3 algorithm using the GMP3Config class..
        Obstacles (list of tuples): List of coordinates (x, y, r)
        representing obstacles, if any and

        """
        self.maxit = maxit
        self.alpha = alpha
        self.wdamp = wdamp
        self.delta = delta
        # self.nobs   = nobs is now calculate by len(obstacles)
        # self.x_in   = x_in
        # self.y_in   = y_in
        # self.x_f1   = x_f1
        # self.y_f1   = y_f1
        # self.x_f2   = x_f2
        # self.y_f2   = y_f2
        self.vx_max = vx_max
        self.vy_max = vy_max
        self.Q11 = Q11
        self.Q22 = Q22
        self.Q12 = Q12
        self.nobs = len(obstacles)
        self.obstacles = obstacles if obstacles is not None else []
        self.pathfound = []
        self.iterations_needed = []


class GMP3:
    def __init__(self, gmpConfig):
        """
        Initialize the GMP3 algorithm using the gmpConfig class.
        Obstacles (list of tuples): List of coordinates (x, y) representing obstacles, if any.
        """
        self.maxit = gmpConfig.maxit
        self.alpha = gmpConfig.alpha
        self.wdamp = gmpConfig.wdamp
        self.delta = gmpConfig.delta
        self.vx_max = gmpConfig.vx_max
        self.vy_max = gmpConfig.vy_max
        self.Q11 = gmpConfig.Q11
        self.Q22 = gmpConfig.Q22
        self.Q12 = gmpConfig.Q12
        self.tf = None
        self.nobs = gmpConfig.nobs
        self.obstacles = gmpConfig.obstacles if gmpConfig.obstacles is not None else []

        # these values will be set by calling the calculate method
        self.x_in = None
        self.y_in = None
        self.x_f1 = None
        self.y_f1 = None
        self.x_f2 = None
        self.y_f2 = None

        self.theta = []
        self.violation = []
        self.computedValue = []
        self.robs = []
        self.Xobs = []
        self.Yobs = []
        self.x = []
        self.y = []
        self.xdot = []
        self.ydot = []
        self.verbose = False

    def sat(self, x):
        """Saturation function to limit the output between -1 and 1."""
        return np.clip(x, -1, 1)

    def Control(self, xdot, ex, ex_dot, Ts, Ux, vmax):
        # C2 Control logic
        z = (1 / Ts) * ((ex / Ts) + (ex_dot / 2))
        z_dot = ex_dot / Ts
        m = np.floor((1 + np.sqrt(1 + 8 * abs(z))) / 2)

        sigma = z_dot + (z / m) + ((m - 1) / 2) * np.sign(z)

        # Calculate the control input with saturation and sign functions
        u = -Ux * self.sat(sigma) * ((1 + np.sign(xdot * np.sign(sigma) + vmax - Ts * Ux)) / 2)
        return u

    def Cost1(self, xobs, yobs, robs, x_in, y_in, x_f1, x_f2, y_f1, y_f2, theta):
        # Unpack theta into wx and wy
        half_n = len(theta) // 2
        wx = theta[:half_n]
        wy = theta[half_n:]
        # Time parameters
        Ts = 0.05
        tf = self.tf
        t = np.arange(0, tf + Ts, Ts)
        k = len(t)
        tt = np.linspace(0, tf, len(wx) + 3)
        # Interpolating the desired setpoints
        XS = np.array([x_in] + list(wx) + [x_f1, x_f2])
        YS = np.array([y_in] + list(wy) + [y_f1, y_f2])
        rxd = np.interp(t, tt, XS)
        ryd = np.interp(t, tt, YS)
        # Velocity calculation
        vrxd = np.diff(rxd) / Ts
        vryd = np.diff(ryd) / Ts
        vrxd = np.append(vrxd, 0)  # Append zero to maintain array size
        vryd = np.append(vryd, 0)
        # Define maximum velocities and control inputs
        vx_max = self.vx_max
        vy_max = self.vy_max
        Ux = Uy = 3
        # Initialize state and control input arrays
        x = x_in * np.ones(k)
        y = y_in * np.ones(k)
        xdot = np.zeros(k)
        ydot = np.zeros(k)
        ux = np.zeros(k)
        uy = np.zeros(k)
        # Simulate control system response
        for i in range(k - 1):
            ux[i] = self.Control(xdot[i], (x[i] - rxd[i]) / Ux, (xdot[i] - vrxd[i]) / Ux, Ts, Ux, vx_max)
            uy[i] = self.Control(ydot[i], (y[i] - ryd[i]) / Uy, (ydot[i] - vryd[i]) / Uy, Ts, Uy, vy_max)
            xdot[i + 1] = xdot[i] + Ts * ux[i]
            x[i + 1] = x[i] + (Ts / 2) * (xdot[i + 1] + xdot[i])
            ydot[i + 1] = ydot[i] + Ts * uy[i]
            y[i + 1] = y[i] + (Ts / 2) * (ydot[i + 1] + ydot[i])
        # Evaluate trajectory's performance
        QQ = np.array([[self.Q11, self.Q12], [self.Q12, self.Q22]])
        quad = np.sum(QQ[0, 0] * np.diff(x) ** 2 + QQ[1, 1] * np.diff(y) ** 2 + 2 * QQ[0, 1] * np.diff(x) * np.diff(y))
        # Obstacle violation measure
        nobs = len(xobs)
        Violation = 0
        for j in range(nobs):
            d = np.sqrt(
                QQ[0, 0] * (x - xobs[j]) ** 2 + QQ[1, 1] * (y - yobs[j]) ** 2 + QQ[0, 1] * (x - xobs[j]) ** 2 * (
                            y - yobs[j]) ** 2)
            v = np.maximum(1 - d / (robs[j] + 0.3), 0)

            Violation += 1.5 * np.mean(v) + 0.8 * np.std(v)
        J = - (0.012 * quad + 1.6 * Violation)
        return J

    def Cost(self, xobs, yobs, robs, x_in, y_in, x_f1, x_f2, y_f1, y_f2, theta):
        # Unpack theta into wx and wy
        half_n = len(theta) // 2
        wx = theta[:half_n]
        wy = theta[half_n:]
        # Time parameters
        Ts = 0.05
        tf = self.tf
        t = np.arange(0, tf + Ts, Ts)
        k = len(t)
        tt = np.linspace(0, tf, len(wx) + 3)
        # Interpolating the desired setpoints
        XS = np.array([x_in] + list(wx) + [x_f1, x_f2])
        YS = np.array([y_in] + list(wy) + [y_f1, y_f2])
        rxd = np.interp(t, tt, XS)
        ryd = np.interp(t, tt, YS)
        # Velocity calculation
        vrxd = np.diff(rxd) / Ts
        vryd = np.diff(ryd) / Ts
        vrxd = np.append(vrxd, 0)  # Append zero to maintain array size
        vryd = np.append(vryd, 0)
        # Define maximum velocities and control inputs
        vx_max = self.vx_max
        vy_max = self.vy_max
        Ux = Uy = 3
        # Initialize state and control input arrays
        x = x_in * np.ones(k)
        y = y_in * np.ones(k)
        xdot = np.zeros(k)
        ydot = np.zeros(k)
        ux = np.zeros(k)
        uy = np.zeros(k)
        # Simulate control system response
        for i in range(k - 1):
            ux[i] = self.Control(xdot[i], (x[i] - rxd[i]) / Ux, (xdot[i] - vrxd[i]) / Ux, Ts, Ux, vx_max)
            uy[i] = self.Control(ydot[i], (y[i] - ryd[i]) / Uy, (ydot[i] - vryd[i]) / Uy, Ts, Uy, vy_max)
            xdot[i + 1] = xdot[i] + Ts * ux[i]
            x[i + 1] = x[i] + (Ts / 2) * (xdot[i + 1] + xdot[i])
            ydot[i + 1] = ydot[i] + Ts * uy[i]
            y[i + 1] = y[i] + (Ts / 2) * (ydot[i + 1] + ydot[i])
        # Evaluate trajectory's performance
        QQ = np.array([[self.Q11, self.Q12], [self.Q12, self.Q22]])
        quad = np.sum(QQ[0, 0] * np.diff(x) ** 2 + QQ[1, 1] * np.diff(y) ** 2 + 2 * QQ[0, 1] * np.diff(x) * np.diff(y))
        # Obstacle violation measure
        nobs = len(xobs)
        Violation = 0
        for j in range(nobs):
            d = np.sqrt(
                QQ[0, 0] * (x - xobs[j]) ** 2 + QQ[1, 1] * (y - yobs[j]) ** 2 + QQ[0, 1] * (x - xobs[j]) ** 2 * (
                            y - yobs[j]) ** 2)
            v = np.maximum(1 - d / (robs[j] + 0.3), 0)

            Violation += 1.5 * np.mean(v) + 0.8 * np.std(v)
        J = - (0.012 * quad + 1.6 * Violation)
        return J, Violation

    def Agents(self, xobs, yobs, robs, x_in, y_in, x_f1, x_f2, y_f1, y_f2, theta, delta, n):
        unit = np.zeros(n)
        grad = np.zeros(n)

        for i in range(n):
            unit[i] = 1
            grad[i] = (-self.Cost1(xobs, yobs, robs, x_in, y_in, x_f1, x_f2, y_f1, y_f2, theta + 2 * delta * unit) +
                       8 * self.Cost1(xobs, yobs, robs, x_in, y_in, x_f1, x_f2, y_f1, y_f2, theta + delta * unit) -
                       8 * self.Cost1(xobs, yobs, robs, x_in, y_in, x_f1, x_f2, y_f1, y_f2, theta - delta * unit) +
                       self.Cost1(xobs, yobs, robs, x_in, y_in, x_f1, x_f2, y_f1, y_f2, theta - 2 * delta * unit)) / (
                                  12 * delta)
            # unit[i] = 0  # Reset the unit vector

        return grad

    def results(self, x_in, y_in, x_f1, x_f2, y_f1, y_f2, theta):
        half_n = len(theta) // 2
        wx = theta[:half_n]
        wy = theta[half_n:]

        # Time parameters
        Ts = 0.05
        tf = self.tf
        t = np.arange(0, tf + Ts, Ts)
        k = len(t)

        tt = np.linspace(0, tf, len(wx) + 3)

        # Interpolating the desired setpoints
        XS = np.array([x_in] + list(wx) + [x_f1, x_f2])
        YS = np.array([y_in] + list(wy) + [y_f1, y_f2])
        rxd = np.interp(t, tt, XS)
        ryd = np.interp(t, tt, YS)

        # Calculate velocities
        vrxd = np.gradient(rxd, t)
        vryd = np.gradient(ryd, t)

        # Initialize arrays for states and control inputs
        x = x_in * np.ones(k)
        y = y_in * np.ones(k)
        xdot = np.zeros(k)
        ydot = np.zeros(k)
        ux = np.zeros(k)
        uy = np.zeros(k)

        # Simulation loop
        for i in range(k - 1):
            ex = (x[i] - rxd[i]) / 3  # Ux is 3
            ey = (y[i] - ryd[i]) / 3  # Uy is 3
            ex_dot = (xdot[i] - vrxd[i]) / 3
            ey_dot = (ydot[i] - vryd[i]) / 3

            ux[i] = self.Control(xdot[i], ex, ex_dot, Ts, 3, 2)  # Ux and vx_max
            uy[i] = self.Control(ydot[i], ey, ey_dot, Ts, 3, 2)  # Uy and vy_max

            xdot[i + 1] = xdot[i] + Ts * ux[i]
            x[i + 1] = x[i] + Ts / 2 * (xdot[i + 1] + xdot[i])

            ydot[i + 1] = ydot[i] + Ts * uy[i]
            y[i + 1] = y[i] + Ts / 2 * (ydot[i + 1] + ydot[i])

        return t, x, y, xdot, ydot, ux, uy

    def calculate(self, startposition, finalposition):
        """
        Calculate the path using the start position [x: float, y: flaot] to the
        end/final position [x: float, y: float]
        """
        self.x_in = startposition[0]
        self.y_in = startposition[1]
        self.x_f1 = finalposition[0]
        self.y_f1 = finalposition[1]
        self.x_f2 = finalposition[0]
        self.y_f2 = finalposition[1]
        self.tf = abs(self.x_f2 - self.x_in) + 3 if abs(self.x_f2 - self.x_in) + 3 > abs(
            self.y_f2 - self.y_in) + 3 else abs(self.y_f2 - self.y_in) + 3

        theta = np.zeros((2 * self.nobs, self.maxit))
        x_values = np.linspace(self.x_in + 1, self.x_f1 - 1, self.nobs)
        y_values = np.linspace(self.y_in + 1, self.y_f1 - 1, self.nobs)
        theta[:, 0] = np.concatenate([x_values, y_values])

        # Assuming necessary functions and config object are defined
        jj = np.zeros((1, self.maxit))
        previous_value = float('inf')  # Initialize previous value to infinity
        tolerance = 0.05  # Define a small tolerance value for the stopping criterion

        # Predefine some values / Reset the object
        self.pathfound = False
        self.iterations_needed = None
        self.x = None
        self.y = None

        self.theta = None
        self.robs = None
        self.Xobs = None
        self.Yobs = None
        self.xdot = None
        self.ydot = None

        theta_opt = []
        # value = 0
        V = 0
        # computedValue = 0
        m = 0
        v = 0
        beta1 = 0.001
        beta2 = 0.98

        theta = np.zeros((2 * self.nobs, self.maxit))
        x_values = np.linspace(self.x_in + 1, self.x_f1 - 1, self.nobs)
        y_values = np.linspace(self.y_in + 1, self.y_f1 - 1, self.nobs)
        theta[:, 0] = np.concatenate([x_values, y_values])

        # Initialize variables for stopping criteria
        previous_value = float('inf')
        consecutive_no_improvement = 0
        max_consecutive_no_improvement = 25  # Number of iterations to wait for improvement
        gradient_tolerance = 1e-20  # Tolerance for gradient norm

        for i in range(self.maxit - 1):

            obstacles = np.transpose(np.array(self.obstacles))

            Xobs = obstacles[0]
            Yobs = obstacles[1]
            robs = obstacles[2]

            grad = self.Agents(Xobs, Yobs, robs, self.x_in, self.y_in, self.x_f1, self.x_f2, self.y_f1, self.y_f2,
                               theta[:, i], self.delta, 2 * self.nobs)

            # Dampen the learning rate
            self.alpha *= self.wdamp

            m = beta1 * m + (1 - beta1) * (grad);
            v = beta2 * v + (1 - beta2) * (grad) ** 2;

            mhat = m / (1 - beta1 ** i + 1);
            vhat = v / (1 - beta2 ** i + 1);

            # Update theta[:, i+1]
            theta[:, i + 1] = theta[:, i] + (self.alpha * mhat / (np.sqrt(1 + vhat)));
            # theta[:, i+1] = theta[:, i] + self.alpha * grad

            # Ensure theta values are clamped between x_in and x_f2
            # theta[:, i+1] = np.clip(theta[:, i+1], self.x_in, self.x_f2)

            # Calculate cost and violation for the new theta values at i+1
            jj, V = self.Cost(Xobs, Yobs, robs, self.x_in, self.y_in, self.x_f1, self.x_f2, self.y_f1, self.y_f2,
                              theta[:, i + 1])

            # Normalize the value
            current_value = abs(jj)

            if (V == 0 and abs(current_value - previous_value) < tolerance):
                theta_opt = theta[:, i + 1]
                t, x, y, xdot, ydot, ux, uy = self.results(self.x_in, self.y_in, self.x_f1, self.x_f2, self.y_f1,
                                                           self.y_f2, theta_opt)
                self.pathfound = True
                break  # Exit the loop if optimal conditions are met

            # Check if the gradient norm is below the tolerance
            if np.linalg.norm(grad) < gradient_tolerance:
                theta_opt = theta[:, i + 1]
                t, x, y, xdot, ydot, ux, uy = self.results(self.x_in, self.y_in, self.x_f1, self.x_f2, self.y_f1,
                                                           self.y_f2, theta_opt)
                self.pathfound = True
                break  # Exit the loop if gradient norm is very small

            # Update previous value and check for consecutive no improvement
            if abs(current_value - previous_value) < tolerance:
                consecutive_no_improvement += 1
                if consecutive_no_improvement >= max_consecutive_no_improvement:
                    theta_opt = theta[:, i + 1]
                    t, x, y, xdot, ydot, ux, uy = self.results(self.x_in, self.y_in, self.x_f1, self.x_f2, self.y_f1,
                                                               self.y_f2, theta_opt)
                    self.pathfound = True
                    break  # Exit the loop if no improvement over many iterations
            else:
                consecutive_no_improvement = 0
            # Update previous value
            previous_value = current_value

            # value_str = f"{value:.4f}" if np.isscalar(value) else str(value)
            if self.verbose == True:
                print(f"Iteration {i + 1}   Violation = {V:.4f}   Computed Value = {current_value}")

        self.iterations_needed = i
        self.violation = V
        self.computedValue = current_value

        # Check if theta_opt was set and handle accordingly
        if theta_opt is not None:
            if self.verbose == True:
                print("Optimal theta found.")
            # Optional: do something with t, x, y, or theta_opt
        else:
            if self.verbose == True:
                print("No optimal theta found within the given iterations.")

        t, x, y, xdot, ydot, ux, uy = self.results(self.x_in, self.y_in, self.x_f1, self.x_f2, self.y_f1, self.y_f2,
                                                   theta[:, i])

        self.x = x
        self.y = y
        self.t = t

        self.theta = theta
        self.robs = robs
        self.Xobs = Xobs
        self.Yobs = Yobs
        self.xdot = xdot
        self.ydot = ydot
        # return t, x, y, xdot, ydot
