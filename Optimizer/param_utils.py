import numpy as np

class ParameterUtils:
    def __init__(self, body_weight = 84):
        """
        Initialize ParameterUtils with parameter ranges and fixed parameters.
        """
        # Variable Parameters Range (6 parameters) - Incline (10 deg, 1 m/s)
        self.variable_params_range = [
            -0.6, -0.2,    # 𝑝_1: Extension Peak Torque
            0.1, 0.3,      # 𝑝_2: Flexion Peak Torque
            10.0, 30.0,     # 𝑝_3: Extension Peak Time
            60.0, 80.0,     # 𝑝_4: Flexion Peak Time
            35.0, 55.0,     # 𝑝_5: Mid Time
            15.0, 35.0      # 𝑝_6: Extension Rise Time
        ]

        self.body_weight = body_weight

        self.scaling_range = [0, 10]

        # Extract min and max parameters from the interleaved list
        self.min_params = self.variable_params_range[::2]
        self.max_params = self.variable_params_range[1::2]

    def scale_params(self, params):
        """
        Scale parameters from their original range to the scaling range.

        Args:
            params (list or np.ndarray): Original parameters.

        Returns:
            np.ndarray: Scaled parameters.
        """
        scaled_params = np.zeros(len(params))
        for i in range(len(params)):
            scaled_params[i] = self.scaling_range[0] + (
                (params[i] - self.min_params[i]) * 
                (self.scaling_range[1] - self.scaling_range[0]) / 
                (self.max_params[i] - self.min_params[i])
            )
        return scaled_params

    def recover_params(self, params):
        """
        Recover original parameters from the scaled range.

        Args:
            params (list or np.ndarray): Scaled parameters.

        Returns:
            np.ndarray: Recovered original parameters.
        """       
        recovered_params = np.zeros(len(params))
        for i in range(len(recovered_params)):
            recovered_params[i] = self.min_params[i] + (
                (params[i] - self.scaling_range[0]) * 
                (self.max_params[i] - self.min_params[i]) / 
                (self.scaling_range[1] - self.scaling_range[0])
            )
        return recovered_params


    def weight_params(self, params):
        # Convert normalized torque values back to absolute values
        params[0] = params[0] * self.body_weight  # Extension Peak Torque
        params[1] = params[1] * self.body_weight  # Flexion Peak Torque
        
        return params
    
    def constrain_params(self, params):
        """
        Apply constraints to the variable parameters based on predefined rules.

        Since other parameters are fixed, constraints related to them are no longer necessary.

        Args:
            params (list or np.ndarray): Variable parameters to be constrained.

        Returns:
            np.ndarray: Constrained parameters.
        """
        flexionFallTimeMin = 10
        min_slope = 15

        if params[5] - params[2] + params[3] > 100 - flexionFallTimeMin:
            params[5] = (100 - flexionFallTimeMin) + params[2] - params[3]

        if (params[4] - params[2] < min_slope) or (params[3] - params[4] < min_slope):
            if (params[4] - params[2] < min_slope):
                params[4] = min_slope + params[2]
            elif (params[3] - params[4] < min_slope):
                params[4] = params[3] - min_slope

        # Check if all parameters are within their specified ranges
        for i in range(len(params)):
            if params[i] < self.min_params[i] or params[i] > self.max_params[i]:
                print(f"Parameter params[{i}] out of range: {params[i]:.1f} "
                    f"(Allowed: {self.min_params[i]:.1f} to {self.max_params[i]:.1f})")
                
        return params

    def hermite_cubic_to_power_cubic(self, x1, f1, d1, x2, f2, d2, c0, c1, c2, c3, index):
        """
        Convert Hermite cubic coefficients to power cubic coefficients for a segment.

        Args:
            x1, f1, d1 (float): Start point and derivative.
            x2, f2, d2 (float): End point and derivative.
            c0, c1, c2, c3 (list): Lists to store coefficients.
            index (int): Current segment index.
        """
        h = x2 - x1
        if h == 0:
            h = 1e-6

        df = (f2 - f1) / h

        c0[index] = f1
        c1[index] = d1
        c2[index] = -(2.0 * d1 - 3.0 * df + d2) / h
        c3[index] = (d1 - 2.0 * df + d2) / (h * h)

    def evaluate_hermite(self, x, c0, c1, c2, c3):
        """
        Evaluate the Hermite cubic polynomial at a given point.

        Args:
            x (float): Point at which to evaluate.
            c0, c1, c2, c3 (float): Hermite coefficients.

        Returns:
            float: Evaluated value.
        """
        return c0 + c1 * x + c2 * x**2 + c3 * x**3

    def calculate_torque(self, params, gaitCycle):
        """
        Calculate the desired torque profile based on current parameters and gait cycle.

        Args:
            params (list or np.ndarray): The six parameters, in the order the
                firmware expects them: [extensionPeakTorque, flexionPeakTorque,
                extensionPeakTime, flexionPeakTime, midTime, extensionRiseTime].
            gaitCycle (np.ndarray): Array representing the gait cycle percentages (0 to 100).

        Returns:
            tuple:
                - torqueDesired (np.ndarray): Array of desired torque values.
                - torque_dots (tuple): Tuple containing (x, y) points used in the calculation.
        """
        
        # Recover original parameters
        p1, p2, p3, p4, p5, p6 = params
        extensionPeakTorque = p1
        flexionPeakTorque = p2
        extensionPeakTime = p3
        flexionPeakTime = p4
        zeroCrossingTime = p5
        zeroDurationTime = 0
        extensionRiseTime = p6
        flexionFallTime = 100 - (extensionRiseTime - extensionPeakTime + flexionPeakTime)

        # Define the x (time) and y (torque) values for key points, including the mid-phase zero
        x = [
            extensionPeakTime - extensionRiseTime,
            extensionPeakTime,
            zeroCrossingTime - zeroDurationTime / 2,
            zeroCrossingTime + zeroDurationTime / 2,
            flexionPeakTime,
            flexionPeakTime + flexionFallTime
        ]
        y = [0, extensionPeakTorque, 0, 0, flexionPeakTorque, 0]

        # Number of segments
        num_segments = len(x) - 1

        # Initialize coefficients arrays
        c0, c1, c2, c3 = [0] * num_segments, [0] * num_segments, [0] * num_segments, [0] * num_segments

        # Calculate Hermite coefficients for each segment
        for i in range(num_segments):
            self.hermite_cubic_to_power_cubic(
                x[i], y[i], 0,            # Start point and derivative
                x[i + 1], y[i + 1], 0,    # End point and derivative
                c0, c1, c2, c3, i
            )

        # Initialize desired torque array
        torqueDesired = np.zeros_like(gaitCycle)

        # Compute torque for each point in the gait cycle
        for i, gait_val in enumerate(gaitCycle):
            x_val = (gait_val - x[0]) % 100 + x[0]

            if x[0] < x_val < x[-1]:
                for k in range(num_segments):
                    if x[k] <= x_val <= x[k + 1]:
                        torqueDesired[i] = self.evaluate_hermite(x_val - x[k], c0[k], c1[k], c2[k], c3[k])
                        break

        # Save x and y points for scatter plot
        torque_dots = (x, y)

        return torqueDesired, torque_dots
