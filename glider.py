import math
class Glider:
    def __init__(self, timesampling=1, maxVeloDes=0.1, arctan=20):
        self.depthNow=0
        self.depthDes=0
        self.lastDepth=0
        self.veloNow=0
        self.lastVelo=0
        self.veloDes=0
        self.F=0
        self.BE_input=350
        self.BE_now=self.BE_input
        self.t=timesampling
        self.last_error=0
        self.integral=0
        self.pitchNow=0
        self.MM_input=50
        self.MM_now=self.MM_input
        self.mass= 86.5
        # in meters
        self.maxVeloDes = maxVeloDes
        self.minVeloDes = 0
        self.CGCB_y = (0.0113)
        # error 0.2 meters
        self.minError = 0.2
        self.maxError = (arctan*maxVeloDes)-self.minVeloDes
        self.minmaxErrorDiff = self.maxError-self.minError
        self.errorDepth = 0
        # pitch variable initialization
        self.q = 0
        self.theta = 0
        self.qdot=0
        # depth_rate
        self.depth_rate = 0


    def __str__(self):
        return "This is Glider Lib for calculation Depth and PID controller"
    
    # return pitch from inputMM
    def pitchMechanism(self, inputMM):
        self.MM_input = inputMM
        W = 687.96
        self.qdot = (-100.85 * abs(self.q) * self.q - 0.0113 * W * math.sin(self.theta) - (4/70.2) * 7.5e-2 * W * (250-self.MM_input)/250 * math.cos(self.theta))/28.25
        self.q += self.qdot*self.t
        self.theta += self.q*self.t
        # # pitch mechanism
        self.pitchNow = self.theta * 180/math.pi
        return self.pitchNow
            
    def depthMechanism(self, inputBE):
        self.BE_input = inputBE
        # if (self.BE_input - self.BE_now) > 15:
        #     self.BE_now += 15
        # elif (self.BE_input - self.BE_now) < -15:
        #     self.BE_now -= 15
        # else:
        #     self.BE_now = self.BE_input
        k=307
        self.F = 1000*9.8*(self.BE_input-350)/1_000_000 - k * abs(self.veloNow) * self.veloNow
        a = self.F / 70
        print("Acceleration:",a)
        self.veloNow += (a * self.t)
        print("Velo Now is: ", self.veloNow, "\n")
        self.depthNow += self.veloNow * self.t

        return self.depthNow, self.BE_input, self.veloNow

    def calculatePID(self, setPoint, process_variable, Kp, Ki, Kd):
        error = setPoint - process_variable
        # Proportional term
        proportional = Kp * error
        # Integral term
        self.integral += error * self.t
        integral = Ki * self.integral
        # Derivative term
        derivative = Kd * (error - self.last_error) / self.t
        self.last_error = error
        # Calculate the control signal
        control_signal = proportional + integral + derivative
        return control_signal
    
    def calculate_desVelo(self, errorDepth):
        if (abs(errorDepth) > self.maxError):
            veloDes = self.maxVeloDes * (errorDepth)/abs(errorDepth)
            # print("Velo menuju {}".format(veloDes))
        elif (abs(errorDepth) < self.minError):
            veloDes = 0
            # print("Menuju nol velonya")
        else:
            veloDes = self.maxVeloDes/self.minmaxErrorDiff * \
                (errorDepth)-((self.minError*self.maxVeloDes/self.minmaxErrorDiff)
                              * (errorDepth)/abs(errorDepth))
            # print("Transient velo")
        print("VeloDes: ", veloDes)
        print("Error Depth: ", errorDepth)
        # veloDes in metres
        return veloDes
    
    # deprecated, not used
    
    # def calculateMPC(self, setPoint, process_variable, horizon):
    #     control_signal = 0

    #     # Step 1: Define the MPC objective function and constraints
    #     # Define the objective function to optimize, including desired setpoints and penalties for deviations
    #     # Define constraints on the system, such as input limits, state limits, or safety constraints
    #     objective = 0
    #     weights = [1, 1]
    #     # calculating error
    #     error = setPoint - process_variable
    #     objective += weights[0] * error**2
    #     # Control effort objective
    #     objective += weights[1] * control_signal**2

    #     # Define constraints on the system, such as input limits, state limits, or safety constraints
    #     constraints = []

    #     # Add input limits constraint
    #     input_min = 0  # Example minimum control signal value
    #     input_max = 700   # Example maximum control signal value
    #     constraints.append({'type': 'ineq', 'fun': lambda u: input_min - u})
    #     constraints.append({'type': 'ineq', 'fun': lambda u: u - input_max})

    #     # Step 2: Generate the prediction model
    #     # Use the glider's mathematical model to predict the future behavior over the horizon
    #     # Incorporate the process variable measurements to update the predicted states
    #     prediction_horizon = range(horizon)
    #     predicted_states = []   
        
    #     for i in prediction_horizon:
    #         # Use the glider's mathematical model to predict the future behavior
    #         # based on the current state and the control signal
    #         # Incorporate the process variable measurements to update the predicted states

    #         # Example: simple prediction model assuming linear dynamics
    #         predicted_state = process_variable + self.lastVelo * self.t
    #         predicted_states.append(predicted_state)

    #         # Update the process variable for the next iteration
    #         process_variable = predicted_state

    #     # Step 3: Solve the optimization problem
    #     # Use an optimization solver (e.g., quadratic programming, nonlinear programming) to solve the MPC problem
    #     # Formulate the optimization problem using the objective function, constraints, and predicted model
    #     # Obtain the optimal control signal that minimizes the objective function while satisfying the constraints
        
    #     initial_guess = 0  # Initial guess for the control signal
    #     optimization_result = minimize(self.objectiveFunction, initial_guess,
    #                                method='SLSQP', constraints=self.constraints)

    #     # Step 4: Extract the first control signal from the optimization solution
    #     # Retrieve the optimal control signal for the current time step from the optimization solution
    #     if optimization_result.success:
    #         control_signal = optimization_result.x[0]
    #     else:
    #         print("Optimization failed:", optimization_result.message)  
            
    #     # Step 5: Apply additional control action (if needed) optional
    #     # Optionally, you can combine the MPC control signal with the PID control signal to enhance performance
    #     # Adjust the control signal based on system-specific considerations or controller blending techniques
        
    #     # pid_control_signal = self.calculatePID(setPoint, process_variable, Kp, Ki, Kd)
    #     # control_signal += pid_control_signal
        
        
    #     # Step 6: Return the final control signal
    #     return control_signal
    
    # def objectiveFunction(self, control_signal):
    #     # Define the objective function to be minimized
    #     # Compute the objective value based on the predicted states and control signal

    #     objective = 0

    #     # Example: tracking error objective
    #     error = setPoint - predicted_states[0]
    #     objective += weights[0] * error**2

    #     # Example: control effort objective
    #     objective += weights[1] * control_signal**2

    #     return objective

    # def constraints(self, control_signal):
    #     # Define the constraints to be satisfied
    #     # Evaluate the constraint functions based on the predicted states and control signal

    #     constraints = []

    #     # Example: input limits constraint
    #     constraints.append(input_min - control_signal)
    #     constraints.append(control_signal - input_max)

    #     return constraints