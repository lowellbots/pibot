class PID:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, setpoint = 0.0, min=None, max=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.setpoint = setpoint

        self.min = min
        self.max = max

        self.integral = 0.0
        self.pv_old = None

    def __call__(self, pv, dt):
        # Calculate error
        error = self.setpoint - pv

        # Calculate p term
        proportional = self.Kp * error
        
        # Calculate i term and clamp to prevent windup
        self.integral += self.Ki * error * dt
        self.integral = clamp(self.integral, max, min)
        
        # Calculate d term if this isn't the first call
        if self.pv_old is not None: 
            dpv = pv - self.pv_old   
            derrivative = self.Kd * dpv / dt
        else:
            derrivative = 0.0

        # Store pv for the next d term
        self.pv_old = pv

        # calculate output, clamp, and return
        output = proportional + self.integral + derrivative
        output = clamp(output, self.min, self.max)

def clamp(value, max, min=0.0):
    if max is not None and value > max:
        return max
    elif min is not None and value < min:
        return min
    return value

