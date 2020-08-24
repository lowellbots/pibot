class PID:
    def __init__(self, Kff=0.0, Kp=1.0, Ki=0.0, Kd=0.0, setpoint = 0.0, min_out=None, max_out=None):
        self.Kff = Kff
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.setpoint = setpoint

        self.min = min_out
        self.max = max_out

        self.integral = 0.0
        self.pv_old = None
        self.last_output = 0

    def __call__(self, pv, dt):
        # Calculate error
        error = self.setpoint - pv
        
        # Feed forward term
        feed_forward = self.Kff * self.setpoint
        
        # Calculate p term
        proportional = self.Kp * error
        
        # Calculate i term and clamp to prevent windup
        self.integral += self.Ki * error * dt
        self.integral = clamp(self.integral, self.max, self.min)
        
        # Calculate d term if this isn't the first call
        if self.pv_old is not None: 
            dpv = pv - self.pv_old   
            derivative = self.Kd * dpv / dt
        else:
            derivative = 0.0

        # Store pv for the next d term
        self.pv_old = pv

        # calculate output, clamp, and return
        output = feed_forward + proportional + self.integral + derivative
        output = clamp(output, self.max, self.min)
        
        # smooth output
        # output = output*0.1 + self.last_output*0.9
        self.last_output = output
        return output, (feed_forward, proportional, self.integral, derivative)

def clamp(value, max_val, min_val=0.0):
    if max_val is not None and value > max_val:
        return max_val
    elif min_val is not None and value < min_val:
        return min_val
    return value

