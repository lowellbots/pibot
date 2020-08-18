from math import pi, sin, cos

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler

from romi_ros import Romi, PID

class RomiBase:
    def __init__(self, nh):
        rospy.on_shutdown(self.on_shutdown)

        self.rate = rospy.Rate(20)

        self.base_frame_id = rospy.get_param('base/frame_id', 'base_link')
        self.wheel_diameter = rospy.get_param('base/wheel_diameter', 0.07) #wheel_diameter in meters
        self.wheel_circumference = self.wheel_diameter * pi
        self.wheelbase = rospy.get_param('base/wheelbase', .141) #wheelbase in meters
        self.encoder_cpr = rospy.get_param('encoder/counts_per_rev', 12.0*120.0) #number of encoder counts *per wheel revolution*
        self.meters_per_count = self.wheel_circumference / self.encoder_cpr #distance a point on the diameter of the wheel would travel per encoder count

        # Encoder directions:
        # +1 means an increase in encoder tick represents the wheel rotating toward the front of the chassis
        self.left_encoder_direction = rospy.get_param('encoder/left/direction', 1)
        self.right_encoder_direction = rospy.get_param('encoder/right/direction', 1)

        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1) 

        self.romi = Romi()
        self.romi.reset_encoders()
        self.romi.beepBoop()

        self.t_old = None
        self.left_wheel_old_pos = 0
        self.right_wheel_old_pos = 0

        self.x = 0
        self.y = 0
        self.heading = 0

        self.cmd_vel = Twist()
        self.linear_pid = PID(Kp = 300.0, Ki = 600.0, Kd = -100.0, min_out = -200, max_out = 200)
        self.angular_pid = PID(Kp = 50.0, Ki = 0, Kd = 0.0, min_out = -100, max_out = 100)

        self.encoders_wraps = [0, 0]
        self.encoders_old = [0, 0]

    def run(self):
        while not rospy.is_shutdown():
            self.loop()

    def loop(self):

        t = rospy.Time.now()
        #should probably also publish battery voltage here....

        #If we have an old timestamp from which to calculate dt
        if self.t_old is not None:

            # Calculate time since last loop iteration
            dt = (t - self.t_old).to_sec()

            # Grab current encoder counts
            encoders = list(self.romi.read_encoders())

            # Check for int_16 overflow and unwrap
            for i in range(len(encoders)):
                d_encoder = encoders[i] - self.encoders_old[i]
                if d_encoder < -32768:
                    self.encoders_wraps[i] += 1
                elif d_encoder > 32768:
                    self.encoders_wraps[i] -= 1
                # Store encoders for unwrapping
                self.encoders_old[i] = encoders[i]
                # Unwrap encoders
                encoders[i] += self.encoders_wraps[i] * 65536

            self.calculate_odometry(tuple(encoders), dt)

            self.motor_pid(dt)

        # Update the old timestamp with the current time.
        self.t_old = t
        # Sleep for a bit.
        self.rate.sleep()

    def calculate_odometry(self, encoders, dt):
        # Convert encoder ticks to wheel distance travelled since startup
        left_wheel_cur_pos, right_wheel_cur_pos = [encoder*self.meters_per_count for encoder in encoders]

        # Estimate per wheel velocities
        left_wheel_est_vel = self.left_encoder_direction*(left_wheel_cur_pos - self.left_wheel_old_pos)/dt
        right_wheel_est_vel = self.right_encoder_direction*(right_wheel_cur_pos - self.right_wheel_old_pos)/dt

        # Estimate vehicle motion based on wheel velocities
        self.linear_vel = (right_wheel_est_vel + left_wheel_est_vel) *0.5
        self.angular_vel  = (right_wheel_est_vel - left_wheel_est_vel) / self.wheelbase

        # K.I.S.S. Odometry integration.  Since our counter is quite fast, let's assume we haven't missed any counts
        # and it's safe to simply use addition.  No more Trailing Average Filter + Runge Kutta for you!
        self.heading += self.angular_vel*dt

        delta_x = self.linear_vel * cos(self.heading) * dt
        self.x += delta_x

        delta_y = self.linear_vel * sin(self.heading) * dt
        self.y += delta_y


        # Stuff into an odometry message and *shipit*
        odom_msg = Odometry()
        odom_msg.header.frame_id = self.base_frame_id
        odom_msg.header.stamp = rospy.Time.now()

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y

        q = quaternion_from_euler(0, 0, self.heading)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = self.linear_vel
        odom_msg.twist.twist.angular.z = self.angular_vel

        self.odom_pub.publish(odom_msg)

        # Update the old positions with current values
        self.left_wheel_old_pos = left_wheel_cur_pos
        self.right_wheel_old_pos = right_wheel_cur_pos

    def motor_pid(self, dt):
        # This is a pid loop which uses the cmd_vel and odom velocities
        # I'm not sure if it's better to pid off of these directly or to
        # compute the appropriate wheel speeds and pid them individually
        # but this seems easier and perhaps more tolerant to differences
        # between wheel responsivities, I'm not sure...

        # Update setpoints (should we be doing this in the callback?)
        self.linear_pid.setpoint = self.cmd_vel.linear.x
        self.angular_pid.setpoint = self.cmd_vel.angular.z
        
        # Calculate pid values using odom linear and angular velocities
        linear_drive = self.linear_pid(self.linear_vel, dt)
        angular_drive = self.angular_pid(self.angular_vel, dt)

        # Make sure we have values to write
        if linear_drive is not None and angular_drive is not None:
            # Conver pid values to differential drive levels
            left = linear_drive - angular_drive
            right = linear_drive + angular_drive

            # Set the robot motor levels
            self.romi.motors(int(left), int(right))

    def cmd_vel_callback(self, data):
        self.cmd_vel = data
        rospy.loginfo("Recieved: %s"%data)

    def on_shutdown(self):
        rospy.logwarn("Romi Base Shutting Down")
        self.cmd_vel_sub.unregister()
