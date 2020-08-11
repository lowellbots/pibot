from math import pi, sin, cos

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler

from romi_ros import Romi

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
            encoders = self.romi.read_encoders()

            self.calculate_odometry(encoders, dt)

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


    def cmd_vel_callback(self):
        return True

    def on_shutdown(self):
        rospy.logwarn("Romi Base Shutting Down")