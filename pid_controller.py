#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

class PIDController(Node):

    # Constructor: creates subscriptions & publisher and defines variables that save data between calls
    def __init__(self, **kwargs):
        super().__init__("pid_control", **kwargs)
        self.create_subscription(PoseStamped, "/goal", self.__goal_callback, 1)
        self.create_subscription(Odometry, "/odom", self.__odom_callback, 1)

        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.Kp = 1.0 # Proportional gain
        self.Ki = 0.0 # Integral gain
        self.Kd = 0.0 # Derivative gain

        self.integral = 0.0
        self.previous_error = 0.0


    # Creates callback triggered when new message published on /goal
    def __goal_callback(self, msg: PoseStamped):
        self.goal_position = msg.pose.position.x

    # Creates callback triggered when new message published on /odom. Also calculates time since last callback
    def __odom_callback(self, msg: Odometry):
        self.odom_position = msg.pose.pose.position.x

        # Calculate dt (time since last callback)
        now = self.get_clock().now()
        if not hasattr(self, "_last_time"):
            self._last_time = now
            return

        dt = (now - self._last_time).nanoseconds / 1e9 # seconds
        self._last_time = now

        # Only compute if we have a goal position
        if hasattr(self, "goal_position"):
            speed = self.compute_pid(self.goal_position, self.odom_position, dt)
            cmd = Twist()
            cmd.linear.x = speed
            self.cmd_vel_publisher.publish(cmd)

    # Calculates correct control output; in this case, the robot's necessary velocity
    def compute_pid(self, setpoint, measured_value, dt):
        error = setpoint - measured_value # How far away from the target are we?
        P = self.Kp * error # Farther away = move faster
        self.integral += error * dt # Keep track of error over time
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0 # How fast is the error changing?

        output = P + (self.Ki * self.integral) + (self.Kd * derivative) # Add everything together: P- move toward target, I: fix leftover offset, D: smooth it so we don't overshoot

        self.previous_error = error # Saves current error so next time we can compute the derivative
        return output

# Main loop that initializes ROS2, sets up subscriptions and publisher; starts listening for messages
def main(): 
    rclpy.init()
    node = PIDController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()


