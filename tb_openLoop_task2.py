import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import math
import threading

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        
        # Publishers and Subscribers
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 50)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 50)
        
        self.timer_period = 0.05  

        # Get user inputs
        self.y_goal = float(input("Enter the goal position (m): "))
        self.v_max =float(input("Enter the maximum velocity (m/s): "))
        self.a_acc = float(input("Enter the acceleration (m/s^2): "))
        self.a_dec = self.a_acc
        
        # distance for acceleration phase
        self.t1 = self.v_max / self.a_acc  
        self.x1 = 0.5 * self.a_acc * self.t1 ** 2  
        
        # Distance left for constant velocity
        self.x3 = (self.v_max ** 2) / (2 * self.a_dec)  
        self.x2 = self.y_goal - (self.x1 + self.x3) 
        self.t2 = self.x2 / self.v_max  
        
        
        # Deceleration phase time
        self.t3 = self.v_max / self.a_dec  

        self.get_logger().info(f't1 {self.t1:.2f}s, t2 {self.t2:.2f}s, t3 {self.t3:.2f}s')

        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Start time
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Initialize position and history tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.position_history = []
        self.time_history = []

        # Start plot thread for real-time plotting
        plot_thread = threading.Thread(target=self.plot_data)
        plot_thread.start()

    def timer_callback(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed_time = current_time - self.start_time  
        move = Twist()

        # Phase 1: Accelerating
        if elapsed_time < self.t1:
            move.linear.x = self.a_acc * elapsed_time
            self.publisher_.publish(move)
            self.get_logger().info(f'Phase 1: Accelerating. Velocity: {move.linear.x:.2f} m/s.  Position: {self.current_x:.2f} m')

        # Phase 2: Constant velocity
        elif self.t1 <= elapsed_time < (self.t1 + self.t2):
            move.linear.x = self.v_max
            self.publisher_.publish(move)
            self.get_logger().info(f'Phase 2: Constant velocity. Velocity: {move.linear.x:.2f} m/s.  Position: {self.current_x:.2f} m')

        # Phase 3: Decelerating
        elif (self.t1 + self.t2) <= elapsed_time < (self.t1 + self.t2 + self.t3):
            time_since_t2 = elapsed_time - (self.t1 + self.t2)
            move.linear.x = max(self.v_max - self.a_dec * time_since_t2, 0.0)  # Ensure velocity doesn't drop below 0
            self.publisher_.publish(move)
            self.get_logger().info(f'Phase 3: Decelerating. Velocity: {move.linear.x:.2f} m/s.  Position: {self.current_x:.2f} m')

        # Stop the robot when the goal is reached
        if elapsed_time >= (self.t1 + self.t2 + self.t3) or self.current_x >= self.y_goal:
            move.linear.x = 0.0
            self.publisher_.publish(move)
            self.get_logger().info(f'Goal reached at y = {self.current_x:.2f} m. Stopping the robot.')
            self.destroy_timer(self.timer)

            
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed_time = current_time - self.start_time
        self.time_history.append(elapsed_time)
        self.position_history.append((self.current_x, self.current_y))
        
        # Stop the robot if it exceeds the target position
        if self.current_x >= self.y_goal:
            move = Twist()
            move.linear.x = 0.0
            self.publisher_.publish(move)
            self.get_logger().info(f'Overshot target, stopping the turtlebot. Current y = {self.current_x:.2f} m')
            self.destroy_timer(self.timer)
        
        
        
    def plot_data(self):
        plt.ion()  
        fig, ax = plt.subplots()

        while rclpy.ok():
            if self.position_history:
                ax.clear()
                x_data, y_data = zip(*self.position_history)
                ax.plot(self.time_history, x_data,label="TurtleBot Path")
                ax.set_xlabel('time (s)')
                ax.set_ylabel('Position (m) ')
                ax.set_title('TurtleBot Path')
                ax.legend()
                plt.pause(0.01)  
        plt.ioff()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        print("KeyboardInterrupt: Stopping...")
    finally:
        print("Destroying node and shutting down...")
        velocity_publisher.destroy_node()
        rclpy.shutdown()
        print("Shutdown complete.")

if __name__ == '__main__':
    main()

