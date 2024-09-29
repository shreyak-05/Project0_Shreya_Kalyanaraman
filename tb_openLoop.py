import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import threading


class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')

        # Publisher to control robot's velocity
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 50)

        # Subscriber to get robot's odometry
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 50)

        self.timer_period = 0.1  # seconds
        self.x = 0
        self.y = float(input("Add goal coordinate y: "))
        print(f"Destination coordinates = ({self.x}, {self.y})")
        self.v = float(input("Enter velocity: "))
        self.t = self.y / self.v
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Capture start time
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.position_history = []  # To store position history for plotting
        self.time_history = []
        # Create a separate thread for plotting
        plot_thread = threading.Thread(target=self.plot_data)
        plot_thread.start()
        
        

    def timer_callback(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed_time = current_time - self.start_time
        if elapsed_time < self.t:
            msg = Twist()
            msg.linear.x = self.v
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info(f'Moving... Time elapsed: {elapsed_time:.2f} seconds')
        else:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info('Reached destination, stopping the robot.')
            self.destroy_timer(self.timer)

    def odom_callback(self, msg):
        # Update current position based on odometry data
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed_time = current_time - self.start_time
        self.time_history.append(elapsed_time)
        # Store position for plotting
        self.position_history.append((current_x, current_y))
        # self.get_logger().info(self.position_history)
        
    def plot_data(self):
        plt.ion()  # Interactive mode on
        fig, ax = plt.subplots()

        while rclpy.ok():
            if self.position_history:
                # Clear the plot and replot with updated position history
                ax.clear()
                x_data, y_data = zip(*self.position_history)
                ax.plot(self.time_history, x_data,label="TurtleBot Path")
                ax.set_xlabel('time (s)')
                ax.set_ylabel('X Position (m) ')
                ax.set_title('TurtleBot Path')
                ax.legend()
                plt.pause(0.01)  # Small pause to update the graph

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
