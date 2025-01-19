import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as BuiltinTime
from signal import signal, SIGINT

class RosTimePublisher(Node):
    def __init__(self):
        super().__init__('ros_time_publisher')
        # Create a publisher on the /ros_time topic
        self.publisher_ = self.create_publisher(BuiltinTime, '/ros_time', 10)
        # Set up a timer to publish the message at regular intervals
        timer_period = 0.1  # Publish every 1 second
        self.timer = self.create_timer(timer_period, self.publish_ros_time)
        self.get_logger().info("ROS Time Publisher Node has started.")

    def publish_ros_time(self):
        # Get the current ROS time
        now = self.get_clock().now().to_msg()
        # Publish the current ROS time
        self.publisher_.publish(now)
        self.get_logger().info(f"Published ROS Time: {now.sec}.{now.nanosec}")

def main(args=None):
    rclpy.init(args=args)
    node = RosTimePublisher()

    def handle_interrupt(signum, frame):
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    signal(SIGINT, handle_interrupt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted, shutting down...')
    finally:
        node.destroy_node()
        if rclpy.ok():  # Shutdown only if not already shut down
            rclpy.shutdown()

if __name__ == '__main__':
    main()
