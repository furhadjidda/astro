import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, TransformBroadcaster, Buffer
from geometry_msgs.msg import TransformStamped

class TimeSynchronizerNode(Node):
    def __init__(self):
        super().__init__('time_synchronizer_node')

        # Subscriber for Odometry messages (odom frame)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Subscriber for LaserScan messages (laser frame)
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # Initialize the Buffer and TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  # Pass node and buffer to listener
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publisher for synchronized Odometry
        self.synchronized_odom_publisher = self.create_publisher(Odometry, '/odom', 10)


        self.latest_odom_time = None
        self.latest_laser_time = None
        self.latest_odom_pose = None
        self.latest_odom_twist = None

    def odom_callback(self, msg: Odometry):
        # Update the latest odom time
        self.latest_odom_time = msg.header.stamp
        self.latest_odom_pose = msg.pose  # Assign pose data here
        self.latest_twist = msg.twist  # Assign pose data here
        self.get_logger().info(f"Received Odometry with timestamp: {self.latest_odom_time}")

        if self.latest_laser_time:
            self.sync_time()

    def laser_callback(self, msg: LaserScan):
        # Update the latest laser time
        self.latest_laser_time = msg.header.stamp
        self.get_logger().info(f"Received LaserScan with timestamp: {self.latest_laser_time}")

        if self.latest_odom_time:
            self.sync_time()

    def sync_time(self):
        # Synchronize odom timestamp with laser timestamp
        if self.latest_odom_time and self.latest_laser_time:
            synced_time = self.latest_laser_time

            # Create a new Odometry message with the synchronized time
            synchronized_odom = Odometry()
            synchronized_odom.header.stamp = synced_time  # Update with the synchronized time
            synchronized_odom.header.frame_id = "odom"
            synchronized_odom.child_frame_id = "base_link"

            # You can copy over the rest of the data from the original odometry message if needed
            synchronized_odom.pose = self.latest_odom_pose  # Example: copy pose data
            synchronized_odom.twist = self.latest_twist # Example: copy twist data

            # Publish the synchronized odometry message
            self.synchronized_odom_publisher.publish(synchronized_odom)
            # Creating a transform to broadcast from odom to laser frame


            transform = TransformStamped()
            transform.header.stamp = synced_time
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'laser'

            # Example translation and rotation (you may adjust based on your setup)
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0

            # Broadcasting the transform
            self.tf_broadcaster.sendTransform(transform)

            self.get_logger().info(f"Synchronized time: {synced_time} for Odometry and Laser")

def main(args=None):
    rclpy.init(args=args)
    node = TimeSynchronizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
