import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from builtin_interfaces.msg import Time as BuiltinTime
from tf2_ros import TransformListener, TransformBroadcaster, Buffer
from signal import signal, SIGINT

class SensorSynchronization(Node):
    def __init__(self):
        super().__init__('sensor_sync_node')

        # Subscribers
        self.odom_subscriber = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)
        self.laser_subscriber = self.create_subscription(
            NavSatFix, '/gnss_raw', self.gnss_callback, 10)
        self.time_subscriber = self.create_subscription(
            BuiltinTime, '/ros_time', self.time_callback, 10)

        # Publishers
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        self.gnss_publisher = self.create_publisher(NavSatFix, '/gnss', 10)

        # Time and message tracking
        self.latest_imu_time = None
        self.latest_gnss_time = None
        self.latest_ros_time = None
        self.previous_ros_time = None
        self.latest_imu_message = None
        self.latest_gnss_message = None
        self.ros_time = None

    def imu_callback(self, msg: Imu):
        self.latest_imu_time = msg.header.stamp
        self.latest_imu_message = msg
        print('imu_callback called')
        if self.ros_time:
            self.sync_imu()

    def gnss_callback(self, msg: NavSatFix):
        self.latest_gnss_time = msg.header.stamp
        self.latest_gnss_message = msg
        if self.ros_time:
            self.sync_gnss()

    def time_callback(self, msg: BuiltinTime):
        print('time_callback called')
        self.ros_time = msg
        self.latest_ros_time = msg

    def sync_gnss(self):
        if self.latest_gnss_time and self.ros_time:
            if not self.previous_ros_time:
                self.previous_ros_time = self.latest_ros_time
                self.get_logger().info("Initialized ROS time tracking.")
                return

            # Initialize synced_time
            synced_time = BuiltinTime()
            if (self.latest_ros_time.sec > self.previous_ros_time.sec) or \
               (self.latest_ros_time.sec == self.previous_ros_time.sec and
                self.latest_ros_time.nanosec > self.previous_ros_time.nanosec):
                synced_time = self.latest_ros_time

            synchronized_gnss = NavSatFix()
            synchronized_gnss.header.stamp = synced_time
            self.gnss_publisher.publish(synchronized_gnss)

            self.previous_ros_time = self.latest_ros_time
            self.get_logger().info(f">>Synchronized GNSS time: {synced_time}")

    def sync_imu(self):
        if self.latest_imu_time and self.ros_time:
            if not self.previous_ros_time:
                self.previous_ros_time = self.latest_ros_time
                self.get_logger().info("Initialized ROS time tracking.")
                return

            # Initialize synced_time
            synced_time = BuiltinTime()
            if (self.latest_ros_time.sec > self.previous_ros_time.sec) or \
               (self.latest_ros_time.sec == self.previous_ros_time.sec and
                self.latest_ros_time.nanosec > self.previous_ros_time.nanosec):
                synced_time = self.latest_ros_time

                synchronized_imu = Imu()
                synchronized_imu.header.stamp = synced_time
                self.imu_publisher.publish(synchronized_imu)

                self.previous_ros_time = self.latest_ros_time
                self.get_logger().info(f">>Synchronized IMU time: {synced_time}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorSynchronization()

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
