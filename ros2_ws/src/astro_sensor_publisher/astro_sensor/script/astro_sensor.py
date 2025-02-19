import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from builtin_interfaces.msg import Time as BuiltinTime
from tf2_ros import TransformListener, TransformBroadcaster, Buffer
from signal import signal, SIGINT
from nav_msgs.msg import Odometry


class SensorSynchronization(Node):
    def __init__(self):
        super().__init__("sensor_sync_node")

        # Subscribers
        self.imu_subscriber = self.create_subscription(
            Imu, "/imu_raw", self.imu_callback, 10
        )
        self.laser_subscriber = self.create_subscription(
            NavSatFix, "/gnss_data_raw", self.gnss_callback, 10
        )
        self.time_subscriber = self.create_subscription(
            BuiltinTime, "/ros_time", self.time_callback, 10
        )
        # Subscriber for Odometry messages (odom frame)
        self.odom_subscriber = self.create_subscription(
            Odometry, "/odom_raw", self.odom_callback, 10
        )

        # Publishers
        self.imu_raw_publisher = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.imu_publisher = self.create_publisher(Imu, "/imu", 10)
        self.gnss_publisher = self.create_publisher(NavSatFix, "/gnss", 10)
        # Publisher for synchronized Odometry
        self.synchronized_odom_publisher = self.create_publisher(Odometry, "/odom", 10)

        self.latest_odom_time = None
        self.latest_odom_pose = None
        self.latest_odom_twist = None

        # Time and message tracking
        self.latest_imu_time = None
        self.latest_gnss_time = None
        self.latest_odom_time = None
        self.latest_ros_time = None
        self.previous_ros_time = None
        self.latest_imu_message = None
        self.latest_gnss_message = None
        self.ros_time = None
        self.alpha = 0.1  # Smoothing factor (0 < alpha <= 1)
        self.filtered_angular_velocity = None
        self.filtered_linear_acceleration = None

    def imu_callback(self, msg: Imu):
        self.latest_imu_time = msg.header.stamp
        self.latest_imu_message = msg
        # print("imu_callback called")
        if self.ros_time:
            self.sync_imu()

    def odom_callback(self, msg: Odometry):
        # Update the latest odom time
        self.latest_odom_time = msg.header.stamp
        self.latest_odom_pose = msg.pose  # Assign pose data here
        self.latest_odom_twist = msg.twist  # Assign pose data here
        if self.ros_time:
            self.sync_odom()

    def gnss_callback(self, msg: NavSatFix):
        self.latest_gnss_time = msg.header.stamp
        self.latest_gnss_message = msg
        if self.ros_time:
            self.sync_gnss()

    def time_callback(self, msg: BuiltinTime):
        # print("time_callback called")
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
            if (self.latest_ros_time.sec > self.previous_ros_time.sec) or (
                self.latest_ros_time.sec == self.previous_ros_time.sec
                and self.latest_ros_time.nanosec > self.previous_ros_time.nanosec
            ):
                synced_time = self.latest_ros_time

            synchronized_gnss = NavSatFix()
            synchronized_gnss.header.stamp = synced_time
            self.gnss_publisher.publish(synchronized_gnss)

            self.previous_ros_time = self.latest_ros_time
            # self.get_logger().info(f">>Synchronized GNSS time: {synced_time}")

    def smooth_imu_data(self, new_angular_velocity, new_linear_acceleration):
        """Applies an Exponential Moving Average (EMA) filter to smooth IMU data."""
        if self.filtered_angular_velocity is None:
            self.filtered_angular_velocity = new_angular_velocity
            self.filtered_linear_acceleration = new_linear_acceleration
        else:
            # Apply EMA smoothing
            self.filtered_angular_velocity.x = (
                self.alpha * new_angular_velocity.x
                + (1 - self.alpha) * self.filtered_angular_velocity.x
            )
            self.filtered_angular_velocity.y = (
                self.alpha * new_angular_velocity.y
                + (1 - self.alpha) * self.filtered_angular_velocity.y
            )
            self.filtered_angular_velocity.z = (
                self.alpha * new_angular_velocity.z
                + (1 - self.alpha) * self.filtered_angular_velocity.z
            )

            self.filtered_linear_acceleration.x = (
                self.alpha * new_linear_acceleration.x
                + (1 - self.alpha) * self.filtered_linear_acceleration.x
            )
            self.filtered_linear_acceleration.y = (
                self.alpha * new_linear_acceleration.y
                + (1 - self.alpha) * self.filtered_linear_acceleration.y
            )
            self.filtered_linear_acceleration.z = (
                self.alpha * new_linear_acceleration.z
                + (1 - self.alpha) * self.filtered_linear_acceleration.z
            )

        return self.filtered_angular_velocity, self.filtered_linear_acceleration

    def sync_imu(self):
        if self.latest_imu_time and self.ros_time:
            if not self.previous_ros_time:
                self.previous_ros_time = self.latest_ros_time
                self.get_logger().info("Initialized ROS time tracking.")
                return

            # Initialize synced_time
            synced_time = BuiltinTime()
            if (self.latest_ros_time.sec > self.previous_ros_time.sec) or (
                self.latest_ros_time.sec == self.previous_ros_time.sec
                and self.latest_ros_time.nanosec > self.previous_ros_time.nanosec
            ):
                synced_time = self.latest_ros_time

                synchronized_imu = Imu()
                synchronized_imu.header.stamp = synced_time
                synchronized_imu.header.frame_id = "imu_link"
                synchronized_imu.orientation = self.latest_imu_message.orientation
                smoothed_angular_velocity, smoothed_linear_acceleration = (
                    self.smooth_imu_data(
                        self.latest_imu_message.angular_velocity,
                        self.latest_imu_message.linear_acceleration,
                    )
                )
                synchronized_imu.angular_velocity = smoothed_angular_velocity
                synchronized_imu.linear_acceleration = smoothed_linear_acceleration
                self.imu_publisher.publish(synchronized_imu)
                self.imu_raw_publisher.publish(synchronized_imu)
                self.previous_ros_time = self.latest_ros_time
                # self.get_logger().info(f">>Synchronized IMU time: {synced_time}")

    def sync_odom(self):
        if self.latest_odom_time and self.ros_time:
            if not self.previous_ros_time:
                self.previous_ros_time = self.latest_ros_time
                self.get_logger().info("Initialized ROS time tracking.")
                return

            # Initialize synced_time
            synced_time = BuiltinTime()
            if (self.latest_ros_time.sec > self.previous_ros_time.sec) or (
                self.latest_ros_time.sec == self.previous_ros_time.sec
                and self.latest_ros_time.nanosec > self.previous_ros_time.nanosec
            ):
                synced_time = self.latest_ros_time

                synchronized_odom = Odometry()
                synchronized_odom.header.stamp = (
                    synced_time  # Update with the synchronized time
                )
                synchronized_odom.header.frame_id = "odom"
                synchronized_odom.child_frame_id = "base_link"
                # You can copy over the rest of the data from the original odometry message if needed
                synchronized_odom.pose = (
                    self.latest_odom_pose
                )  # Example: copy pose data
                synchronized_odom.twist = self.latest_twist  # Example: copy twist data
                self.synchronized_odom_publisher.publish(synchronized_odom)

                self.previous_ros_time = self.latest_ros_time
                self.get_logger().info(f">>Synchronized Odom time: {synced_time}")


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
        node.get_logger().info("Node interrupted, shutting down...")
    except rclpy.executors.ExternalShutdownException:
        node.get_logger().warn("External shutdown detected.")
    finally:
        node.destroy_node()
        if rclpy.ok():  # Shutdown only if not already shut down
            rclpy.shutdown()


if __name__ == "__main__":
    main()
