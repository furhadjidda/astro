import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from signal import signal, SIGINT
from nav_msgs.msg import Odometry


class SensorSynchronization(Node):
    def __init__(self):
        super().__init__("sensor_sync_node")

        # Subscribers
        self.imu_subscriber = self.create_subscription(
            Imu, "/bno055_imu_raw", self.imu_callback, 10
        )
        self.laser_subscriber = self.create_subscription(
            NavSatFix, "/ublox_gnss_raw", self.gnss_callback, 10
        )
        # Subscriber for Odometry messages (odom frame)
        self.odom_subscriber = self.create_subscription(
            Odometry, "/odom_raw", self.odom_callback, 10
        )

        # Publishers
        self.imu_raw_publisher = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.imu_publisher = self.create_publisher(Imu, "/imu", 10)
        self.gnss_publisher = self.create_publisher(NavSatFix, "/gnss", 10)
        self.synchronized_odom_publisher = self.create_publisher(Odometry, "/odom", 10)

        self.alpha = 0.1  # Smoothing factor (0 < alpha <= 1)
        self.filtered_angular_velocity = None
        self.filtered_linear_acceleration = None

    def imu_callback(self, msg: Imu):
        now = self.get_clock().now().to_msg()

        smoothed_angular_velocity, smoothed_linear_acceleration = self.smooth_imu_data(
            msg.angular_velocity,
            msg.linear_acceleration,
        )

        imu_out = Imu()
        imu_out.header.stamp = now
        imu_out.header.frame_id = "imu_link"
        imu_out.orientation = msg.orientation
        imu_out.angular_velocity = smoothed_angular_velocity
        imu_out.linear_acceleration = smoothed_linear_acceleration
        self.imu_publisher.publish(imu_out)
        self.imu_raw_publisher.publish(imu_out)

    def odom_callback(self, msg: Odometry):
        now = self.get_clock().now().to_msg()

        odom_out = Odometry()
        odom_out.header.stamp = now
        odom_out.header.frame_id = "odom"
        odom_out.child_frame_id = "base_link"
        odom_out.pose = msg.pose
        odom_out.twist = msg.twist
        self.synchronized_odom_publisher.publish(odom_out)

    def gnss_callback(self, msg: NavSatFix):
        now = self.get_clock().now().to_msg()

        gnss_out = NavSatFix()
        gnss_out.header.stamp = now
        self.gnss_publisher.publish(gnss_out)

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
