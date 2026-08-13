#!/usr/bin/env python3
"""Stamp covariances onto the sim's IMU and odometry for robot_localization.

Neither Ignition message carries covariance at all:

    message IMU      { Header header; string entity_name; Quaternion orientation;
                       Vector3d angular_velocity; Vector3d linear_acceleration; }
    message Odometry { Header header; Pose pose; Twist twist; }

so ros_gz_bridge can only emit sensor_msgs/Imu and nav_msgs/Odometry with
all-zero covariance matrices. robot_localization raises a zero variance to 1e-9
-- near-infinite confidence -- and the consequences are not subtle: with the IMU
at 1e-9 the EKF rejected 100% of its messages on the Mahalanobis gate (squared
distance 331.99 against a 0.64 limit), and with wheel odometry also at 1e-9 the
filter copied /odom through bit-for-bit, drifting 18 degrees in yaw over nine
navigation goals while the IMU sat there holding the right answer.

This node republishes both streams onto separate topics with honest covariances,
leaving the bridge's own /imu and /odom untouched -- nav2, cartographer and the
velocity smoother read those directly and none of them look at covariance, so
they keep working even if this node is not running.

    /imu  -> /imu_with_covariance    (ekf_filter_node imu0)
    /odom -> /odom_with_covariance   (ekf_filter_node odom0)

A matrix that arrives already populated is passed through untouched, so this
node becomes a no-op if a future Gazebo release starts reporting covariance.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def diag3(variances):
    """Row-major 3x3 covariance with the given diagonal."""
    cov = [0.0] * 9
    for i, var in enumerate(variances):
        cov[i * 3 + i] = var
    return cov


def diag6(variances):
    """Row-major 6x6 covariance with the given diagonal (x y z roll pitch yaw)."""
    cov = [0.0] * 36
    for i, var in enumerate(variances):
        cov[i * 6 + i] = var
    return cov


class SimCovarianceRelay(Node):

    def __init__(self):
        super().__init__('sim_covariance_relay')

        # IMU variances follow the noise actually configured on the Ignition
        # imu_sensor in tortoisebot_ignition.gazebo, inflated to leave room for
        # sample-timing jitter that the SDF stddev does not describe:
        #   angular_velocity    stddev 2e-4 rad/s   -> 4e-8, used 1e-6
        #   linear_acceleration stddev 5e-3 m/s^2   -> 2.5e-5, used 1e-4
        # Ignition derives orientation from the link's true pose and applies no
        # noise to it, so 1e-4 (1 sigma = 0.57 deg) is a floor chosen to keep the
        # filter from becoming over-confident rather than a measured value.
        imu_orient_var = float(self.declare_parameter(
            'imu_orientation_variance', 1.0e-4).value)
        imu_gyro_var = float(self.declare_parameter(
            'imu_angular_velocity_variance', 1.0e-6).value)
        imu_accel_var = float(self.declare_parameter(
            'imu_linear_acceleration_variance', 1.0e-4).value)

        # Wheel odometry is dead reckoning through a slipping contact, so these
        # are deliberately loose. Yaw especially: it is the quantity that drifted
        # 18 degrees, and the ratio between this and imu_orientation_variance is
        # what decides whether the IMU or the wheels win the heading argument.
        odom_xy_var = float(self.declare_parameter(
            'odom_position_variance', 1.0e-2).value)
        odom_yaw_var = float(self.declare_parameter(
            'odom_yaw_variance', 5.0e-2).value)
        odom_vx_var = float(self.declare_parameter(
            'odom_linear_velocity_variance', 1.0e-3).value)
        odom_vyaw_var = float(self.declare_parameter(
            'odom_angular_velocity_variance', 3.0e-3).value)

        # Unfused axes get a large variance rather than zero: two_d_mode already
        # holds z, roll and pitch at zero, and a zero here would be read back as
        # 1e-9 by the very code path this node exists to work around.
        unused = 1.0e6

        self.imu_orientation_covariance = diag3([imu_orient_var] * 3)
        self.imu_angular_velocity_covariance = diag3([imu_gyro_var] * 3)
        self.imu_linear_acceleration_covariance = diag3([imu_accel_var] * 3)
        self.odom_pose_covariance = diag6(
            [odom_xy_var, odom_xy_var, unused, unused, unused, odom_yaw_var])
        self.odom_twist_covariance = diag6(
            [odom_vx_var, unused, unused, unused, unused, odom_vyaw_var])

        self.imu_pub = self.create_publisher(Imu, 'imu_with_covariance', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom_with_covariance', 10)
        self.create_subscription(Imu, 'imu', self.on_imu, 20)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)

        self.get_logger().info(
            f'relaying {self.imu_pub.topic_name} and {self.odom_pub.topic_name} '
            'with covariances filled in')

    @staticmethod
    def _empty(covariance):
        """True if Gazebo left this matrix at all zeros, as it always does."""
        return not any(covariance)

    def on_imu(self, msg):
        if self._empty(msg.orientation_covariance):
            msg.orientation_covariance = self.imu_orientation_covariance
        if self._empty(msg.angular_velocity_covariance):
            msg.angular_velocity_covariance = self.imu_angular_velocity_covariance
        if self._empty(msg.linear_acceleration_covariance):
            msg.linear_acceleration_covariance = self.imu_linear_acceleration_covariance
        self.imu_pub.publish(msg)

    def on_odom(self, msg):
        if self._empty(msg.pose.covariance):
            msg.pose.covariance = self.odom_pose_covariance
        if self._empty(msg.twist.covariance):
            msg.twist.covariance = self.odom_twist_covariance
        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimCovarianceRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
