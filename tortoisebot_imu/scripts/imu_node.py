#!/usr/bin/env python3
"""BNO055 IMU publisher for the TortoiseBot.

Publishes sensor_msgs/Imu on /imu in the base_link frame with covariances
filled in. Consumers are cartographer_node (which remaps imu:=/imu) and, when
enabled, robot_localization's ekf_filter_node.
"""

import math

import board
import adafruit_bno055
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')

        # frame_id must name a link that actually exists in TF -- there is no
        # frame called 'imu', and both cartographer and the EKF drop every
        # message whose frame they cannot look up.
        #
        # base_link rather than imu_link, even though the chip is physically at
        # imu_link (0, 0, 0.11) in tortoisebotreal.xacro. imu_joint is a pure z
        # translation with no rotation and the robot rotates about z, so
        # orientation and angular velocity are identical in the two frames and
        # the lever-arm acceleration terms w x (w x r) and a x r are both exactly
        # zero: for planar motion the readings are the same either way. Labelling
        # them base_link lets Cartographer keep tracking_frame = base_link, which
        # avoids dragging the 0.11 m offset into map->odom as z = -0.110 -- the
        # colocation assert would otherwise force tracking_frame = imu_link.
        # imu_link stays in the URDF as the record of where the chip really is.
        # Revisit if linear acceleration is ever fused or the robot goes 3D.
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value

        # 50 Hz to match the Ignition imu_sensor in sim. The BNO055 fusion
        # output tops out near 100 Hz and each cycle is three short I2C reads,
        # so this is well inside the sensor's and the bus's budget. Drop it if
        # the Pi's I2C bus turns out to be the bottleneck.
        rate_hz = float(self.declare_parameter('rate_hz', 50.0).value)

        # sensor_msgs/Imu covariances are row-major 3x3. Publishing [-1.0] * 9
        # is the ROS convention for "this quantity is not measured", which makes
        # robot_localization refuse the field outright -- so the old node's
        # orientation was unusable no matter what else was fixed.
        # These are 1-sigma values for a calibrated BNO055 in NDOF mode. Yaw is
        # the loosest because indoor magnetic disturbance, not sensor noise,
        # dominates heading error near the motors.
        orient_sigma = self.declare_parameter(
            'orientation_stddev_rpy', [0.05, 0.05, 0.10]).value      # rad
        gyro_sigma = float(self.declare_parameter(
            'angular_velocity_stddev', 0.01).value)                  # rad/s
        accel_sigma = float(self.declare_parameter(
            'linear_acceleration_stddev', 0.20).value)               # m/s^2

        # Raw BNO055 calibration offset registers, captured from this unit.
        # Units are register LSBs -- accel 1/100 m/s^2, gyro 1/16 deg/s,
        # mag 1/16 uT -- which is what the offset registers expect. They are
        # written to the sensor at startup, not subtracted in software; see
        # _apply_stored_offsets.
        offsets_accel = self.declare_parameter(
            'offsets_accelerometer', [-59, -2, -22]).value
        offsets_gyro = self.declare_parameter(
            'offsets_gyroscope', [-2, -2, 1]).value
        offsets_mag = self.declare_parameter(
            'offsets_magnetometer', [53, -130, -477]).value

        self.orientation_covariance = self._diag(
            [float(s) ** 2 for s in orient_sigma])
        self.angular_velocity_covariance = self._diag([gyro_sigma ** 2] * 3)
        self.linear_acceleration_covariance = self._diag([accel_sigma ** 2] * 3)

        self.publisher_ = self.create_publisher(Imu, 'imu', 10)

        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self.get_logger().info('BNO055 IMU initialized')
        self._apply_stored_offsets(offsets_accel, offsets_gyro, offsets_mag)

        self.timer = self.create_timer(1.0 / rate_hz, self.publish_imu_data)
        self.get_logger().info(
            f'publishing sensor_msgs/Imu on {self.publisher_.topic_name} '
            f'in frame {self.frame_id} at {rate_hz:.1f} Hz')

    @staticmethod
    def _diag(variances):
        """Row-major 3x3 covariance with the given diagonal."""
        cov = [0.0] * 9
        for i, var in enumerate(variances):
            cov[i * 3 + i] = var
        return cov

    def _apply_stored_offsets(self, accel, gyro, mag):
        """Write the stored calibration offsets into the sensor's registers.

        The previous node subtracted these triples from the *scaled* readings
        instead. Because they are register LSBs, the accelerometer triple alone
        injected 59 m/s^2 -- six g -- of phantom acceleration on X, enough to
        make any gravity-based attitude estimate meaningless, and the
        magnetometer triple was never used at all. Writing them to the offset
        registers is what they are for; adafruit_bno055 handles the required
        CONFIG_MODE round-trip.
        """
        try:
            self.sensor.offsets_accelerometer = tuple(int(v) for v in accel)
            self.sensor.offsets_gyroscope = tuple(int(v) for v in gyro)
            self.sensor.offsets_magnetometer = tuple(int(v) for v in mag)
        except Exception as exc:                     # noqa: BLE001 - I2C can fail any number of ways
            self.get_logger().warn(f'could not write calibration offsets: {exc}')
            return
        self.get_logger().info(
            f'wrote calibration offsets accel={tuple(accel)} '
            f'gyro={tuple(gyro)} mag={tuple(mag)}')

    @staticmethod
    def _complete(reading, length):
        """True if the sensor returned a full tuple with no None entries."""
        return (reading is not None
                and len(reading) == length
                and all(v is not None for v in reading))

    def publish_imu_data(self):
        quat = self.sensor.quaternion
        gyro = self.sensor.gyro
        # sensor.acceleration, not sensor.linear_acceleration: sensor_msgs/Imu
        # expects gravity to be *included* -- that is why robot_localization
        # offers imu0_remove_gravitational_acceleration, and why Ignition
        # reports gravity in sim. BNO055's linear_acceleration is the
        # gravity-compensated vector, which left cartographer with no gravity
        # direction to align its 2D frame to.
        accel = self.sensor.acceleration

        if not (self._complete(quat, 4) and self._complete(gyro, 3)
                and self._complete(accel, 3)):
            self.get_logger().warn('incomplete BNO055 read, skipping sample',
                                   throttle_duration_sec=5.0)
            return

        # The BNO055 returns an all-zero quaternion while the fusion algorithm
        # is still converging. Publishing that would hand every consumer an
        # invalid rotation, so skip the sample instead.
        norm = math.sqrt(sum(float(c) ** 2 for c in quat))
        if abs(norm - 1.0) > 0.1:
            self.get_logger().warn(
                f'BNO055 returned a non-unit quaternion (norm {norm:.3f}), '
                'skipping sample', throttle_duration_sec=5.0)
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        w, x, y, z = (float(c) / norm for c in quat)
        msg.orientation.w = w
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation_covariance = self.orientation_covariance

        # adafruit_bno055 scales the gyro registers by 0.001090830782496456,
        # i.e. it already returns rad/s. The old math.radians() call divided
        # every rate by a further 57.3.
        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])
        msg.angular_velocity_covariance = self.angular_velocity_covariance

        msg.linear_acceleration.x = float(accel[0])
        msg.linear_acceleration.y = float(accel[1])
        msg.linear_acceleration.z = float(accel[2])
        msg.linear_acceleration_covariance = self.linear_acceleration_covariance

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('IMU publisher stopped cleanly')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
