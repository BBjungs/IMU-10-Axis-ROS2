import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


GRAVITY = 9.80665


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def quaternion_conjugate(q):
    return [-q[0], -q[1], -q[2], q[3]]


def normalize_quaternion(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm == 0.0:
        return [0.0, 0.0, 0.0, 1.0]
    return [value / norm for value in q]


def quaternion_from_yaw(yaw):
    half_yaw = yaw * 0.5
    return [0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)]


def rotate_vector(q, vector):
    q = normalize_quaternion(q)
    rotated = quaternion_multiply(
        quaternion_multiply(q, [vector[0], vector[1], vector[2], 0.0]),
        quaternion_conjugate(q),
    )
    return rotated[0], rotated[1], rotated[2]


class ImuOdomNode(Node):
    def __init__(self):
        super().__init__('imu_odom_node')

        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('integrate_position', True)
        self.declare_parameter('axis_correction_yaw', 0.0)
        self.declare_parameter('acceleration_deadband', 0.08)
        self.declare_parameter('max_dt', 0.1)

        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.integrate_position = self.get_parameter('integrate_position').value
        self.axis_correction_yaw = self.get_parameter('axis_correction_yaw').value
        self.acceleration_deadband = self.get_parameter('acceleration_deadband').value
        self.max_dt = self.get_parameter('max_dt').value
        self.axis_correction = quaternion_from_yaw(self.axis_correction_yaw)

        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.last_time = None

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 20)

        if self.integrate_position:
            mode = 'integrating acceleration'
        else:
            mode = 'orientation only'
        self.get_logger().info(
            f'Publishing IMU odometry from {self.imu_topic} to {self.odom_topic} '
            f'({mode}, yaw correction {math.degrees(self.axis_correction_yaw):.1f} deg)'
        )

    def imu_callback(self, msg):
        stamp = msg.header.stamp
        current_time = stamp.sec + stamp.nanosec * 1e-9
        if current_time == 0.0:
            current_time = self.get_clock().now().nanoseconds * 1e-9
            stamp = self.get_clock().now().to_msg()

        if not self.integrate_position:
            self.position = [0.0, 0.0, 0.0]
            self.velocity = [0.0, 0.0, 0.0]
            self.last_time = current_time
            self.publish_odom(msg, stamp)
            return

        if self.last_time is None:
            self.last_time = current_time
            self.publish_odom(msg, stamp)
            return

        dt = current_time - self.last_time
        self.last_time = current_time
        if dt <= 0.0 or dt > self.max_dt:
            self.publish_odom(msg, stamp)
            return

        orientation = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]
        orientation = self.correct_orientation(orientation)
        acceleration = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ]
        acceleration = self.correct_vector(acceleration)

        ax, ay, az = rotate_vector(orientation, acceleration)
        az -= GRAVITY

        world_acceleration = [self.apply_deadband(ax), self.apply_deadband(ay), self.apply_deadband(az)]
        for index in range(3):
            self.velocity[index] += world_acceleration[index] * dt
            self.position[index] += self.velocity[index] * dt

        self.publish_odom(msg, stamp)

    def apply_deadband(self, value):
        if abs(value) < self.acceleration_deadband:
            return 0.0
        return value

    def correct_orientation(self, orientation):
        return normalize_quaternion(quaternion_multiply(self.axis_correction, orientation))

    def correct_vector(self, vector):
        return rotate_vector(self.axis_correction, vector)

    def publish_odom(self, imu_msg, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        orientation = self.correct_orientation([
            imu_msg.orientation.x,
            imu_msg.orientation.y,
            imu_msg.orientation.z,
            imu_msg.orientation.w,
        ])
        angular_velocity = self.correct_vector([
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z,
        ])

        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = self.position[2]
        odom.pose.pose.orientation.x = orientation[0]
        odom.pose.pose.orientation.y = orientation[1]
        odom.pose.pose.orientation.z = orientation[2]
        odom.pose.pose.orientation.w = orientation[3]

        odom.twist.twist.linear.x = self.velocity[0]
        odom.twist.twist.linear.y = self.velocity[1]
        odom.twist.twist.linear.z = self.velocity[2]
        odom.twist.twist.angular.x = angular_velocity[0]
        odom.twist.twist.angular.y = angular_velocity[1]
        odom.twist.twist.angular.z = angular_velocity[2]

        self.odom_pub.publish(odom)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.position[0]
            transform.transform.translation.y = self.position[1]
            transform.transform.translation.z = self.position[2]
            transform.transform.rotation.x = orientation[0]
            transform.transform.rotation.y = orientation[1]
            transform.transform.rotation.z = orientation[2]
            transform.transform.rotation.w = orientation[3]
            self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = ImuOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
