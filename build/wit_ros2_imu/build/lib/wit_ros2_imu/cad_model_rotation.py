import math

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CadModelRotationNode(Node):
    def __init__(self):
        super().__init__('cad_model_rotation')

        self.declare_parameter('spin', True)
        self.declare_parameter('yaw_rate_deg_s', 15.0)
        self.declare_parameter('initial_yaw_deg', 0.0)

        self.spin = self.get_parameter('spin').value
        self.yaw_rate_deg_s = self.get_parameter('yaw_rate_deg_s').value
        self.yaw = math.radians(self.get_parameter('initial_yaw_deg').value)
        self.last_time = self.get_clock().now()

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.create_timer(0.05, self.publish_joint_state)

        self.get_logger().info(
            'CAD model rotation is '
            f"{'enabled' if self.spin else 'disabled'} at {self.yaw_rate_deg_s:.1f} deg/s"
        )

    def parameter_callback(self, parameters):
        for parameter in parameters:
            if parameter.name == 'spin' and parameter.type_ != parameter.Type.BOOL:
                return SetParametersResult(successful=False, reason='spin must be a boolean')
            if (
                parameter.name == 'yaw_rate_deg_s'
                and parameter.type_ != parameter.Type.DOUBLE
            ):
                return SetParametersResult(
                    successful=False,
                    reason='yaw_rate_deg_s must be a floating-point value',
                )
            if (
                parameter.name == 'initial_yaw_deg'
                and parameter.type_ != parameter.Type.DOUBLE
            ):
                return SetParametersResult(
                    successful=False,
                    reason='initial_yaw_deg must be a floating-point value',
                )

        for parameter in parameters:
            if parameter.name == 'spin':
                self.spin = parameter.value
            elif parameter.name == 'yaw_rate_deg_s':
                self.yaw_rate_deg_s = parameter.value
            elif parameter.name == 'initial_yaw_deg':
                self.yaw = math.radians(parameter.value)

        return SetParametersResult(successful=True)

    def publish_joint_state(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if self.spin:
            self.yaw = math.fmod(
                self.yaw + math.radians(self.yaw_rate_deg_s) * dt,
                math.tau,
            )

        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['cad_model_yaw_joint']
        joint_state.position = [self.yaw]
        self.joint_state_pub.publish(joint_state)


def main():
    rclpy.init()
    node = CadModelRotationNode()
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
