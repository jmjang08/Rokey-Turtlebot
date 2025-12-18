# my_nav_utils/initial_pose_publisher.py

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # 파라미터 선언
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', 'initialpose')
        self.declare_parameter('publish_delay', 3.0)

        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.topic = self.get_parameter('topic').value
        self.publish_delay = float(self.get_parameter('publish_delay').value)

        # initialpose 퍼블리셔
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.topic,
            10
        )

        # 지정한 시간 뒤에 한 번만 퍼블리시
        self.timer = self.create_timer(self.publish_delay, self._publish_once)

        self.get_logger().info(
            f'InitialPosePublisher 준비: topic={self.topic}, '
            f'frame_id={self.frame_id}, x={self.x}, y={self.y}, yaw={self.yaw}'
        )

    def _publish_once(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # 위치
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        # yaw → quaternion
        half_yaw = self.yaw * 0.5
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        # 대충 AMCL 기본 수준의 covariance (x, y, yaw)
        cov = [0.0] * 36
        cov[0] = 0.25   # x
        cov[7] = 0.25   # y
        cov[35] = 0.0685  # yaw
        msg.pose.covariance = cov

        self.pub.publish(msg)
        self.get_logger().info(
            f'초기 포즈 publish 완료: ({self.x:.3f}, {self.y:.3f}, yaw={self.yaw:.3f} rad)'
        )

        # 타이머 중지 (한 번만 쏘고 끝)
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
