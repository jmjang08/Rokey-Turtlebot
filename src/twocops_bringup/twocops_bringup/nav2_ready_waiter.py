#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose


class Nav2ReadyWaiter(Node):
    """
    /robotN/navigate_to_pose 액션 서버(Nav2)가 켜질 때까지 기다렸다가
    켜지면 바로 종료하는 helper 노드.
    """

    def __init__(self):
        super().__init__('nav2_ready_waiter')

        # 파라미터: robot_id (1 또는 3), timeout(초)
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('timeout', 300.0)  # 최대 5분까지 대기

        robot_id_param = self.get_parameter('robot_id').value
        timeout_param = self.get_parameter('timeout').value

        # Launch에서 문자열로 넘어와도 int(...) 가능하게 처리
        try:
            self.robot_id = int(robot_id_param)
        except Exception:
            self.robot_id = 1

        try:
            self.timeout = float(timeout_param)
        except Exception:
            self.timeout = 300.0

        self.action_name = f'/robot{self.robot_id}/navigate_to_pose'
        self._client = ActionClient(self, NavigateToPose, self.action_name)

        self.get_logger().info(
            f'🛰 Nav2ReadyWaiter 시작: robot_id={self.robot_id}, '
            f'waiting for action server [{self.action_name}], '
            f'timeout={self.timeout}s'
        )

    def wait_for_nav2(self) -> bool:
        start = time.time()
        # 1초 간격으로 액션 서버 존재 여부 체크
        while rclpy.ok():
            if self._client.wait_for_server(timeout_sec=5.0):
                self.get_logger().info(
                    f'✅ Nav2 NavigateToPose action server is available at [{self.action_name}]'
                )
                return True

            elapsed = time.time() - start
            self.get_logger().info(
                f'⏳ Waiting for Nav2 action server [{self.action_name}]... '
                f'elapsed={elapsed:.1f}s'
            )
            if elapsed > self.timeout:
                self.get_logger().error(
                    f'⛔ Timeout({self.timeout}s) waiting for Nav2 action server [{self.action_name}]'
                )
                return False
        return False


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ReadyWaiter()
    try:
        ok = node.wait_for_nav2()
    except KeyboardInterrupt:
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # nav2_ready_waiter가 종료되면 exit code 0/1로 상태를 알려줌
    if ok:
        raise SystemExit(0)
    else:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
