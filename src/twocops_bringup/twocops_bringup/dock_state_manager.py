#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from irobot_create_msgs.msg import DockStatus
from irobot_create_msgs.action import Undock


class DockStateManager(Node):
    """
    - 첫 dock_status를 기준으로 초기 상태 판단:
      - 처음 is_docked == True  → UNDOCK 실행
      - 처음 is_docked == False → 경고 로그 후 종료
    """

    def __init__(self):
        super().__init__('dock_state_manager')

        # 파라미터: robot_id (1 → /robot1)
        self.declare_parameter('robot_id', 1)
        robot_id = self.get_parameter('robot_id').value

        if isinstance(robot_id, int):
            ns = f'/robot{robot_id}'
        else:
            ns = f'/{robot_id.lstrip("/")}'  # "robot1" 문자열도 허용

        self.ns = ns

        # 상태 변수
        self.is_docked = None
        self.initial_state_checked = False  # ✅ 초기 상태 판단이 끝났는지
        self._undock_sent = False
        self.done = False

        # 액션 클라이언트 (/robotN/undock)
        self.undock_client = ActionClient(self, Undock, f'{self.ns}/undock')

        # 도크 상태 구독 (/robotN/dock_status)
        self.create_subscription(
            DockStatus,
            f'{self.ns}/dock_status',
            self.dock_status_callback,
            10,
        )

        self.get_logger().info(
            f"✅ DockStateManager initialized (ns={self.ns}) - waiting for dock_status..."
        )

    # -----------------------------
    # DockStatus 콜백
    # -----------------------------
    def dock_status_callback(self, msg: DockStatus):
        if self.done:
            return

        self.is_docked = msg.is_docked
        self.get_logger().info(f"📡 Dock status received: is_docked={self.is_docked}")

        # ✅ 첫 메시지에서만 초기 상태 판단
        if not self.initial_state_checked:
            self.initial_state_checked = True

            if self.is_docked:
                # 처음 상태가 dock → undock 진행
                self.get_logger().info("🚀 Robot is docked at startup → sending UNDOCK goal")
                self.send_undock_goal()
            else:
                # 처음부터 undocked → 경고만 출력하고 종료
                self.get_logger().warn(
                    "⚠️ Robot is already undocked at startup. "
                    "Please start the scenario from DOCK state."
                )
                self.done = True

        else:
            # 이후 들어오는 dock_status는 참고용 로그 정도로만 사용
            self.get_logger().debug(
                f"[DEBUG] Subsequent dock_status: is_docked={self.is_docked}"
            )

    # -----------------------------
    # Undock 액션 관련
    # -----------------------------
    def send_undock_goal(self):
        self._undock_sent = True

        if not self.undock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "❌ Undock action server not available after 10 seconds."
            )
            self.done = True
            return

        goal_msg = Undock.Goal()
        send_future = self.undock_client.send_goal_async(
            goal_msg,
            feedback_callback=self.undock_feedback_callback,
        )
        send_future.add_done_callback(self.undock_goal_response_callback)

    def undock_feedback_callback(self, feedback_msg):
        # 필요하면 피드백 로그 찍기
        pass

    def undock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Undock goal rejected")
            self.done = True
            return

        self.get_logger().info("✅ Undock goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.undock_result_callback)

    def undock_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🎉 Undock succeeded.")
        else:
            self.get_logger().warn(f"⚠️ Undock finished with status: {status}")

        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = DockStateManager()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    node.get_logger().info("🔚 DockStateManager finished. Shutting down...")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
