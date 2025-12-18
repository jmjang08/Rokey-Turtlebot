#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from irobot_create_msgs.action import Dock, Undock


class DockingNode(Node):
    """
    /robotN/dock, /robotN/undock 액션을 한 번 호출하고
    결과를 기다렸다가 종료하는 노드.
    - 파라미터:
      - robot_id (int or str, 예: 1 → /robot1)
      - command: 'dock' 또는 'undock'
    """

    def __init__(self):
        super().__init__('docking_node')

        # 파라미터 선언
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('command', 'undock')

        robot_id = self.get_parameter('robot_id').value
        command = self.get_parameter('command').value

        # /robotX 네임스페이스 생성
        if isinstance(robot_id, int):
            ns = f'/robot{robot_id}'
        else:
            ns = f'/{robot_id.lstrip("/")}'  # "robot1" 같은 문자열도 허용

        self.ns = ns
        self.command = command

        # 액션 클라이언트 생성
        self.undock_client = ActionClient(self, Undock, f'{self.ns}/undock')
        self.dock_client = ActionClient(self, Dock, f'{self.ns}/dock')

        self._active_goal_handle = None

        self.get_logger().info(
            f"✅ DockingNode initialized (ns={self.ns}, command={self.command})"
        )

    # ------------ 외부에서 한 번만 호출 ------------
    def start(self):
        if self.command == 'undock':
            self.send_undock_goal()
        elif self.command == 'dock':
            self.send_dock_goal()
        else:
            self.get_logger().error(
                f"❌ Unknown command: {self.command} (use 'dock' or 'undock')"
            )
            self._shutdown()

    # --------------------
    # Undock
    # --------------------
    def send_undock_goal(self):
        self.get_logger().info(f"📤 Sending UNDOCK goal to {self.ns}/undock")

        # 서버 기다리기 (최대 10초)
        if not self.undock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ Undock action server not available after 10 seconds.")
            self._shutdown()
            return

        goal_msg = Undock.Goal()  # 빈 goal
        send_future = self.undock_client.send_goal_async(
            goal_msg,
            feedback_callback=self.undock_feedback_callback
        )
        send_future.add_done_callback(self.undock_goal_response_callback)

    def undock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Undock goal rejected")
            self._shutdown()
            return

        self.get_logger().info("✅ Undock goal accepted")
        self._active_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.undock_result_callback)

    def undock_feedback_callback(self, feedback_msg):
        # 필요하면 피드백 출력
        # feedback = feedback_msg.feedback
        # self.get_logger().info("Undock feedback ...")
        pass

    def undock_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🎉 Undock succeeded.")
        else:
            self.get_logger().warn(f"⚠️ Undock finished with status: {status}")

        self._active_goal_handle = None
        self._shutdown()

    # --------------------
    # Dock
    # --------------------
    def send_dock_goal(self):
        self.get_logger().info(f"📤 Sending DOCK goal to {self.ns}/dock")

        if not self.dock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ Dock action server not available after 10 seconds.")
            self._shutdown()
            return

        goal_msg = Dock.Goal()
        send_future = self.dock_client.send_goal_async(
            goal_msg,
            feedback_callback=self.dock_feedback_callback
        )
        send_future.add_done_callback(self.dock_goal_response_callback)

    def dock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Dock goal rejected")
            self._shutdown()
            return

        self.get_logger().info("✅ Dock goal accepted")
        self._active_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.dock_result_callback)

    def dock_feedback_callback(self, feedback_msg):
        pass

    def dock_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🎉 Dock succeeded.")
        else:
            self.get_logger().warn(f"⚠️ Dock finished with status: {status}")

        self._active_goal_handle = None
        self._shutdown()

    # --------------------
    # 공통 종료 처리
    # --------------------
    def _shutdown(self):
        self.get_logger().info("🔚 DockingNode finished. Shutting down...")
        # 여기서 직접 rclpy.shutdown()까지 해준다.
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DockingNode()

    # ⬇️ 노드 생성 직후 바로 start() 한 번 호출
    node.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # _shutdown()에서 rclpy.shutdown() 호출하므로 여기선 별도 처리 필요 X


if __name__ == "__main__":
    main()
