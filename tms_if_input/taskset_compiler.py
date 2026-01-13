#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from tms_msg_if_input.srv import SegmentedScenario


SEGMENTS_SERVICE = "/segmented_scenario"


class TasksetCompiler(Node):
    def __init__(self) -> None:
        super().__init__("taskset_compiler")

        self._cli = self.create_client(SegmentedScenario, SEGMENTS_SERVICE)

        self.get_logger().info(f"taskset_compiler started. calling {SEGMENTS_SERVICE} ...")

        # 起動直後に1回だけ呼ぶ（結果ログ出して終了）
        self._timer = self.create_timer(0.1, self._kick_once)
        self._kicked = False

    def _kick_once(self) -> None:
        if self._kicked:
            return
        self._kicked = True
        self._timer.cancel()

        if not self._cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service not available: {SEGMENTS_SERVICE}")
            rclpy.shutdown()
            return

        future = self._cli.call_async(SegmentedScenario.Request())
        future.add_done_callback(self._on_response)

    def _on_response(self, future) -> None:
        try:
            res: SegmentedScenario.Response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            rclpy.shutdown()
            return

        if not res.success:
            self.get_logger().error("segmented_scenario returned success=false")
            rclpy.shutdown()
            return

        # ログ出力（長いのでそのままだと膨大。必要ならtruncateする）
        self.get_logger().info("=== Received SegmentedScenario ===")
        self.get_logger().info(f"[tasksets]\n{res.tasksets}")
        self.get_logger().info(f"[graph]\n{res.graph}")
        self.get_logger().info(f"[leveling_area]\n{res.leveling_area}")
        self.get_logger().info(f"[operational_area]\n{res.operational_area}")
        self.get_logger().info(f"[machinery]\n{res.machinery}")
        self.get_logger().info("=== Done ===")

        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TasksetCompiler()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
