#!/usr/bin/env python3
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from tms_msg_if_input.srv import SegmentedScenario

REQUIRED_KEYS = ["tasksets", "graph", "leveling_area", "operational_area", "machinery"]

SOURCE_SERVICE = "/scenario_json"
SEGMENTS_SERVICE = "/segmented_scenario"


class ScenarioSegmentor(Node):
    def __init__(self) -> None:
        super().__init__("scenario_segmentor")

        self._cb_group = ReentrantCallbackGroup()

        self._cli = self.create_client(Trigger, SOURCE_SERVICE, callback_group=self._cb_group)
        self.create_service(SegmentedScenario, SEGMENTS_SERVICE, self._handle, callback_group=self._cb_group)

        self.get_logger().info(f"scenario_segmentor started. source={SOURCE_SERVICE}, serve={SEGMENTS_SERVICE}")

    def _clear_response(self, response: SegmentedScenario.Response) -> None:
        response.success = False
        response.tasksets = ""
        response.graph = ""
        response.leveling_area = ""
        response.operational_area = ""
        response.machinery = ""

    def _handle(self, request: SegmentedScenario.Request, response: SegmentedScenario.Response):
        self._clear_response(response)

        if not self._cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Source service not available: {SOURCE_SERVICE}")
            return response

        future = self._cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            self.get_logger().error("Timed out waiting for /scenario_json response")
            return response

        try:
            src_res: Trigger.Response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return response

        if not src_res.success:
            self.get_logger().error(f"/scenario_json returned success=false: {src_res.message}")
            return response

        try:
            root = json.loads(src_res.message)
        except Exception as e:
            self.get_logger().error(f"JSON parse failed: {e}")
            return response

        if not isinstance(root, dict):
            self.get_logger().error("JSON root is not an object(dict)")
            return response

        missing = [k for k in REQUIRED_KEYS if k not in root]
        if missing:
            self.get_logger().error(f"Missing required keys: {missing}")
            return response

        response.tasksets = json.dumps(root["tasksets"], ensure_ascii=False)
        response.graph = json.dumps(root["graph"], ensure_ascii=False)
        response.leveling_area = json.dumps(root["leveling_area"], ensure_ascii=False)
        response.operational_area = json.dumps(root["operational_area"], ensure_ascii=False)
        response.machinery = json.dumps(root["machinery"], ensure_ascii=False)

        response.success = True
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioSegmentor()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
