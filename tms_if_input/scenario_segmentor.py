#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
import xml.etree.ElementTree as ET

REQUIRED_CHILDREN = ["machines", "graphml", "scxml"]

SOURCE_SERVICE = "/scenario_source"
SEGMENTS_SERVICE = "/segmented_scenario"


def strip_namespaces(elem: ET.Element) -> ET.Element:
    for el in elem.iter():
        if "}" in el.tag:
            el.tag = el.tag.split("}", 1)[1]
    return elem


class ScenarioSegmentor(Node):
    def __init__(self) -> None:
        super().__init__("scenario_segmentor")

        self._cb_group = ReentrantCallbackGroup()

        self._cli = self.create_client(Trigger, SOURCE_SERVICE, callback_group=self._cb_group)
        self.create_service(Trigger, SEGMENTS_SERVICE, self._handle, callback_group=self._cb_group)

        # サービス呼び出しに一度応答したらこのノードの役目は終わりなので、
        # main()の手動spinループがこのフラグを見て終了する。
        self.exit_requested = False

        self.get_logger().info(f"scenario_segmentor started. source={SOURCE_SERVICE}, serve={SEGMENTS_SERVICE}")

    def _handle(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success = False
        response.message = ""

        try:
            if not self._cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(f"Source service not available: {SOURCE_SERVICE}")
                return response

            future = self._cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if not future.done():
                self.get_logger().error(f"Timed out waiting for {SOURCE_SERVICE} response")
                return response

            try:
                src_res: Trigger.Response = future.result()
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
                return response

            if not src_res.success:
                self.get_logger().error(f"{SOURCE_SERVICE} returned success=false: {src_res.message}")
                return response

            xml_text = src_res.message

            try:
                root = ET.fromstring(xml_text)
            except Exception as e:
                self.get_logger().error(f"XML parse failed: {e}")
                return response

            strip_namespaces(root)

            if root.tag != "ConstructionPlan":
                self.get_logger().error(f"Unexpected root tag: {root.tag} (expected 'ConstructionPlan')")
                return response

            missing = [tag for tag in REQUIRED_CHILDREN if root.find(tag) is None]
            if missing:
                self.get_logger().error(f"Missing required elements under ConstructionPlan: {missing}")
                return response

            response.message = xml_text
            response.success = True
            return response

        finally:
            self.exit_requested = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioSegmentor()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        # rclpy.shutdown()をコールバックの中から呼ぶとspin()が正しく戻らないため、
        # メインスレッドでフラグを見ながら手動でspin_once()する。
        while rclpy.ok() and not node.exit_requested:
            executor.spin_once(timeout_sec=0.5)
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
