#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import xml.etree.ElementTree as ET

SOURCE_SERVICE = "/scenario_source"


class LoaderFromPath(Node):
    def __init__(self) -> None:
        super().__init__("loader_from_path")
        self.declare_parameter("xml_path", "")
        self.create_service(Trigger, SOURCE_SERVICE, self._handle)

        # サービス呼び出しに一度応答したらこのノードの役目は終わりなので、
        # main()の手動spinループがこのフラグを見て終了する。
        self.exit_requested = False

    def _handle(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        xml_path = str(self.get_parameter("xml_path").value)

        if not xml_path:
            response.success = False
            response.message = 'Parameter "xml_path" is empty.'
            self.get_logger().error(response.message)
            self.exit_requested = True
            return response

        p = Path(xml_path)
        if not p.exists():
            response.success = False
            response.message = f"XML file not found: {p}"
            self.get_logger().error(response.message)
            self.exit_requested = True
            return response

        if not p.is_file():
            response.success = False
            response.message = f"Path exists but is not a file: {p}"
            self.get_logger().error(response.message)
            self.exit_requested = True
            return response

        try:
            xml_text = p.read_text(encoding="utf-8")
            ET.fromstring(xml_text)  # 妥当なXMLかどうかだけ検証する

            response.success = True
            response.message = xml_text
            self.get_logger().info(f"Served XML bytes={len(xml_text)} from {p}")

        except Exception as e:
            response.success = False
            response.message = f"Failed to read/parse XML: {e}"
            self.get_logger().error(response.message)

        self.exit_requested = True
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LoaderFromPath()

    # rclpy.shutdown()をコールバックの中から呼ぶとspin()が正しく戻らないため、
    # メインスレッドでフラグを見ながら手動でspin_once()する。
    while rclpy.ok() and not node.exit_requested:
        rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
