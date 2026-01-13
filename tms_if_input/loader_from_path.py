#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class LoaderFromPath(Node):
    def __init__(self) -> None:
        super().__init__("loader_from_path")
        self.declare_parameter("json_path", "")
        self.create_service(Trigger, "/scenario_json", self._handle)

    def _fatal(self, msg: str) -> None:
        self.get_logger().error(msg)
        rclpy.shutdown()

    def _handle(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        json_path = str(self.get_parameter("json_path").value)

        if not json_path:
            response.success = False
            response.message = 'Parameter "json_path" is empty.'
            self._fatal(response.message)
            return response

        p = Path(json_path)
        if not p.exists():
            response.success = False
            response.message = f"JSON file not found: {p}"
            self._fatal(response.message)
            return response

        if not p.is_file():
            response.success = False
            response.message = f"Path exists but is not a file: {p}"
            self._fatal(response.message)
            return response

        try:
            data = json.loads(p.read_text(encoding="utf-8"))
            pretty = json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True)
            # self.get_logger().info(f"Loaded JSON from {p}:\n{pretty}")

            payload = json.dumps(data, ensure_ascii=False)
            response.success = True
            response.message = payload
            self.get_logger().info(f"Served JSON bytes={len(payload)} from {p}")
            return response

        except Exception as e:
            response.success = False
            response.message = f"Failed to read/parse JSON: {e}"
            self._fatal(response.message)
            return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LoaderFromPath()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
