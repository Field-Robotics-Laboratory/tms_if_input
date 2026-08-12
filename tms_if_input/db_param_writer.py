#!/usr/bin/env python3
from __future__ import annotations

import json
import os
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from pymongo import MongoClient


DONE_SERVICE_NAME = "/taskset_compiler/done"

DEFAULT_INPUT_DIR = "/tmp"
DEFAULT_PARAMS_FILENAME = "record_params"  # 拡張子なし -> record_params.json

DEFAULT_MONGO_URI = "mongodb://localhost:27017"
DEFAULT_DB_NAME = "rostmsdb"
DEFAULT_PARAM_COLLECTION = "parameter"


def ensure_ext_no_double(name: str, ext: str) -> str:
    if not ext.startswith("."):
        ext = "." + ext
    base, cur = os.path.splitext(name)
    if cur == ext:
        return name
    if base:
        return base + ext
    return name + ext


def read_json(path: str) -> Any:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


class ParamJsonToMongo(Node):
    """
    /taskset_compiler/done が True になったら record_params.json を読み、
    rostmsdb.parameter に insert する。

    record_params.json は taskset_compiler がすでに最終形の
    {record_name: {model_name, type, record_name, ...}} を書き出しているので、
    ここではそのまま各エントリをドキュメントとして挿入するだけでよい。
    """

    def __init__(self) -> None:
        super().__init__("param_json_to_mongo")

        self.declare_parameter("input_dir", DEFAULT_INPUT_DIR)
        self.declare_parameter("params_filename", DEFAULT_PARAMS_FILENAME)

        self.declare_parameter("mongo_uri", DEFAULT_MONGO_URI)
        self.declare_parameter("mongo_db", DEFAULT_DB_NAME)
        self.declare_parameter("mongo_collection", DEFAULT_PARAM_COLLECTION)

        self.declare_parameter("poll_period_sec", 0.5)
        self.declare_parameter("startup_timeout_sec", 15.0)

        input_dir = str(self.get_parameter("input_dir").value)
        params_filename = str(self.get_parameter("params_filename").value).strip()
        params_filename = ensure_ext_no_double(params_filename, ".json")
        self._params_path = os.path.join(input_dir, params_filename)

        self._mongo_uri = str(self.get_parameter("mongo_uri").value)
        self._mongo_db = str(self.get_parameter("mongo_db").value)
        self._mongo_collection = str(self.get_parameter("mongo_collection").value)

        self._poll_period = float(self.get_parameter("poll_period_sec").value)
        self._startup_timeout = float(self.get_parameter("startup_timeout_sec").value)

        self._started = False
        self._waiting_call = False
        self._start_time = self.get_clock().now()

        # rclpy.shutdown()はコールバックの中からではなくmain()の手動spinループから呼ぶ。
        self.exit_requested = False

        self._done_cli = self.create_client(Trigger, DONE_SERVICE_NAME)

        self.get_logger().info(f"Waiting done service: {DONE_SERVICE_NAME}")
        self.get_logger().info(f"Params JSON path: {self._params_path}")
        self.get_logger().info(f"Mongo: uri={self._mongo_uri} db={self._mongo_db} col={self._mongo_collection}")

        self._timer = self.create_timer(self._poll_period, self._poll_done_service)

    def _shutdown(self) -> None:
        self._timer.cancel()
        self.exit_requested = True

    def _poll_done_service(self) -> None:
        if self._started or self._waiting_call:
            return

        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed > self._startup_timeout:
            self.get_logger().error(
                f"Gave up waiting for {DONE_SERVICE_NAME} after {self._startup_timeout}s. Shutting down."
            )
            self._started = True
            self._shutdown()
            return

        if not self._done_cli.service_is_ready():
            if not self._done_cli.wait_for_service(timeout_sec=0.0):
                return

        self._waiting_call = True
        future = self._done_cli.call_async(Trigger.Request())
        future.add_done_callback(self._on_done_response)

    def _on_done_response(self, future) -> None:
        self._waiting_call = False

        try:
            res: Trigger.Response = future.result()
        except Exception as e:
            self.get_logger().warn(f"Done service call failed: {e}")
            return

        if not res.success:
            self.get_logger().debug(f"Not ready yet: {res.message}")
            return

        if self._started:
            return

        self._started = True
        self._timer.cancel()

        self.get_logger().info(f"Done received: {res.message}")
        self._process_once()

    def _process_once(self) -> None:
        try:
            data = read_json(self._params_path)
        except Exception as e:
            self.get_logger().error(f"Failed to read params JSON '{self._params_path}': {e}")
            self._shutdown()
            return

        if not isinstance(data, dict):
            self.get_logger().error("params JSON root must be object/dict")
            self._shutdown()
            return

        docs: List[Dict[str, Any]] = [entry for entry in data.values() if isinstance(entry, dict)]
        skipped = len(data) - len(docs)

        if not docs:
            self.get_logger().warn(f"No insertable entries found. skipped={skipped}")
            self._shutdown()
            return

        try:
            client = MongoClient(self._mongo_uri)
            col = client[self._mongo_db][self._mongo_collection]

            deleted = col.delete_many({}).deleted_count
            self.get_logger().info(f"Cleared existing documents in {self._mongo_collection}: count={deleted}")

            result = col.insert_many(docs)
            self.get_logger().info(f"Inserted into MongoDB: count={len(result.inserted_ids)} skipped={skipped}")

        except Exception as e:
            self.get_logger().error(f"MongoDB insert failed: {e}")
            self._shutdown()
            return

        self.get_logger().info("=== Done (params.json -> MongoDB parameter) ===")
        self._shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParamJsonToMongo()

    # rclpy.shutdown()をコールバックの中から呼ぶとspin()が正しく戻らないため、
    # メインスレッドでフラグを見ながら手動でspin_once()する。
    while rclpy.ok() and not node.exit_requested:
        rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
