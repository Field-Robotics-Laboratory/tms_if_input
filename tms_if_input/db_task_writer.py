#!/usr/bin/env python3
from __future__ import annotations

import os
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from pymongo import MongoClient


DONE_SERVICE_NAME = "/taskset_compiler/done"

DEFAULT_INPUT_DIR = "/tmp"
DEFAULT_XML_FILENAME = "taskset"  # 拡張子なし -> taskset.xml

DEFAULT_MONGO_URI = "mongodb://localhost:27017"
DEFAULT_DB_NAME = "rostmsdb"
DEFAULT_COLLECTION_NAME = "task"

DEFAULT_DESCRIPTION = "No Commented ..."


def ensure_ext_no_double(name: str, ext: str) -> str:
    if not ext.startswith("."):
        ext = "." + ext
    base, cur = os.path.splitext(name)
    if cur == ext:
        return name
    if base:
        return base + ext
    return name + ext


def read_text(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def smallest_missing_positive_int(sorted_ids: List[int]) -> int:
    expect = 1
    for v in sorted_ids:
        if v < expect:
            continue
        if v == expect:
            expect += 1
        else:
            return expect
    return expect


class TaskXmlToMongo(Node):
    def __init__(self) -> None:
        super().__init__("task_xml_to_mongo")

        # ---- parameters ----
        self.declare_parameter("input_dir", DEFAULT_INPUT_DIR)
        self.declare_parameter("xml_filename", DEFAULT_XML_FILENAME) 

        self.declare_parameter("mongo_uri", DEFAULT_MONGO_URI)
        self.declare_parameter("mongo_db", DEFAULT_DB_NAME)
        self.declare_parameter("mongo_collection", DEFAULT_COLLECTION_NAME)

        self.declare_parameter("poll_period_sec", 0.5)

        input_dir = str(self.get_parameter("input_dir").value)
        xml_filename = str(self.get_parameter("xml_filename").value).strip()
        xml_filename = ensure_ext_no_double(xml_filename, ".xml")
        self._xml_path = os.path.join(input_dir, xml_filename)

        self._mongo_uri = str(self.get_parameter("mongo_uri").value)
        self._mongo_db = str(self.get_parameter("mongo_db").value)
        self._mongo_collection = str(self.get_parameter("mongo_collection").value)
        self._poll_period = float(self.get_parameter("poll_period_sec").value)

        # ---- state ----
        self._started = False
        self._waiting_call = False

        # ---- client ----
        self._done_cli = self.create_client(Trigger, DONE_SERVICE_NAME)

        self.get_logger().info(f"Waiting done service: {DONE_SERVICE_NAME}")
        self.get_logger().info(f"XML path: {self._xml_path}")
        self.get_logger().info(f"Mongo: uri={self._mongo_uri} db={self._mongo_db} col={self._mongo_collection}")

        self._timer = self.create_timer(self._poll_period, self._poll_done_service)

    def _poll_done_service(self) -> None:
        if self._started or self._waiting_call:
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
            xml_text = read_text(self._xml_path)
        except Exception as e:
            self.get_logger().error(f"Failed to read XML '{self._xml_path}': {e}")
            return

        try:
            client = MongoClient(self._mongo_uri)
            col = client[self._mongo_db][self._mongo_collection]

            ids = [doc["task_id"] for doc in col.find({}, {"task_id": 1, "_id": 0}).sort("task_id", 1)]
            new_id = smallest_missing_positive_int(ids)

            doc: Dict[str, Any] = {
                "task_id": int(new_id),
                "description": DEFAULT_DESCRIPTION,
                "task_sequence": str(xml_text),
            }

            result = col.insert_one(doc)
            self.get_logger().info(f"Inserted into MongoDB: task_id={new_id}, _id={result.inserted_id}")

        except Exception as e:
            self.get_logger().error(f"MongoDB insert failed: {e}")
            return

        self.get_logger().info("=== Done (XML -> MongoDB) ===")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskXmlToMongo()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
