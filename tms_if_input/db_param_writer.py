#!/usr/bin/env python3
from __future__ import annotations

import json
import os
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from pymongo import MongoClient


DONE_SERVICE_NAME = "/taskset_compiler/done"

DEFAULT_INPUT_DIR = "/tmp"
DEFAULT_PARAMS_FILENAME = "record_params"  # 拡張子なし -> record_params.json

DEFAULT_MONGO_URI = "mongodb://localhost:27017"
DEFAULT_DB_NAME = "rostmsdb"
DEFAULT_PARAM_COLLECTION = "param"


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


def _get_machinery_and_params(entry: Dict[str, Any]) -> Optional[tuple[str, Dict[str, Any]]]:
    """
    entry:
      {"machinery_model":"zx200_1", "parameters":{...}}
    """
    if not isinstance(entry, dict):
        return None
    machinery_model = entry.get("machinery_model")
    params = entry.get("parameters")
    if not isinstance(machinery_model, str) or not isinstance(params, dict):
        return None
    return machinery_model.lower(), params  # ★ model_name は全部小文字運用


def build_param_doc_node_quat(record_name: str, entry: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """
    node + quaternion:
      {
        "model_name": ["ic120_2"],
        "type": "dynamic",
        "x":[...], "y":[...], "z":[...],
        "qx":[...], "qy":[...], "qz":[...], "qw":[...],
        "record_name":"param12"
      }
    """
    got = _get_machinery_and_params(entry)
    if got is None:
        return None
    machinery_model, params = got

    node = params.get("node")
    quat = params.get("quaternion")
    if not isinstance(node, dict) or not isinstance(quat, dict):
        return None

    x, y, z = node.get("x"), node.get("y"), node.get("z")
    qx, qy, qz, qw = quat.get("x"), quat.get("y"), quat.get("z"), quat.get("w")
    nums = [x, y, z, qx, qy, qz, qw]
    if not all(isinstance(v, (int, float)) for v in nums):
        return None

    return {
        "model_name": [machinery_model],
        "type": "dynamic",
        "x": [float(x)],
        "y": [float(y)],
        "z": [float(z)],
        "qx": [float(qx)],
        "qy": [float(qy)],
        "qz": [float(qz)],
        "qw": [float(qw)],
        "record_name": record_name,
    }


def build_param_doc_vessel_angle(record_name: str, entry: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """
    vessel_angle:
      {
        "model_name": "ic120_2",
        "type": "dynamic",
        "description": "",
        "target_angle": -1.0,
        "record_name": "param20"
      }
    """
    got = _get_machinery_and_params(entry)
    if got is None:
        return None
    machinery_model, params = got

    if "vessel_angle" not in params:
        return None

    vessel_angle = params.get("vessel_angle")
    if not isinstance(vessel_angle, (int, float)):
        return None

    return {
        "model_name": machinery_model,
        "type": "dynamic",
        "description": "",
        "target_angle": float(vessel_angle),
        "record_name": record_name,
    }


def _build_point_like_excavate_schema(
    record_name: str,
    machinery_model: str,
    x: float,
    y: float,
    z: float,
) -> Dict[str, Any]:
    """
    excavate_point の形式に揃える共通スキーマ（ダミー固定含む）
    """
    return {
        "model_name": machinery_model,
        "type": "dynamic",
        "description": "",
        "position_with_angle": 1,  # ダミー固定
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "theta_w": 0,              # ダミー固定
        "record_name": record_name,
        "LOCK_FLG": False,         # ダミー固定
        "offset": 1,               # ダミー固定
    }


def build_param_doc_excavate_point(record_name: str, entry: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """
    excavate_point（掘削）
    """
    got = _get_machinery_and_params(entry)
    if got is None:
        return None
    machinery_model, params = got

    ep = params.get("excavate_point")
    if not isinstance(ep, dict):
        return None

    x, y, z = ep.get("x"), ep.get("y"), ep.get("z")
    if not all(isinstance(v, (int, float)) for v in [x, y, z]):
        return None

    return _build_point_like_excavate_schema(record_name, machinery_model, float(x), float(y), float(z))


def build_param_doc_load_point(record_name: str, entry: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """
    load_point（積込）も excavate_point と同じ形式に揃える（ダミー固定OK）
    """
    got = _get_machinery_and_params(entry)
    if got is None:
        return None
    machinery_model, params = got

    lp = params.get("load_point")
    if not isinstance(lp, dict):
        return None

    x, y, z = lp.get("x"), lp.get("y"), lp.get("z")
    if not all(isinstance(v, (int, float)) for v in [x, y, z]):
        return None

    return _build_point_like_excavate_schema(record_name, machinery_model, float(x), float(y), float(z))


class ParamJsonToMongo(Node):
    """
    /taskset_compiler/done が True になったら record_params.json を読み、
    rostmsdb.param に insert する。

    対応:
      - vessel_angle
      - excavate_point（掘削）
      - load_point（積込：掘削と同じ形式に揃える）
      - node+quaternion
    """

    def __init__(self) -> None:
        super().__init__("param_json_to_mongo")

        self.declare_parameter("input_dir", DEFAULT_INPUT_DIR)
        self.declare_parameter("params_filename", DEFAULT_PARAMS_FILENAME)

        self.declare_parameter("mongo_uri", DEFAULT_MONGO_URI)
        self.declare_parameter("mongo_db", DEFAULT_DB_NAME)
        self.declare_parameter("mongo_collection", DEFAULT_PARAM_COLLECTION)

        self.declare_parameter("poll_period_sec", 0.5)

        input_dir = str(self.get_parameter("input_dir").value)
        params_filename = str(self.get_parameter("params_filename").value).strip()
        params_filename = ensure_ext_no_double(params_filename, ".json")
        self._params_path = os.path.join(input_dir, params_filename)

        self._mongo_uri = str(self.get_parameter("mongo_uri").value)
        self._mongo_db = str(self.get_parameter("mongo_db").value)
        self._mongo_collection = str(self.get_parameter("mongo_collection").value)

        self._poll_period = float(self.get_parameter("poll_period_sec").value)

        self._started = False
        self._waiting_call = False

        self._done_cli = self.create_client(Trigger, DONE_SERVICE_NAME)

        self.get_logger().info(f"Waiting done service: {DONE_SERVICE_NAME}")
        self.get_logger().info(f"Params JSON path: {self._params_path}")
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
            data = read_json(self._params_path)
        except Exception as e:
            self.get_logger().error(f"Failed to read params JSON '{self._params_path}': {e}")
            return

        if not isinstance(data, dict):
            self.get_logger().error("params JSON root must be object/dict")
            return

        try:
            client = MongoClient(self._mongo_uri)
            col = client[self._mongo_db][self._mongo_collection]

            docs: List[Dict[str, Any]] = []
            skipped = 0

            for record_name, entry in data.items():
                if not isinstance(record_name, str):
                    skipped += 1
                    continue

                # 優先順（衝突があり得る場合のため）
                doc = (
                    build_param_doc_vessel_angle(record_name, entry)
                    or build_param_doc_excavate_point(record_name, entry)
                    or build_param_doc_load_point(record_name, entry)
                    or build_param_doc_node_quat(record_name, entry)
                )

                if doc is None:
                    skipped += 1
                    continue

                docs.append(doc)

            if not docs:
                self.get_logger().warn(f"No insertable entries found. skipped={skipped}")
                return

            result = col.insert_many(docs)
            self.get_logger().info(f"Inserted into MongoDB: count={len(result.inserted_ids)} skipped={skipped}")

        except Exception as e:
            self.get_logger().error(f"MongoDB insert failed: {e}")
            return

        self.get_logger().info("=== Done (params.json -> MongoDB param) ===")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParamJsonToMongo()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
