#!/usr/bin/env python3
from __future__ import annotations

import copy
import json
import os
from typing import Any, Dict, List, Tuple, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_srvs.srv import Trigger
from tms_msg_if_input.srv import SegmentedScenario
import yaml
import xml.etree.ElementTree as ET


SEGMENTS_SERVICE = "/segmented_scenario"
DONE_SERVICE = "/taskset_compiler/done"

CONFIG_PKG_NAME = "tms_if_input"

DEFAULT_OUTPUT_DIR = "/tmp"
DEFAULT_XML_FILENAME = "taskset"            # -> taskset.xml
DEFAULT_PARAMS_FILENAME = "record_params"   # -> record_params.json


def ensure_ext_no_double(name: str, ext: str) -> str:
    if not ext.startswith("."):
        ext = "." + ext
    base, cur = os.path.splitext(name)
    if cur == ext:
        return name
    if base:
        return base + ext
    return name + ext


def ensure_parent_dir(path: str) -> None:
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)


def write_text_file(path: str, content: str) -> None:
    ensure_parent_dir(path)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)


def write_json_file(path: str, obj: Any) -> None:
    ensure_parent_dir(path)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)


def machinery_json_to_unique_id_name(machinery_json: str) -> Dict[int, str]:
    obj = json.loads(machinery_json)
    if not isinstance(obj, list):
        raise TypeError(f"machinery must be a JSON list, got {type(obj).__name__}")

    out: Dict[int, str] = {}
    per_model_count: Dict[str, int] = {}

    for i, item in enumerate(obj):
        if not isinstance(item, dict):
            raise TypeError(f"machinery[{i}] must be an object, got {type(item).__name__}")

        name = item.get("name")
        mid = item.get("id")

        if not isinstance(name, str):
            raise TypeError(f"machinery[{i}].name must be str, got {type(name).__name__}")
        if not isinstance(mid, int):
            raise TypeError(f"machinery[{i}].id must be int, got {type(mid).__name__}")

        per_model_count[name] = per_model_count.get(name, 0) + 1
        out[mid] = f"{name}_{per_model_count[name]}"

    return out


def graph_json_to_id_point(graph_json: str) -> Dict[int, Dict[str, float]]:
    obj = json.loads(graph_json)
    if not isinstance(obj, list):
        raise TypeError(f"graph must be a JSON list, got {type(obj).__name__}")

    out: Dict[int, Dict[str, float]] = {}
    for i, item in enumerate(obj):
        if not isinstance(item, dict):
            raise TypeError(f"graph[{i}] must be an object, got {type(obj).__name__}")

        nid = item.get("id")
        pt = item.get("point")

        if not isinstance(nid, int):
            raise TypeError(f"graph[{i}].id must be int, got {type(nid).__name__}")
        if not isinstance(pt, dict):
            raise TypeError(f"graph[{i}].point must be object, got {type(pt).__name__}")

        x, y, z = pt.get("x"), pt.get("y"), pt.get("z")
        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)) or not isinstance(z, (int, float)):
            raise TypeError(f"graph[{i}].point must have numeric x,y,z")

        out[nid] = {"x": float(x), "y": float(y), "z": float(z)}

    return out


def tasksets_json_split_top_level(tasksets_json: str) -> List[Dict[str, Any]]:
    obj = json.loads(tasksets_json)
    if not isinstance(obj, list):
        raise TypeError(f"tasksets must be a JSON list, got {type(obj).__name__}")

    out: List[Dict[str, Any]] = []
    for i, item in enumerate(obj):
        if not isinstance(item, dict):
            raise TypeError(f"tasksets[{i}] must be an object, got {type(item).__name__}")
        out.append(item)

    return out


def rewrite_tasksets(
    tasksets_blocks: List[Dict[str, Any]],
    machinery_id_to_name: Dict[int, str],
    node_id_to_point: Dict[int, Dict[str, float]],
) -> List[Dict[str, Any]]:
    tasksets = copy.deepcopy(tasksets_blocks)

    for ts_i, ts in enumerate(tasksets):
        if not isinstance(ts, dict):
            raise TypeError(f"tasksets[{ts_i}] must be an object, got {type(ts).__name__}")

        key = "condition" if "condition" in ts else ("consition" if "consition" in ts else None)
        if key is not None:
            cond = ts.get(key)
            if isinstance(cond, list):
                new_cond = []
                for v in cond:
                    if isinstance(v, int) and v in machinery_id_to_name:
                        new_cond.append(machinery_id_to_name[v])
                    else:
                        new_cond.append(v)
                ts[key] = new_cond

        m_tasks = ts.get("machinery_tasks", [])
        if not isinstance(m_tasks, list):
            raise TypeError(f"tasksets[{ts_i}].machinery_tasks must be a list")

        for mt_i, mt in enumerate(m_tasks):
            if not isinstance(mt, dict):
                raise TypeError(f"machinery_tasks[{mt_i}] must be an object")

            mid = mt.get("machinery")
            if isinstance(mid, int) and mid in machinery_id_to_name:
                mt["machinery"] = machinery_id_to_name[mid]

            tasks = mt.get("tasks", [])
            if not isinstance(tasks, list):
                raise TypeError(f"machinery_tasks[{mt_i}].tasks must be a list")

            for t_i, task in enumerate(tasks):
                if not isinstance(task, dict):
                    raise TypeError(f"tasks[{t_i}] must be an object")

                params = task.get("parameters", {})
                if not isinstance(params, dict):
                    continue

                nxt = params.get("next")
                nxt_point = None
                if isinstance(nxt, int) and nxt in node_id_to_point:
                    nxt_point = node_id_to_point[nxt]
                    params["next"] = nxt_point

                nd = params.get("node")
                if isinstance(nd, int) and nd in node_id_to_point:
                    params["node"] = node_id_to_point[nd]

                if task.get("type") == "移動タスク" and nxt_point is not None:
                    task["id"] = nxt_point

    return tasksets


def load_yaml_dict(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise TypeError(f"YAML root must be dict: {path}")
    return data


def base_model_from_instance_name(instance_name: str) -> str:
    return instance_name.split("_", 1)[0]


def build_action_id_from_machine_type(machine_type: str) -> str:
    mapping = {
        "excavator": "LeafNodeExcavator",
        "crawler_dump": "LeafNodeCrawlerDump",
        "crawlerdump": "LeafNodeCrawlerDump",
        "bulldozer": "LeafNodeBulldozer",
    }
    return mapping.get(machine_type, "LeafNodeUnknown")


def normalize_machine_type(machine_type: str) -> str:
    """
    YAMLキーに合わせて machine_type を正規化する。
    - crawler_dump -> crawlerdump
    """
    if not isinstance(machine_type, str):
        return ""
    return machine_type.replace("_", "").lower()


def resolve_subtask_name(
    task_types_by_machine: Dict[str, Dict[str, str]],
    machine_type: str,
    task_type: str,
) -> str:
    """
    task_types.yaml から machine_type と task_type で subtask を引く。
    見つからなければ空文字。
    """
    mt = normalize_machine_type(machine_type)
    if not mt or not isinstance(task_type, str):
        return ""
    submap = task_types_by_machine.get(mt)
    if not isinstance(submap, dict):
        return ""
    sub = submap.get(task_type, "")
    return sub if isinstance(sub, str) else ""


def append_parallel_block_and_params(
    block: Dict[str, Any],
    parent_sequence: ET.Element,
    model_to_machine_type: Dict[str, str],
    task_types_by_machine: Dict[str, Dict[str, str]],
    record_start_index: int,
) -> Tuple[Dict[str, Any], int]:
    """
    params_store["paramX"] = {
      "machinery_model": "ZX200_1",
      "parameters": {...}
    }
    """
    machinery_tasks = block.get("machinery_tasks", [])
    if not isinstance(machinery_tasks, list):
        raise TypeError("machinery_tasks must be list")

    actions: List[Dict[str, Any]] = []
    for mt in machinery_tasks:
        if not isinstance(mt, dict):
            continue
        inst_name = mt.get("machinery")
        tasks = mt.get("tasks", [])
        if not isinstance(inst_name, str) or not isinstance(tasks, list) or len(tasks) == 0:
            continue
        for task in tasks:
            if isinstance(task, dict):
                actions.append({"machinery": inst_name, "task": task})

    parallel = ET.SubElement(
        parent_sequence,
        "Parallel",
        {
            "failure_threshold": "1",
            "success_threshold": str(len(actions)) if len(actions) > 0 else "0",
        },
    )

    params_store: Dict[str, Any] = {}
    rec_idx = record_start_index

    for a in actions:
        inst_name: str = a["machinery"]
        task: Dict[str, Any] = a["task"]

        model = base_model_from_instance_name(inst_name)          # "ZX200"
        machine_type_raw = model_to_machine_type.get(model, "")   # "excavator" etc
        machine_type = normalize_machine_type(machine_type_raw)   # yaml key: "excavator"/"crawlerdump"/...

        action_id = build_action_id_from_machine_type(machine_type)

        task_type = task.get("type", "")
        if not isinstance(task_type, str):
            task_type = ""

        subtask_name = resolve_subtask_name(task_types_by_machine, machine_type, task_type)

        record_name = f"param{rec_idx}"
        rec_idx += 1

        ET.SubElement(
            parallel,
            "Action",
            {
                "model_name": inst_name.lower(),
                "ID": action_id,
                "record_name": record_name,
                "subtask_name": subtask_name,
            },
        )

        params_store[record_name] = {
            "machinery_model": inst_name.lower(),
            "parameters": task.get("parameters", {}),
        }

    return params_store, rec_idx


class TasksetCompiler(Node):
    def __init__(self) -> None:
        super().__init__("taskset_compiler")

        self.declare_parameter("output_dir", DEFAULT_OUTPUT_DIR)
        self.declare_parameter("xml_filename", DEFAULT_XML_FILENAME)
        self.declare_parameter("params_filename", DEFAULT_PARAMS_FILENAME)

        output_dir = str(self.get_parameter("output_dir").value)
        xml_name = str(self.get_parameter("xml_filename").value).strip()
        params_name = str(self.get_parameter("params_filename").value).strip()

        xml_filename = ensure_ext_no_double(xml_name, ".xml")
        params_filename = ensure_ext_no_double(params_name, ".json")

        self._xml_path = os.path.join(output_dir, xml_filename)
        self._params_path = os.path.join(output_dir, params_filename)

        self._cli = self.create_client(SegmentedScenario, SEGMENTS_SERVICE)
        self.get_logger().info(f"taskset_compiler started. calling {SEGMENTS_SERVICE} ...")
        self.get_logger().info(f"Output XML  : {self._xml_path}")
        self.get_logger().info(f"Output JSON : {self._params_path}")

        self._done = False
        self._done_message = "not finished"
        self._done_srv = self.create_service(Trigger, DONE_SERVICE, self._on_done_service)
        self.get_logger().info(f"Providing done service: {DONE_SERVICE} (std_srvs/Trigger)")

        pkg_share = get_package_share_directory(CONFIG_PKG_NAME)
        machinery_yaml = os.path.join(pkg_share, "data", "machinery_model_and_type.yaml")
        task_types_yaml = os.path.join(pkg_share, "data", "task_types.yaml")

        mcfg = load_yaml_dict(machinery_yaml)
        tcfg = load_yaml_dict(task_types_yaml)

        self._model_to_type: Dict[str, str] = dict(mcfg.get("machine_type_by_model", {}))

        # ★ task_types は machine_type -> (task_type -> subtask) の入れ子辞書を読む
        self._task_types_by_machine: Dict[str, Dict[str, str]] = dict(tcfg.get("task_types", {}))

        self.get_logger().info(f"Loaded machinery YAML: {machinery_yaml}")
        self.get_logger().info(f"Loaded task types YAML: {task_types_yaml}")

        self._timer = self.create_timer(0.1, self._kick_once)
        self._kicked = False

    def _on_done_service(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success = bool(self._done)
        response.message = str(self._done_message)
        return response

    def _kick_once(self) -> None:
        if self._kicked:
            return
        self._kicked = True
        self._timer.cancel()

        if not self._cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service not available: {SEGMENTS_SERVICE}")
            self._done = False
            self._done_message = f"Service not available: {SEGMENTS_SERVICE}"
            return

        future = self._cli.call_async(SegmentedScenario.Request())
        future.add_done_callback(self._on_response)

    def _on_response(self, future) -> None:
        try:
            res: SegmentedScenario.Response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self._done = False
            self._done_message = f"Service call failed: {e}"
            return

        if not res.success:
            self.get_logger().error("segmented_scenario returned success=false")
            self._done = False
            self._done_message = "segmented_scenario returned success=false"
            return

        try:
            machinery_id_to_name = machinery_json_to_unique_id_name(res.machinery)
            node_id_to_point = graph_json_to_id_point(res.graph)
            blocks = tasksets_json_split_top_level(res.tasksets)
        except Exception as e:
            self.get_logger().error(f"Preprocess failed: {e}")
            self._done = False
            self._done_message = f"Preprocess failed: {e}"
            return

        rewritten = rewrite_tasksets(blocks, machinery_id_to_name, node_id_to_point)

        root = ET.Element("root", {"main_tree_to_execute": "BehaviorTree"})
        bt = ET.SubElement(root, "BehaviorTree", {"ID": "BehaviorTree"})
        seq = ET.SubElement(bt, "Sequence")

        record_counter = 1
        all_params: Dict[str, Any] = {}

        for idx, block in enumerate(rewritten, start=1):
            seq.append(ET.Comment(f" block_{idx} "))
            params_store, record_counter = append_parallel_block_and_params(
                block=block,
                parent_sequence=seq,
                model_to_machine_type=self._model_to_type,
                task_types_by_machine=self._task_types_by_machine,
                record_start_index=record_counter,
            )
            all_params.update(params_store)

        xml_bytes = ET.tostring(root, encoding="utf-8", xml_declaration=True)
        xml_text = xml_bytes.decode("utf-8")
        xml_text_one_line = xml_text.replace("\r", "").replace("\n", "")

        try:
            write_text_file(self._xml_path, xml_text_one_line)
            self.get_logger().info(f"Saved XML to: {self._xml_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write XML: {e}")
            self._done = False
            self._done_message = f"Failed to write XML: {e}"
            return

        try:
            write_json_file(self._params_path, all_params)
            self.get_logger().info(f"Saved JSON to: {self._params_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write JSON: {e}")
            self._done = False
            self._done_message = f"Failed to write JSON: {e}"
            return

        self.get_logger().info(f"[sequence_xml_saved_one_line]\n{xml_text_one_line}")
        self.get_logger().info(f"[record_params_dict]\n{json.dumps(all_params, ensure_ascii=False, indent=2)}")

        self._done = True
        self._done_message = "saved xml/json successfully"
        self.get_logger().info("=== Done (files saved) ===")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TasksetCompiler()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
