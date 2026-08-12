#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import re
from typing import Any, Dict, List, Optional
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_srvs.srv import Trigger
import yaml


SEGMENTS_SERVICE = "/segmented_scenario"
DONE_SERVICE = "/taskset_compiler/done"

CONFIG_PKG_NAME = "tms_if_input"

DEFAULT_OUTPUT_DIR = "/tmp"
DEFAULT_XML_FILENAME = "taskset"                      # -> taskset.xml (combined/parallel)
DEFAULT_PARAMS_FILENAME = "record_params"             # -> record_params.json
DEFAULT_INDIVIDUAL_TASKS_FILENAME = "individual_tasks"  # -> individual_tasks.json

# done serviceが最終状態(成功/失敗)に達した後、db_task_writer/db_param_writerが
# ポーリングし終えるのを待つための猶予時間
DONE_SERVICE_GRACE_SEC = 3.0


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


def element_to_one_line_xml(elem: ET.Element) -> str:
    xml_bytes = ET.tostring(elem, encoding="utf-8", xml_declaration=True)
    return xml_bytes.decode("utf-8").replace("\r", "").replace("\n", "")


def load_yaml_dict(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise TypeError(f"YAML root must be dict: {path}")
    return data


def strip_namespaces(elem: ET.Element) -> ET.Element:
    for el in elem.iter():
        if "}" in el.tag:
            el.tag = el.tag.split("}", 1)[1]
    return elem


# datamodel の expr="..." は JavaScriptのオブジェクトリテラルに近い記法
# (例: "{x:2, y:4, z:1.5}", "[\"Node_a\", \"Node_b\"]", "Machine 1", "0.785")。
# キーだけ引用符が無いのでJSONとして読めるように補ってからjson.loadsする。
_BARE_KEY_RE = re.compile(r'([{,]\s*)([A-Za-z_][A-Za-z0-9_]*)\s*:')


def parse_expr(expr: Optional[str]) -> Any:
    if expr is None:
        return None
    s = expr.strip()
    if not s:
        return s

    looks_like_json = s[0] in "{[\"" or s[0].isdigit() or (s[0] == "-" and len(s) > 1 and s[1].isdigit())
    if looks_like_json:
        quoted = _BARE_KEY_RE.sub(r'\1"\2":', s)
        try:
            return json.loads(quoted)
        except json.JSONDecodeError:
            return s

    # "Machine 1" や "Task_00c16fc4" のような裸の識別子は文字列参照として扱う
    return s


def resolve_refs(value: Any, ref_map: Dict[str, Any]) -> Any:
    if isinstance(value, str):
        return ref_map.get(value, value)
    if isinstance(value, list):
        return [resolve_refs(v, ref_map) for v in value]
    if isinstance(value, dict):
        return {k: resolve_refs(v, ref_map) for k, v in value.items()}
    return value


def parse_machines(machines_el: ET.Element) -> Dict[str, str]:
    """<machine id="Machine 1" type="zx200"/> -> {"Machine 1": "zx200_1"}"""
    per_type_count: Dict[str, int] = {}
    machine_id_to_name: Dict[str, str] = {}

    for m in machines_el.findall("machine"):
        mid = m.get("id")
        mtype = (m.get("type") or "").strip().lower()
        if not mid or not mtype:
            continue
        per_type_count[mtype] = per_type_count.get(mtype, 0) + 1
        machine_id_to_name[mid] = f"{mtype}_{per_type_count[mtype]}"

    return machine_id_to_name


def parse_road_network(graphml_el: ET.Element) -> tuple[Dict[str, Dict[str, float]], Dict[str, Any]]:
    """
    <node id="Node_x"><data key="x">1.0</data><data key="y">2.0</data></node>
    <edge source="Node_x" target="Node_y" />
    を読み取り、以下2つを返す:
      - node_id_to_point: {"Node_x": {"x":1.0,"y":2.0}}  (他パラメータ内のnode参照解決用)
      - road_network: {"nodes": [{"id","x","y"}...], "edges": [{"from","to"}...]}  (mongo格納用、そのまま可読)
    """
    node_id_to_point: Dict[str, Dict[str, float]] = {}
    nodes: List[Dict[str, Any]] = []
    edges: List[Dict[str, str]] = []

    graph_el = graphml_el.find("graph")
    if graph_el is None:
        return node_id_to_point, {"nodes": nodes, "edges": edges}

    for node_el in graph_el.findall("node"):
        node_id = node_el.get("id")
        if not node_id:
            continue

        values: Dict[str, str] = {}
        for data_el in node_el.findall("data"):
            key = data_el.get("key")
            if key is not None:
                values[key] = data_el.text or ""

        x, y = values.get("x"), values.get("y")
        if x is None or y is None:
            continue
        try:
            point = {"x": float(x), "y": float(y)}
        except ValueError:
            continue

        node_id_to_point[node_id] = point
        nodes.append({"id": node_id, "x": point["x"], "y": point["y"]})

    for edge_el in graph_el.findall("edge"):
        src = edge_el.get("source")
        tgt = edge_el.get("target")
        if src and tgt:
            edges.append({"from": src, "to": tgt})

    return node_id_to_point, {"nodes": nodes, "edges": edges}


class ScxmlTaskGraph:
    def __init__(self, scxml_el: ET.Element) -> None:
        self.states: Dict[str, Dict[str, Any]] = {}

        for state_el in scxml_el.findall("state"):
            sid = state_el.get("id")
            if not sid:
                continue

            datamodel: Dict[str, str] = {}
            datamodel_el = state_el.find("datamodel")
            if datamodel_el is not None:
                for data_el in datamodel_el.findall("data"):
                    key = data_el.get("id")
                    if key is not None:
                        datamodel[key] = data_el.get("expr")

            targets = [t.get("target") for t in state_el.findall("transition") if t.get("target")]

            self.states[sid] = {
                "name": state_el.get("name") or sid,
                "datamodel": datamodel,
                "targets": targets,
            }

        self.initial_id = scxml_el.get("initial")

    def task_state_ids(self) -> List[str]:
        """Startから直接分岐する全state(=並列実行される最上位タスク群)のidを順序通り返す"""
        if self.initial_id is None or self.initial_id not in self.states:
            raise ValueError(f"scxml initial state not found: {self.initial_id}")
        return list(self.states[self.initial_id]["targets"])


def build_single_task_tree(action_spec: Dict[str, str]) -> ET.Element:
    """1タスクだけをStart->Task->Endのように直結したBehaviorTree(Parallelなし)"""
    root = ET.Element("root", {"main_tree_to_execute": "BehaviorTree"})
    bt = ET.SubElement(root, "BehaviorTree", {"ID": "BehaviorTree"})
    seq = ET.SubElement(bt, "Sequence")
    ET.SubElement(seq, "Action", action_spec)
    return root


def build_params_and_tree(
    machines_el: ET.Element,
    graphml_el: ET.Element,
    scxml_el: ET.Element,
    task_types_map: Dict[str, str],
    logger,
) -> tuple[ET.Element, Dict[str, Any], List[Dict[str, Any]]]:
    machine_id_to_name = parse_machines(machines_el)
    node_id_to_point, road_network = parse_road_network(graphml_el)
    task_graph = ScxmlTaskGraph(scxml_el)
    task_state_ids = task_graph.task_state_ids()

    record_names: Dict[str, str] = {
        sid: f"param{i}" for i, sid in enumerate(task_state_ids, start=1)
    }

    ref_map: Dict[str, Any] = dict(node_id_to_point)
    ref_map.update(record_names)

    all_params: Dict[str, Any] = {}
    action_specs: List[Dict[str, str]] = []

    for sid in task_state_ids:
        state = task_graph.states[sid]
        task_name = state["name"]
        record_name = record_names[sid]

        entries = dict(state["datamodel"])
        machine_expr = entries.pop("machine", None)
        machine_id = parse_expr(machine_expr) if machine_expr is not None else None
        model_name = machine_id_to_name.get(machine_id, "") if isinstance(machine_id, str) else ""
        if not model_name:
            logger.warn(f"[{sid}] unresolved machine reference: {machine_expr!r}")

        params: Dict[str, Any] = {}
        for key, expr in entries.items():
            raw = parse_expr(expr)
            params[key] = resolve_refs(raw, ref_map)

        all_params[record_name] = {
            "model_name": model_name,
            "type": "dynamic",
            "task_type": task_name,
            "record_name": record_name,
            **params,
        }

        # Transportタスクは経路計画のためにver3.xmlの道路網(ノード座標+接続関係)をそのまま持たせる
        if task_name == "Transport":
            all_params[record_name]["road_network"] = road_network

        action_id = task_types_map.get(task_name)
        if action_id is None:
            logger.warn(f"[{sid}] unknown task type '{task_name}' (not in task_types.yaml); using as-is")
            action_id = task_name

        action_specs.append({
            "model_name": model_name,
            "ID": action_id,
            "record_name": record_name,
        })

    root = ET.Element("root", {"main_tree_to_execute": "BehaviorTree"})
    bt = ET.SubElement(root, "BehaviorTree", {"ID": "BehaviorTree"})
    seq = ET.SubElement(bt, "Sequence")
    parallel = ET.SubElement(seq, "Parallel", {
        "failure_threshold": "1",
        "success_threshold": str(len(action_specs)),
    })
    for spec in action_specs:
        ET.SubElement(parallel, "Action", spec)

    individual_trees: List[Dict[str, Any]] = [
        {
            "description": f"{spec['ID']} (model={spec['model_name']}, record={spec['record_name']})",
            "tree": build_single_task_tree(spec),
        }
        for spec in action_specs
    ]

    return root, all_params, individual_trees


class TasksetCompiler(Node):
    def __init__(self) -> None:
        super().__init__("taskset_compiler")

        self.declare_parameter("output_dir", DEFAULT_OUTPUT_DIR)
        self.declare_parameter("xml_filename", DEFAULT_XML_FILENAME)
        self.declare_parameter("params_filename", DEFAULT_PARAMS_FILENAME)
        self.declare_parameter("individual_tasks_filename", DEFAULT_INDIVIDUAL_TASKS_FILENAME)

        output_dir = str(self.get_parameter("output_dir").value)
        xml_name = str(self.get_parameter("xml_filename").value).strip()
        params_name = str(self.get_parameter("params_filename").value).strip()
        individual_tasks_name = str(self.get_parameter("individual_tasks_filename").value).strip()

        xml_filename = ensure_ext_no_double(xml_name, ".xml")
        params_filename = ensure_ext_no_double(params_name, ".json")
        individual_tasks_filename = ensure_ext_no_double(individual_tasks_name, ".json")

        self._xml_path = os.path.join(output_dir, xml_filename)
        self._params_path = os.path.join(output_dir, params_filename)
        self._individual_tasks_path = os.path.join(output_dir, individual_tasks_filename)

        self._cli = self.create_client(Trigger, SEGMENTS_SERVICE)
        self.get_logger().info(f"taskset_compiler started. calling {SEGMENTS_SERVICE} ...")
        self.get_logger().info(f"Output XML (combined/parallel)  : {self._xml_path}")
        self.get_logger().info(f"Output JSON (params)            : {self._params_path}")
        self.get_logger().info(f"Output JSON (individual tasks)  : {self._individual_tasks_path}")

        self._done = False
        self._done_message = "not finished"
        self._done_srv = self.create_service(Trigger, DONE_SERVICE, self._on_done_service)
        self.get_logger().info(f"Providing done service: {DONE_SERVICE} (std_srvs/Trigger)")

        pkg_share = get_package_share_directory(CONFIG_PKG_NAME)
        task_types_yaml = os.path.join(pkg_share, "data", "task_types.yaml")

        tcfg = load_yaml_dict(task_types_yaml)
        self._task_types_map: Dict[str, str] = dict(tcfg.get("task_types", {}))

        self.get_logger().info(f"Loaded task types YAML: {task_types_yaml}")

        self._timer = self.create_timer(0.1, self._kick_once)
        self._kicked = False

        # done serviceが最終状態に達してから一定時間経ったらexit_requestedを立てる。
        # rclpy.shutdown()はここ(コールバック)からではなくmain()の手動spinループから呼ぶ。
        self.exit_requested = False
        self._finished_at: Optional[float] = None
        self._exit_timer = self.create_timer(0.5, self._maybe_request_exit)

    def _mark_finished(self) -> None:
        if self._finished_at is None:
            self._finished_at = self.get_clock().now().nanoseconds / 1e9

    def _maybe_request_exit(self) -> None:
        if self._finished_at is None:
            return
        elapsed = self.get_clock().now().nanoseconds / 1e9 - self._finished_at
        if elapsed < DONE_SERVICE_GRACE_SEC:
            return
        self._exit_timer.cancel()
        self.get_logger().info(
            f"Grace period elapsed ({DONE_SERVICE_GRACE_SEC}s). Shutting down taskset_compiler."
        )
        self.exit_requested = True

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
            self._mark_finished()
            return

        future = self._cli.call_async(Trigger.Request())
        future.add_done_callback(self._on_response)

    def _on_response(self, future) -> None:
        try:
            res: Trigger.Response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self._done = False
            self._done_message = f"Service call failed: {e}"
            self._mark_finished()
            return

        if not res.success:
            self.get_logger().error("segmented_scenario returned success=false")
            self._done = False
            self._done_message = "segmented_scenario returned success=false"
            self._mark_finished()
            return

        try:
            root = ET.fromstring(res.message)
            strip_namespaces(root)

            machines_el = root.find("machines")
            graphml_el = root.find("graphml")
            scxml_el = root.find("scxml")
            if machines_el is None or graphml_el is None or scxml_el is None:
                raise ValueError("ConstructionPlan must contain <machines>, <graphml>, <scxml>")

            bt_root, all_params, individual_trees = build_params_and_tree(
                machines_el=machines_el,
                graphml_el=graphml_el,
                scxml_el=scxml_el,
                task_types_map=self._task_types_map,
                logger=self.get_logger(),
            )
        except Exception as e:
            self.get_logger().error(f"Compile failed: {e}")
            self._done = False
            self._done_message = f"Compile failed: {e}"
            self._mark_finished()
            return

        xml_text_one_line = element_to_one_line_xml(bt_root)
        individual_docs = [
            {
                "description": item["description"],
                "task_sequence": element_to_one_line_xml(item["tree"]),
            }
            for item in individual_trees
        ]

        try:
            write_text_file(self._xml_path, xml_text_one_line)
            self.get_logger().info(f"Saved XML to: {self._xml_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write XML: {e}")
            self._done = False
            self._done_message = f"Failed to write XML: {e}"
            self._mark_finished()
            return

        try:
            write_json_file(self._individual_tasks_path, individual_docs)
            self.get_logger().info(
                f"Saved individual tasks ({len(individual_docs)}) to: {self._individual_tasks_path}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to write individual tasks JSON: {e}")
            self._done = False
            self._done_message = f"Failed to write individual tasks JSON: {e}"
            self._mark_finished()
            return

        try:
            write_json_file(self._params_path, all_params)
            self.get_logger().info(f"Saved JSON to: {self._params_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write JSON: {e}")
            self._done = False
            self._done_message = f"Failed to write JSON: {e}"
            self._mark_finished()
            return

        self.get_logger().info(f"[sequence_xml_saved_one_line]\n{xml_text_one_line}")
        self.get_logger().info(f"[record_params_dict]\n{json.dumps(all_params, ensure_ascii=False, indent=2)}")

        self._done = True
        self._done_message = "saved xml/json successfully"
        self.get_logger().info("=== Done (files saved) ===")
        self._mark_finished()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TasksetCompiler()

    # rclpy.shutdown()をコールバックの中から呼ぶとspin()が正しく戻らないため、
    # メインスレッドでフラグを見ながら手動でspin_once()する。
    while rclpy.ok() and not node.exit_requested:
        rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
