#!/usr/bin/env python3
"""4 机场景3 仿真测试客户端（替代 references/runtime 的 TCP 发送端）。

references/runtime 的 `task_test.py --backend tcp` 发送的是 15 机整组批次
（prepare 含 A01..A15），而 4 机仿真的机载层只启动 UAV1..UAV3（UAV4 预留位）。
若直接用 runtime，GCS_A 会把任务分派到不存在的 UAV（A05..A15）导致超时失败。

本脚本是纯 TCP 工具（无 ROS / runtime 依赖），按 `references/runtime` 的
`uav_bt_runtime/tcp_transport.py` 线协议发送 4 机子集批次：

  发送: {"version":"1.0","type":"COMMAND","mission_id":...,"group_id":"GroupA",
         "command_id":"tcp-...","command":"MOVE_TO","timeout_s":...,
         "assignments":[{"uav_id":"A01","x":..,"y":..,"z":..,"yaw":..}, ...]}
  接收: {"version":"1.0","type":"STATUS","group_id":"GroupA","mission_id":...,
         "command_id":"tcp-...","status":"ACCEPTED"} / "COMPLETED" / "FAILED"

批次数据来自 `references/runtime/examples/joint_mission/group_a/plans/plans.json`
中 A01..A04 的坐标与航点，仅保留 4 机仿真可覆盖的子集：
  - prepare:            MOVE_TO A01..A03（A04 可选）
  - coverage-segment-1: FOLLOW_ROUTE A01 leader + A02..A03 formation_follow
  - recovery:           MOVE_TO A01..A03（A04 可选）
  - hold-end:           HOVER A01..A03（A04 可选）

用法（在任意终端，无 ROS 环境要求）：
  python3 src/tcp_to_ros/scripts/tcp_4sim_test_client.py \
      --host 127.0.0.1 --port 39001 \
      --tasks prepare,coverage-segment-1,hold-end
  # 全部批次（含 A04 预留位参与，需先启动 UAV4 机载层）：
  python3 src/tcp_to_ros/scripts/tcp_4sim_test_client.py --include-a04 \
      --tasks prepare,coverage-segment-1,recovery,hold-end
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
import uuid
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

PROTOCOL_VERSION = "1.0"
GROUP_ID = "GroupA"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 39001
MAX_MESSAGE_BYTES = 64 * 1024
MAX_BATCH_WAIT_S = 30.0      # 单批次总等待（ACCEPTED 5s + 终态 25s）


# ---- 批次数据（来自 plans.json 的 A01..A04 子集） -------------------------------

def _pose(x: float, y: float, z: float, yaw: float = 0.0) -> Dict[str, float]:
    return {"x": x, "y": y, "z": z, "yaw": yaw}


# prepare-a（MOVE_TO，z=12）
PREPARE_ASSIGNMENTS: List[Dict[str, object]] = [
    {"uav_id": "A01", **_pose(17.5, 50.0, 12.0)},
    {"uav_id": "A02", **_pose(12.5, 45.0, 12.0)},
    {"uav_id": "A03", **_pose(17.5, 45.0, 12.0)},
    {"uav_id": "A04", **_pose(22.5, 45.0, 12.0)},
]

# coverage-segment-1（FOLLOW_ROUTE：A01 leader 11 航点 + followers formation）
COVERAGE1_LEADER_WAYPOINTS: List[Dict[str, float]] = [
    _pose(15.0, 12.0, 12.0), _pose(80.0, 12.0, 12.0), _pose(80.0, 26.0, 12.0, 3.141593),
    _pose(15.0, 26.0, 12.0, 3.141593), _pose(15.0, 40.0, 12.0), _pose(80.0, 40.0, 12.0),
    _pose(80.0, 54.0, 12.0, 3.141593), _pose(15.0, 54.0, 12.0, 3.141593),
    _pose(15.0, 68.0, 12.0), _pose(80.0, 68.0, 12.0), _pose(80.0, 75.0, 12.0, 1.570796),
]


def _coverage1_assignments(include_a04: bool) -> List[Dict[str, object]]:
    assignments: List[Dict[str, object]] = [
        {"uav_id": "A01", "waypoints": COVERAGE1_LEADER_WAYPOINTS},
        {"uav_id": "A02", "formation_follow": True},
        {"uav_id": "A03", "formation_follow": True},
    ]
    if include_a04:
        assignments.append({"uav_id": "A04", "formation_follow": True})
    return assignments


FOLLOW_ROUTE_LEADER = "A01"

# recovery-a（MOVE_TO，z=12）
RECOVERY_ASSIGNMENTS: List[Dict[str, object]] = [
    {"uav_id": "A01", **_pose(80.0, 75.0, 12.0)},
    {"uav_id": "A02", **_pose(75.0, 70.0, 12.0)},
    {"uav_id": "A03", **_pose(80.0, 70.0, 12.0)},
    {"uav_id": "A04", **_pose(85.0, 70.0, 12.0)},
]

# hold-a-end（HOVER：runtime 的 HOVER assignment 只携带 uav_id）
HOLD_ASSIGNMENTS: List[Dict[str, object]] = [
    {"uav_id": "A01"},
    {"uav_id": "A02"},
    {"uav_id": "A03"},
    {"uav_id": "A04"},
]


class BatchResult:
    def __init__(self, task: str, command: str, ok: bool, detail: str = "") -> None:
        self.task = task
        self.command = command
        self.ok = ok
        self.detail = detail

    def to_line(self) -> str:
        mark = "[PASS]" if self.ok else "[FAIL]"
        return f"{mark} {self.task:<22} {self.command:<12} {self.detail}"


def build_tasks(
    task_names: Sequence[str], include_a04: bool
) -> List[Tuple[str, str, str, List[Dict[str, object]], Optional[str]]]:
    """返回 (任务名, command, plan_id, assignments, leader_id) 列表。"""
    tasks: List[Tuple[str, str, str, List[Dict[str, object]], Optional[str]]] = []
    for name in task_names:
        if name == "prepare":
            tasks.append((
                "prepare", "MOVE_TO", "prepare-a",
                [a for a in PREPARE_ASSIGNMENTS if include_a04 or a["uav_id"] != "A04"],
                None,
            ))
        elif name == "coverage-segment-1":
            tasks.append((
                "coverage-segment-1", "FOLLOW_ROUTE", "coverage-segment-1",
                _coverage1_assignments(include_a04), FOLLOW_ROUTE_LEADER,
            ))
        elif name == "recovery":
            tasks.append((
                "recovery", "MOVE_TO", "recovery-a",
                [a for a in RECOVERY_ASSIGNMENTS if include_a04 or a["uav_id"] != "A04"],
                None,
            ))
        elif name == "hold-end":
            tasks.append((
                "hold-end", "HOVER", "hold-a-end",
                [a for a in HOLD_ASSIGNMENTS if include_a04 or a["uav_id"] != "A04"],
                None,
            ))
        else:
            raise ValueError(
                f"未知任务 {name!r}；支持 prepare/coverage-segment-1/recovery/hold-end"
            )
    return tasks


class Tcp4SimClient:
    """连接 GCS_A TCP Server，逐批次发送 4 机命令并等待整组终态。"""

    def __init__(self, host: str, port: int, connect_timeout_s: float = 5.0) -> None:
        self.host = host
        self.port = int(port)
        self.last_mission_id = ""
        self.last_wire_command_id = ""
        try:
            self._sock = socket.create_connection(
                (host, int(port)), timeout=connect_timeout_s
            )
        except OSError as exc:
            raise ConnectionError(
                f"无法连接 GCS_A TCP Server {host}:{port}: {exc}"
            ) from exc
        self._sock.settimeout(0.5)
        self._buffer = bytearray()
        self._connected = True

    # ---- 发送 ----

    def send_batch(
        self,
        command: str,
        plan_id: str,
        assignments: Sequence[Mapping[str, object]],
        leader_id: Optional[str],
        timeout_s: float = 15.0,
    ) -> None:
        del plan_id  # plan_id 保留在签名中以对齐 build_tasks 元组结构
        mission_id = f"joint-mission-4sim-{uuid.uuid4().hex[:8]}"
        wire_command_id = f"tcp-{uuid.uuid4().hex}"
        message: Dict[str, object] = {
            "version": PROTOCOL_VERSION,
            "type": "COMMAND",
            "mission_id": mission_id,
            "group_id": GROUP_ID,
            "command_id": wire_command_id,
            "command": command,
            "timeout_s": timeout_s,
            "assignments": [dict(item) for item in assignments],
        }
        if leader_id:
            message["leader_id"] = leader_id
        payload = (
            json.dumps(message, ensure_ascii=False, separators=(",", ":"))
            .encode("utf-8")
            + b"\n"
        )
        if len(payload) > MAX_MESSAGE_BYTES:
            raise ValueError("TCP JSON 消息超过 64 KiB")
        self.last_mission_id = mission_id
        self.last_wire_command_id = wire_command_id
        self._send_raw(payload)

    def _send_raw(self, payload: bytes) -> None:
        if not self._connected:
            raise ConnectionError("TCP 连接已关闭")
        try:
            self._sock.sendall(payload)
        except OSError as exc:
            self._connected = False
            raise ConnectionError(f"发送失败: {exc}") from exc

    # ---- 接收 ----

    def _read_line(self) -> Optional[dict]:
        while True:
            if b"\n" in self._buffer:
                raw, _, remainder = self._buffer.partition(b"\n")
                self._buffer = bytearray(remainder)
                if not raw.strip():
                    continue
                try:
                    return json.loads(raw.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                    raise ValueError(f"收到非法 JSON 状态行: {exc}") from exc
            if not self._connected:
                return None
            try:
                chunk = self._sock.recv(4096)
            except socket.timeout:
                continue
            except OSError as exc:
                self._connected = False
                raise ConnectionError(f"接收失败: {exc}") from exc
            if not chunk:
                self._connected = False
                raise ConnectionError("GCS_A 关闭了 TCP 连接")
            self._buffer.extend(chunk)
            if len(self._buffer) > MAX_MESSAGE_BYTES and b"\n" not in self._buffer:
                raise ValueError("GCS_A 发送了超过 64 KiB 的消息")

    def wait_terminal(
        self, expected_uavs: set, deadline_s: float = MAX_BATCH_WAIT_S
    ) -> Dict[str, object]:
        """等待与最近发送批次匹配的 ACCEPTED→终态，返回终态消息。"""
        started = time.monotonic()
        accepted = False
        while True:
            remaining = deadline_s - (time.monotonic() - started)
            if remaining <= 0:
                raise TimeoutError(
                    f"批次 {self.last_wire_command_id} 等待超时（{deadline_s:g}s）"
                )
            msg = self._read_line()
            if msg is None:
                raise ConnectionError("连接已断开，无法获得批次终态")
            if (
                msg.get("version") != PROTOCOL_VERSION
                or msg.get("type") != "STATUS"
                or msg.get("group_id") != GROUP_ID
                or msg.get("mission_id") != self.last_mission_id
                or msg.get("command_id") != self.last_wire_command_id
            ):
                continue
            status = msg.get("status")
            if status == "ACCEPTED":
                accepted = True
                continue
            if status == "COMPLETED":
                completed = msg.get("completed_uavs")
                if not isinstance(completed, list) or set(completed) != expected_uavs:
                    raise ValueError(
                        f"COMPLETED 参与机与期望不符: {completed} != {sorted(expected_uavs)}"
                    )
                return msg
            if status == "FAILED":
                return msg
            raise ValueError(f"收到不支持的状态 {status!r}")

    def close(self) -> None:
        if not self._connected:
            return
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self._sock.close()
        except OSError:
            pass
        self._connected = False


def run_batch(
    client: Tcp4SimClient,
    task: str,
    command: str,
    plan_id: str,
    assignments: Sequence[Mapping[str, object]],
    leader_id: Optional[str],
    timeout_s: float,
) -> BatchResult:
    uav_ids = {str(item["uav_id"]) for item in assignments}
    client.send_batch(command, plan_id, assignments, leader_id, timeout_s=timeout_s)
    terminal = client.wait_terminal(expected_uavs=uav_ids)
    status = terminal.get("status")
    if status == "COMPLETED":
        return BatchResult(task, command, True, f"COMPLETED uavs={sorted(uav_ids)}")
    failed = terminal.get("failed_uavs")
    error = terminal.get("error_code") or terminal.get("message") or "FAILED"
    return BatchResult(task, command, False, f"FAILED uavs={failed} error={error}")


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="4机场景3 TCP 测试客户端")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument(
        "--tasks",
        default="prepare,coverage-segment-1,hold-end",
        help="逗号分隔任务：prepare/coverage-segment-1/recovery/hold-end",
    )
    parser.add_argument(
        "--include-a04",
        action="store_true",
        help="把 A04(UAV4 预留位) 纳入参与集（需额外启动 UAV4 机载层）",
    )
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--batch-timeout", type=float, default=MAX_BATCH_WAIT_S)
    parser.add_argument("--command-timeout", type=float, default=15.0)
    args = parser.parse_args(argv)

    task_names = [name.strip() for name in args.tasks.split(",") if name.strip()]
    tasks = build_tasks(task_names, args.include_a04)
    if not tasks:
        print("[ERROR] 未指定任何任务", file=sys.stderr)
        return 2

    client: Optional[Tcp4SimClient] = None
    results: List[BatchResult] = []
    try:
        client = Tcp4SimClient(args.host, args.port, args.connect_timeout)
        print(
            f"已连接 GCS_A TCP Server {args.host}:{args.port}，"
            f"共 {len(tasks)} 个批次开始执行"
        )
        for task, command, plan_id, assignments, leader_id in tasks:
            print(
                f"  -> 发送 {task!r} ({command}, {len(assignments)} 机)...",
                flush=True,
            )
            try:
                result = run_batch(
                    client,
                    task,
                    command,
                    plan_id,
                    assignments,
                    leader_id,
                    timeout_s=args.command_timeout,
                )
            except (TimeoutError, ConnectionError, ValueError) as exc:
                result = BatchResult(task, command, False, str(exc))
            results.append(result)
            print("     " + result.to_line(), flush=True)
            if not result.ok:
                print("[ABORT] 批次失败，停止后续批次", file=sys.stderr)
                break
    except ConnectionError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2
    finally:
        if client is not None:
            client.close()

    failed = [r for r in results if not r.ok]
    print("\n===== 汇总 =====")
    for result in results:
        print("  " + result.to_line())
    if failed:
        print(f"结果: {len(results) - len(failed)}/{len(results)} PASS")
        return 1
    print("结果: 全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())