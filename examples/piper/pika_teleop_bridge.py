#!/usr/bin/env python3
"""ROS 2 Humble <-> LeRobot local TCP IK bridge for Pika handheld teleoperation of AgileX Piper.

Runs inside the `pika` environment (`source ~/lerobot/activate_pika.bash`), subscribing to:
  - `/pika_pose` (`geometry_msgs/msg/PoseStamped`, Vive Tracker 6-DoF handle pose)
  - `/gripper/joint_state` & `/sensor/gripper/joint_state` (`sensor_msgs/msg/JointState`, width in m)
  - `/teleop_trigger` (`std_srvs/srv/Trigger`, Pika handle button clutch toggle)

Solves 6-DoF Piper inverse kinematics using Pinocchio + CasADi (`piper_with_gripper_description.urdf`,
matching `arm_ik_pose_node.piper.yaml` and `teleop_single_piper.launch.py`) and serves 7-D joint
targets (`joint_1..6` in radians, `gripper` in metres) to LeRobot's `PikaLeader` (`--teleop.type=pika_leader`)
over a low-latency localhost TCP JSON-RPC socket (`127.0.0.1:18765`).

Important: This bridge NEVER opens `can0` or any RealSense camera; `PiperFollower` in the LeRobot
process retains exclusive ownership of CAN and camera hardware.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
from pathlib import Path
import socket
import socketserver
import threading
import time
from typing import Any, Protocol

import numpy as np

logger = logging.getLogger("pika_teleop_bridge")


# ---------------------------------------------------------------------------
# Pure NumPy SE(3) utilities (matching scipy Rotation extrinsic "xyz" & xyzw quat)
# ---------------------------------------------------------------------------


def euler_xyz_to_mat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Extrinsic XYZ Euler angles (radians) -> 3x3 rotation matrix: Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    return rz @ ry @ rx


def quat_xyzw_to_mat(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Unit quaternion (x, y, z, w) -> 3x3 rotation matrix."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("Zero-norm quaternion in Pika pose.")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def xyzrpy_to_se3(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
    mat = np.eye(4, dtype=float)
    mat[:3, :3] = euler_xyz_to_mat(roll, pitch, yaw)
    mat[:3, 3] = [x, y, z]
    return mat


def mat_to_quat_xyzw(rot: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 rotation matrix -> unit quaternion (x, y, z, w)."""
    trace = float(rot[0, 0] + rot[1, 1] + rot[2, 2])
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (rot[2, 1] - rot[1, 2]) * s
        y = (rot[0, 2] - rot[2, 0]) * s
        z = (rot[1, 0] - rot[0, 1]) * s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = 2.0 * math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2])
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = 2.0 * math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2])
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1])
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return float(x / norm), float(y / norm), float(z / norm), float(w / norm)


# ---------------------------------------------------------------------------
# IK Solver interface & Pinocchio+CasADi Piper implementation
# ---------------------------------------------------------------------------


class IKSolverProtocol(Protocol):
    def sync_state(self, q_current: list[float]) -> None: ...
    def get_pose_mat(self, q: list[float] | np.ndarray) -> np.ndarray: ...
    def solve(self, target_pose: np.ndarray) -> np.ndarray: ...


def resolve_piper_urdf(explicit_urdf: str | None = None) -> tuple[str, list[str]]:
    """Locate `piper_with_gripper_description.urdf` and Pinocchio `package_dirs`."""
    rel = Path("agx_arm_urdf/piper/urdf/piper_with_gripper_description.urdf")
    candidates: list[tuple[Path, Path]] = []

    if explicit_urdf:
        urdf_p = Path(explicit_urdf).expanduser().resolve()
        # Package root `agx_arm_description` is 4 levels up from `.../piper/urdf/file.urdf`;
        # Pinocchio wants the directory *containing* `agx_arm_description`.
        pkg_parent = urdf_p.parents[4] if len(urdf_p.parents) >= 5 else urdf_p.parent
        candidates.append((urdf_p, pkg_parent))

    try:
        from ament_index_python.packages import get_package_share_directory

        share_dir = Path(get_package_share_directory("agx_arm_description")).resolve()
        candidates.append((share_dir / rel, share_dir.parent))
    except Exception:
        pass

    workspace_root = Path(__file__).resolve().parents[3]
    for base in (
        workspace_root / "pika/pika_ros/install/agx_arm_description/share/agx_arm_description",
        workspace_root
        / "pika/pika_ros/src/PikaAnyArm/agx_arm/agx_arm_ros/src/agx_arm_description",
        Path.home() / "lerobot/pika/pika_ros/install/agx_arm_description/share/agx_arm_description",
        Path.home()
        / "lerobot/pika/pika_ros/src/PikaAnyArm/agx_arm/agx_arm_ros/src/agx_arm_description",
    ):
        candidates.append((base / rel, base.parent))

    for urdf_path, pkg_dir in candidates:
        if urdf_path.is_file():
            return str(urdf_path), [str(pkg_dir)]

    searched = [str(u) for u, _ in candidates]
    raise FileNotFoundError(
        f"Could not find piper_with_gripper_description.urdf. Searched: {searched}"
    )


class PiperPinocchioIK:
    """6-DoF damped SE(3) inverse kinematics for Piper (matches `arm_ik_pose_node.piper.yaml`)."""

    def __init__(
        self,
        urdf_path: str | None = None,
        *,
        locked_joints: tuple[str, ...] = (
            "gripper",
            "gripper_base_joint",
            "gripper_joint1",
            "gripper_joint2",
        ),
        active_arm_joints: tuple[str, ...] = (
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ),
        ee_parent_joint: str = "joint6",
        ee_frame_name: str = "ee",
        tool_pre_rot_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
        tool_translation_xyz: tuple[float, float, float] = (0.0, 0.0, 0.13),
        w_pos: float = 10.0,
        w_ori: float = 2.0,
        w_reg: float = 0.01,
        w_smooth: float = 0.1,
        ipopt_max_iter: int = 50,
        ipopt_tol: float = 1e-4,
    ) -> None:
        import casadi
        import pinocchio as pin
        from pinocchio import casadi as cpin

        resolved_urdf, package_dirs = resolve_piper_urdf(urdf_path)
        self.urdf_path = resolved_urdf

        full_robot = pin.RobotWrapper.BuildFromURDF(resolved_urdf, package_dirs=package_dirs)
        active_set = set(active_arm_joints)
        candidates_to_lock = list(locked_joints) + [
            name for name in full_robot.model.names if name != "universe" and name not in active_set
        ]
        valid_locked = []
        seen_ids: set[int] = set()
        for name in candidates_to_lock:
            try:
                jid = full_robot.model.getJointId(name)
            except Exception:
                continue
            if jid > 0 and jid not in seen_ids:
                seen_ids.add(jid)
                valid_locked.append(name)

        self.reduced_robot = full_robot.buildReducedRobot(
            list_of_joints_to_lock=valid_locked,
            reference_configuration=np.zeros(full_robot.model.nq),
        )

        ee_mat = xyzrpy_to_se3(0.0, 0.0, 0.0, *tool_pre_rot_rpy) @ xyzrpy_to_se3(
            *tool_translation_xyz, 0.0, 0.0, 0.0
        )
        qx, qy, qz, qw = mat_to_quat_xyzw(ee_mat[:3, :3])
        self.reduced_robot.model.addFrame(
            pin.Frame(
                ee_frame_name,
                self.reduced_robot.model.getJointId(ee_parent_joint),
                pin.SE3(pin.Quaternion(qw, qx, qy, qz), np.array(ee_mat[:3, 3])),
                pin.FrameType.OP_FRAME,
            )
        )
        self.reduced_robot.data = self.reduced_robot.model.createData()
        self.ee_id = self.reduced_robot.model.getFrameId(ee_frame_name)
        self._pin = pin

        cmodel = cpin.Model(self.reduced_robot.model)
        cdata = cmodel.createData()
        cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        ctf = casadi.SX.sym("tf", 4, 4)
        cpin.framesForwardKinematics(cmodel, cdata, cq)
        error_fn = casadi.Function(
            "error",
            [cq, ctf],
            [casadi.vertcat(cpin.log6(cdata.oMf[self.ee_id].inverse() * cpin.SE3(ctf)).vector)],
        )

        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.param_q_prev = self.opti.parameter(self.reduced_robot.model.nq)
        self.param_q_anchor = self.opti.parameter(self.reduced_robot.model.nq)
        self.param_tf = self.opti.parameter(4, 4)

        err = error_fn(self.var_q, self.param_tf)
        cost = (
            casadi.sumsqr(w_pos * err[:3])
            + casadi.sumsqr(w_ori * err[3:])
            + w_reg * casadi.sumsqr(self.var_q - self.param_q_anchor)
            + w_smooth * casadi.sumsqr(self.var_q - self.param_q_prev)
        )
        self.opti.minimize(cost)
        self.opti.subject_to(
            self.opti.bounded(
                self.reduced_robot.model.lowerPositionLimit,
                self.var_q,
                self.reduced_robot.model.upperPositionLimit,
            )
        )
        self.opti.solver(
            "ipopt",
            {"ipopt": {"print_level": 0, "max_iter": ipopt_max_iter, "tol": ipopt_tol}, "print_time": False},
        )

        self.nq: int = int(self.reduced_robot.model.nq)
        self.lower_limits = np.asarray(self.reduced_robot.model.lowerPositionLimit, dtype=float).reshape(-1)
        self.upper_limits = np.asarray(self.reduced_robot.model.upperPositionLimit, dtype=float).reshape(-1)
        self.init_data = np.zeros(self.nq, dtype=float)
        self.history_data = np.zeros(self.nq, dtype=float)
        self.anchor_data = np.zeros(self.nq, dtype=float)

    def sync_state(self, q_current: list[float]) -> None:
        q = np.clip(np.asarray(q_current, dtype=float).reshape(-1), self.lower_limits, self.upper_limits)
        if q.shape[0] == self.nq:
            self.init_data = q.copy()
            self.history_data = q.copy()
            self.anchor_data = q.copy()

    def get_pose_mat(self, q: list[float] | np.ndarray) -> np.ndarray:
        q_arr = np.asarray(q, dtype=float).reshape(-1)
        self._pin.framesForwardKinematics(self.reduced_robot.model, self.reduced_robot.data, q_arr)
        return np.array(self.reduced_robot.data.oMf[self.ee_id].homogeneous, dtype=float)

    def solve(self, target_pose: np.ndarray) -> np.ndarray:
        self.opti.set_initial(self.var_q, self.init_data)
        self.opti.set_value(self.param_q_prev, self.history_data)
        self.opti.set_value(self.param_q_anchor, self.anchor_data)
        self.opti.set_value(self.param_tf, target_pose)
        self.opti.solve_limited()
        sol_q = np.asarray(self.opti.value(self.var_q), dtype=float).reshape(-1)
        sol_q = np.clip(sol_q, self.lower_limits, self.upper_limits)
        self.init_data = sol_q.copy()
        self.history_data = sol_q.copy()
        return sol_q


# ---------------------------------------------------------------------------
# Stateful clutch + relative-SE(3) controller (pure Python/NumPy, testable without ROS)
# ---------------------------------------------------------------------------


class PikaIKController:
    """Thread-safe clutch state machine + relative SE(3) delta -> 7-D Piper joint targets."""

    def __init__(
        self,
        ik_solver: IKSolverProtocol,
        *,
        handle_pose_rpy: tuple[float, float, float] = (0.0, 1.57, 0.0),
        gripper_max_range: float = 0.07,
        gripper_scale: float = 1.0,
        gripper_offset: float = 0.0,
        pose_timeout_s: float = 0.5,
        max_joint_step_rad: float = 0.35,
    ) -> None:
        self.ik = ik_solver
        self._lock = threading.Lock()
        self._handle_rot_offset = euler_xyz_to_mat(*handle_pose_rpy)
        self.gripper_max_range = float(gripper_max_range)
        self.gripper_scale = float(gripper_scale)
        self.gripper_offset = float(gripper_offset)
        self.pose_timeout_s = float(pose_timeout_s)
        self.max_joint_step_rad = float(max_joint_step_rad)

        self._handle_matrix: np.ndarray | None = None
        self._handle_ts: float | None = None
        self._gripper_width: float | None = None
        self._gripper_ts: float | None = None

        self._engaged: bool = False
        self._baseline_matrix: np.ndarray | None = None
        self._reanchor_requested: bool = True
        self._held_joints: list[float] = [0.0, 0.0, 0.0, 0.0, 0.52, 0.0]
        self._held_gripper: float = 0.0

    def update_handle_pose(self, raw_pose_mat: np.ndarray, *, now: float | None = None) -> None:
        ts = time.monotonic() if now is None else float(now)
        mat = np.array(raw_pose_mat, dtype=float, copy=True)
        mat[:3, :3] = mat[:3, :3] @ self._handle_rot_offset
        with self._lock:
            self._handle_matrix = mat
            self._handle_ts = ts

    def update_gripper(self, raw_width_m: float, *, now: float | None = None) -> None:
        ts = time.monotonic() if now is None else float(now)
        mapped = float(raw_width_m) * self.gripper_scale + self.gripper_offset
        clamped = max(0.0, min(mapped, self.gripper_max_range))
        with self._lock:
            self._gripper_width = clamped
            self._gripper_ts = ts

    def toggle_clutch(self) -> tuple[bool, str]:
        """Handle Pika hardware button (`/teleop_trigger`): toggle clutch engage/disengage."""
        with self._lock:
            if self._engaged:
                self._engaged = False
                self._baseline_matrix = None
                self._reanchor_requested = False
                return False, "Clutch disengaged (holding current robot pose)"
            self._engaged = True
            self._baseline_matrix = None
            self._reanchor_requested = True
            return True, "Clutch engaged (will re-anchor on next step)"

    def status(self, *, now: float | None = None) -> dict[str, Any]:
        ts = time.monotonic() if now is None else float(now)
        with self._lock:
            pose_age = (ts - self._handle_ts) if self._handle_ts is not None else None
            gripper_age = (ts - self._gripper_ts) if self._gripper_ts is not None else None
            has_pose = pose_age is not None and pose_age <= self.pose_timeout_s
            has_gripper = gripper_age is not None
            return {
                "ok": True,
                "has_pose": has_pose,
                "has_gripper": has_gripper,
                "pose_age_s": round(pose_age, 4) if pose_age is not None else None,
                "gripper_age_s": round(gripper_age, 4) if gripper_age is not None else None,
                "engaged": self._engaged,
                "anchored": self._baseline_matrix is not None,
            }

    def reanchor(
        self,
        joints6: list[float],
        gripper: float,
        *,
        engage: bool = True,
        now: float | None = None,
    ) -> dict[str, Any]:
        ts = time.monotonic() if now is None else float(now)
        q = [float(v) for v in joints6[:6]]
        g = max(0.0, min(float(gripper), self.gripper_max_range))
        with self._lock:
            self._held_joints = list(q)
            self._held_gripper = g
            self._engaged = bool(engage)
            self.ik.sync_state(q)
            pose_fresh = (
                self._handle_matrix is not None
                and self._handle_ts is not None
                and (ts - self._handle_ts) <= self.pose_timeout_s
            )
            if self._engaged and pose_fresh:
                assert self._handle_matrix is not None
                tcp_zero = self.ik.get_pose_mat(q)
                self._baseline_matrix = tcp_zero @ np.linalg.inv(self._handle_matrix)
                self._reanchor_requested = False
            else:
                self._baseline_matrix = None
                self._reanchor_requested = self._engaged
            return {
                "ok": True,
                "engaged": self._engaged,
                "anchored": self._baseline_matrix is not None,
            }

    def disengage(self) -> dict[str, Any]:
        with self._lock:
            self._engaged = False
            self._baseline_matrix = None
            self._reanchor_requested = False
            return {"ok": True, "engaged": False, "anchored": False}

    def step(
        self,
        joints6: list[float],
        gripper: float,
        *,
        now: float | None = None,
    ) -> dict[str, Any]:
        ts = time.monotonic() if now is None else float(now)
        q_robot = [float(v) for v in joints6[:6]]
        g_robot = max(0.0, min(float(gripper), self.gripper_max_range))

        with self._lock:
            if not self._engaged:
                return {
                    "ok": True,
                    "joints": list(self._held_joints),
                    "gripper": float(self._held_gripper),
                    "engaged": False,
                    "anchored": False,
                }

            target_gripper = self._gripper_width if self._gripper_width is not None else g_robot
            self._held_gripper = float(target_gripper)

            pose_fresh = (
                self._handle_matrix is not None
                and self._handle_ts is not None
                and (ts - self._handle_ts) <= self.pose_timeout_s
            )
            if not pose_fresh:
                # Pose lost or not yet seen: hold current pose and force clean re-anchor on recovery.
                if self._baseline_matrix is not None:
                    logger.warning("Pika pose stale (>%.2fs); holding arm and arming recovery re-anchor", self.pose_timeout_s)
                self._baseline_matrix = None
                self._reanchor_requested = True
                return {
                    "ok": True,
                    "joints": list(self._held_joints),
                    "gripper": float(self._held_gripper),
                    "engaged": True,
                    "anchored": False,
                }

            assert self._handle_matrix is not None
            if self._reanchor_requested or self._baseline_matrix is None:
                self.ik.sync_state(q_robot)
                tcp_zero = self.ik.get_pose_mat(q_robot)
                self._baseline_matrix = tcp_zero @ np.linalg.inv(self._handle_matrix)
                self._reanchor_requested = False
                self._held_joints = list(q_robot)
                return {
                    "ok": True,
                    "joints": list(self._held_joints),
                    "gripper": float(self._held_gripper),
                    "engaged": True,
                    "anchored": True,
                }

            target_pose = self._baseline_matrix @ self._handle_matrix
            try:
                sol_q = self.ik.solve(target_pose)
            except Exception as exc:
                logger.warning("IK solve failed (%s); holding previous joint target", exc)
                return {
                    "ok": True,
                    "joints": list(self._held_joints),
                    "gripper": float(self._held_gripper),
                    "engaged": True,
                    "anchored": True,
                }

            prev = np.asarray(self._held_joints, dtype=float)
            delta = np.clip(sol_q - prev, -self.max_joint_step_rad, self.max_joint_step_rad)
            clamped_q = (prev + delta).tolist()
            self._held_joints = [float(v) for v in clamped_q]
            return {
                "ok": True,
                "joints": list(self._held_joints),
                "gripper": float(self._held_gripper),
                "engaged": True,
                "anchored": True,
            }


# ---------------------------------------------------------------------------
# Threaded localhost TCP JSON-RPC Server
# ---------------------------------------------------------------------------


class _BridgeTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address: tuple[str, int], controller: PikaIKController):
        self.controller = controller
        super().__init__(server_address, _BridgeRequestHandler)


class _BridgeRequestHandler(socketserver.StreamRequestHandler):
    def setup(self) -> None:
        self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        super().setup()

    def handle(self) -> None:
        controller: PikaIKController = self.server.controller  # type: ignore[attr-defined]
        for raw_line in self.rfile:
            line = raw_line.strip()
            if not line:
                continue
            try:
                req = json.loads(line.decode("utf-8"))
                reply = _dispatch_rpc(controller, req)
            except Exception as exc:
                logger.exception("RPC dispatch error")
                reply = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
            self.wfile.write((json.dumps(reply, separators=(",", ":")) + "\n").encode("utf-8"))
            self.wfile.flush()


def _dispatch_rpc(controller: PikaIKController, req: dict[str, Any]) -> dict[str, Any]:
    cmd = req.get("cmd")
    if cmd == "ping":
        return controller.status()
    if cmd == "reanchor":
        joints = req.get("joints")
        if not isinstance(joints, list) or len(joints) != 6:
            return {"ok": False, "error": "reanchor requires 6 joint values"}
        gripper = float(req.get("gripper", 0.0))
        engage = bool(req.get("engage", True))
        return controller.reanchor(joints, gripper, engage=engage)
    if cmd == "disengage":
        return controller.disengage()
    if cmd == "step":
        joints = req.get("joints")
        if not isinstance(joints, list) or len(joints) != 6:
            return {"ok": False, "error": "step requires 6 joint values"}
        gripper = float(req.get("gripper", 0.0))
        return controller.step(joints, gripper)
    if cmd == "toggle_clutch":
        engaged, message = controller.toggle_clutch()
        return {"ok": True, "engaged": engaged, "message": message}
    return {"ok": False, "error": f"Unknown command: {cmd!r}"}


class PikaBridgeServer:
    """Lifecycle wrapper around the threaded TCP JSON-RPC server."""

    def __init__(self, controller: PikaIKController, host: str = "127.0.0.1", port: int = 18765) -> None:
        self._server = _BridgeTCPServer((host, port), controller)
        self._thread: threading.Thread | None = None

    @property
    def server_address(self) -> tuple[str, int]:
        addr = self._server.server_address
        return str(addr[0]), int(addr[1])

    def start_in_background(self) -> None:
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="pika-bridge-tcp",
            daemon=True,
        )
        self._thread.start()

    def shutdown(self) -> None:
        self._server.shutdown()
        self._server.server_close()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)


# ---------------------------------------------------------------------------
# ROS 2 Humble node wiring `/pika_pose`, gripper topics & `/teleop_trigger`
# ---------------------------------------------------------------------------


def run_ros2_bridge(args: argparse.Namespace) -> None:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger

    try:
        from data_msgs.msg import Gripper as PikaGripperMsg
    except Exception:
        PikaGripperMsg = None

    logger.info("Initializing Pinocchio + CasADi IK solver for Piper...")
    ik_solver = PiperPinocchioIK(
        urdf_path=args.urdf_path,
        tool_translation_xyz=tuple(args.tool_xyz),  # type: ignore[arg-type]
        w_pos=args.w_pos,
        w_ori=args.w_ori,
        w_reg=args.w_reg,
        w_smooth=args.w_smooth,
        ipopt_max_iter=args.ipopt_max_iter,
        ipopt_tol=args.ipopt_tol,
    )
    logger.info("Loaded Piper URDF: %s (nq=%d)", ik_solver.urdf_path, ik_solver.nq)

    controller = PikaIKController(
        ik_solver,
        handle_pose_rpy=tuple(args.handle_rpy),  # type: ignore[arg-type]
        gripper_max_range=args.gripper_max_range,
        gripper_scale=args.gripper_scale,
        gripper_offset=args.gripper_offset,
        pose_timeout_s=args.pose_timeout_s,
        max_joint_step_rad=args.max_joint_step_rad,
    )

    tcp_server = PikaBridgeServer(controller, host=args.host, port=args.port)
    tcp_server.start_in_background()
    bound_host, bound_port = tcp_server.server_address
    logger.info("LeRobot TCP bridge listening on %s:%d", bound_host, bound_port)

    rclpy.init()

    class PikaRosBridgeNode(Node):
        def __init__(self) -> None:
            super().__init__("pika_lerobot_bridge")
            self.create_subscription(PoseStamped, args.pose_topic, self._on_pose, 10)
            for topic in args.gripper_topics:
                self.create_subscription(JointState, topic, self._on_gripper_joint_state, 10)
            if PikaGripperMsg is not None:
                for topic in args.gripper_data_topics:
                    self.create_subscription(PikaGripperMsg, topic, self._on_gripper_data, 10)
            self.create_service(Trigger, args.trigger_service, self._on_trigger)
            self.get_logger().info(
                f"Subscribed to pose={args.pose_topic}, gripper_joint_topics={args.gripper_topics}, "
                f"clutch_service={args.trigger_service}"
            )

        def _on_pose(self, msg: PoseStamped) -> None:
            p = msg.pose.position
            q = msg.pose.orientation
            try:
                mat = np.eye(4, dtype=float)
                mat[:3, :3] = quat_xyzw_to_mat(q.x, q.y, q.z, q.w)
                mat[:3, 3] = [p.x, p.y, p.z]
                controller.update_handle_pose(mat)
            except ValueError as exc:
                self.get_logger().warning(f"Ignoring invalid Pika pose: {exc}")

        def _on_gripper_joint_state(self, msg: JointState) -> None:
            if msg.position:
                controller.update_gripper(float(msg.position[0]))

        def _on_gripper_data(self, msg: Any) -> None:
            if hasattr(msg, "distance"):
                controller.update_gripper(float(msg.distance))

        def _on_trigger(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
            engaged, message = controller.toggle_clutch()
            self.get_logger().info(f"[Pika button] {message} (engaged={engaged})")
            resp.success = True
            resp.message = message
            return resp

    node = PikaRosBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        tcp_server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="Local TCP bind host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=18765, help="Local TCP bind port (default: 18765)")
    parser.add_argument("--pose-topic", default="/pika_pose", help="Vive tracker PoseStamped topic")
    parser.add_argument(
        "--gripper-topics",
        nargs="+",
        default=["/gripper/joint_state", "/sensor/gripper/joint_state"],
        help="Gripper JointState topics (width in metres)",
    )
    parser.add_argument(
        "--gripper-data-topics",
        nargs="+",
        default=["/gripper/data", "/sensor/gripper/data"],
        help="Fallback data_msgs/Gripper topics",
    )
    parser.add_argument("--trigger-service", default="/teleop_trigger", help="Pika button clutch Trigger service")
    parser.add_argument("--urdf-path", default=None, help="Explicit path to piper_with_gripper_description.urdf")
    parser.add_argument(
        "--handle-rpy",
        type=float,
        nargs=3,
        default=[0.0, 1.57, 0.0],
        metavar=("ROLL", "PITCH", "YAW"),
        help="Pika handle-to-TCP orientation offset in radians (default: 0.0 1.57 0.0)",
    )
    parser.add_argument(
        "--tool-xyz",
        type=float,
        nargs=3,
        default=[0.0, 0.0, 0.13],
        metavar=("X", "Y", "Z"),
        help="Joint6-to-TCP translation offset in metres (default: 0.0 0.0 0.13)",
    )
    parser.add_argument("--gripper-max-range", type=float, default=0.07, help="Max Piper gripper width in m")
    parser.add_argument("--gripper-scale", type=float, default=1.0, help="Scale factor from Pika gripper width")
    parser.add_argument("--gripper-offset", type=float, default=0.0, help="Additive offset on gripper width in m")
    parser.add_argument("--pose-timeout-s", type=float, default=0.5, help="Hold arm if pose older than this (s)")
    parser.add_argument("--max-joint-step-rad", type=float, default=0.35, help="Max per-tick joint delta (rad)")
    parser.add_argument("--w-pos", type=float, default=10.0)
    parser.add_argument("--w-ori", type=float, default=2.0)
    parser.add_argument("--w-reg", type=float, default=0.01)
    parser.add_argument("--w-smooth", type=float, default=0.1)
    parser.add_argument("--ipopt-max-iter", type=int, default=50)
    parser.add_argument("--ipopt-tol", type=float, default=1e-4)
    return parser


def main() -> None:
    logging.basicConfig(
        level=os.environ.get("LOGLEVEL", "INFO").upper(),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    args = build_arg_parser().parse_args()
    run_ros2_bridge(args)


if __name__ == "__main__":
    main()
