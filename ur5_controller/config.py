"""UR5 配置：机器人 IP、速度/加速度、安全参数，以及 RTDE 状态枚举。

设计要点：
- 开发期默认速度**远低于** ur_rtde 的内置默认值（关节 v=1.05 rad/s, a=1.4 rad/s^2），
  真实硬件上更安全。
- 支持用环境变量覆盖每一项，方便部署；示例脚本也可用 ``--host``/``--dry-run`` 临时覆盖。
- ``from_env`` / ``from_overrides`` 让示例脚本零样板即可拿到配置。
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field

# ──────────────────────────────────────────────────────────────────────────
#  RTDE 枚举（按 Universal Robots RTDE 规范固定值，多年稳定）
# ──────────────────────────────────────────────────────────────────────────

# 机器人模式：getRobotMode() 返回值
ROBOT_MODE_DISCONNECTED = 0
ROBOT_MODE_CONFIRM_SAFETY = 1
ROBOT_MODE_BOOTING = 2
ROBOT_MODE_POWER_OFF = 3
ROBOT_MODE_POWER_ON = 4
ROBOT_MODE_IDLE = 5
ROBOT_MODE_BACKDRIVE = 6
ROBOT_MODE_RUNNING = 7

ROBOT_MODE_NAMES = {
    0: "DISCONNECTED",
    1: "CONFIRM_SAFETY",
    2: "BOOTING",
    3: "POWER_OFF",
    4: "POWER_ON",
    5: "IDLE",
    6: "BACKDRIVE",
    7: "RUNNING",
}

# 安全模式：getSafetyMode() 返回值
SAFETY_MODE_NORMAL = 0
SAFETY_MODE_REDUCED = 1
SAFETY_MODE_PROTECTIVE_STOP = 2
SAFETY_MODE_RECOVERY = 3
SAFETY_MODE_SAFEGUARD_STOP = 4
SAFETY_MODE_SYSTEM_EMERGENCY_STOP = 5
SAFETY_MODE_ROBOT_EMERGENCY_STOP = 6
SAFETY_MODE_VIOLATION = 7
SAFETY_MODE_FAULT = 8

SAFETY_MODE_NAMES = {
    0: "NORMAL",
    1: "REDUCED",
    2: "PROTECTIVE_STOP",
    3: "RECOVERY",
    4: "SAFEGUARD_STOP",
    5: "SYSTEM_EMERGENCY_STOP",
    6: "ROBOT_EMERGENCY_STOP",
    7: "VIOLATION",
    8: "FAULT",
}


def robot_mode_name(mode: int) -> str:
    """把 ``getRobotMode()`` 的整数值翻译成可读名字。"""
    return ROBOT_MODE_NAMES.get(mode, f"UNKNOWN({mode})")


def safety_mode_name(mode: int) -> str:
    """把 ``getSafetyMode()`` 的整数值翻译成可读名字。"""
    return SAFETY_MODE_NAMES.get(mode, f"UNKNOWN({mode})")


# ──────────────────────────────────────────────────────────────────────────
#  未配置 IP 的占位符：示例脚本看到它就会拒绝运行，避免误连陌生设备
# ──────────────────────────────────────────────────────────────────────────
PLACEHOLDER_IP = "192.168.xxx.xxx"


def _env_bool(name: str, default: bool = False) -> bool:
    v = os.environ.get(name)
    if v is None:
        return default
    return v.strip().lower() in ("1", "true", "yes", "on")


def _env_float(name: str, default: float) -> float:
    v = os.environ.get(name)
    return float(v) if v else default


def _env_float_list(name: str, default: list[float]) -> list[float]:
    v = os.environ.get(name)
    if not v:
        return list(default)
    return [float(x) for x in v.split(",")]


@dataclass
class RobotConfig:
    """UR5 控制配置。

    Attributes:
        host: 机器人 IP（必填）。留为占位符时 ``assert_host_configured`` 会抛错。
        joint_velocity: moveJ 默认关节速度 [rad/s]。
        joint_acceleration: moveJ 默认关节加速度 [rad/s^2]。
        linear_velocity: moveL 默认工具速度 [m/s]。
        linear_acceleration: moveL 默认工具加速度 [m/s^2]。
        speed_joint_accel: speedJ 实时控制加速度 [rad/s^2]。
        speed_linear_accel: speedL 实时控制加速度 [m/s^2]。
        tcp_offset: 工具坐标系偏移 [x,y,z,rx,ry,rz]，零向量表示不设置。
        payload_mass: 末端负载质量 [kg]，0 表示不设置。
        payload_cog: 末端负载质心 [x,y,z] [m]。
        home_joints: ``home()`` 的目标关节角 [rad]。
        dry_run: True 时只读状态/打印计划动作，**不真正下发运动指令**。
        check_safety_limits: 运动前用 isPoseWithinSafetyLimits / isJointsWithinSafetyLimits 校验。
        require_running: 运动前要求 robot_mode == RUNNING，否则报错。
    """

    host: str = PLACEHOLDER_IP

    # 关节运动 moveJ（开发期保守值，远低于 ur_rtde 内置默认）
    joint_velocity: float = 0.5        # rad/s
    joint_acceleration: float = 0.3    # rad/s^2

    # 笛卡尔直线运动 moveL
    linear_velocity: float = 0.10      # m/s
    linear_acceleration: float = 0.10  # m/s^2

    # 实时速度控制 speedJ / speedL 的加速度
    speed_joint_accel: float = 0.3     # rad/s^2
    speed_linear_accel: float = 0.15   # m/s^2

    # 工具 / 负载（默认零：不主动设置，沿用示教器上的配置）
    tcp_offset: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    payload_mass: float = 0.0
    payload_cog: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    # 归零位姿（关节角 rad）：一个安全的“竖直指向上方”姿态，按需改
    home_joints: list[float] = field(
        default_factory=lambda: [0.0, -1.5707963267948966, 0.0, -1.5707963267948966, 0.0, 0.0]
    )

    # 安全开关
    dry_run: bool = False
    check_safety_limits: bool = True
    require_running: bool = True

    @classmethod
    def from_env(cls) -> "RobotConfig":
        """从环境变量构造配置，便于不改代码即调整。

        支持的环境变量::

            UR5_HOST                 机器人 IP（必填，否则仍为占位符）
            UR5_DRY_RUN              1/true 时启用 dry-run
            UR5_JOINT_VEL            moveJ 速度 rad/s
            UR5_JOINT_ACC            moveJ 加速度 rad/s^2
            UR5_LINEAR_VEL           moveL 速度 m/s
            UR5_LINEAR_ACC           moveL 加速度 m/s^2
            UR5_SPEED_JOINT_ACC      speedJ 加速度 rad/s^2
            UR5_SPEED_LINEAR_ACC     speedL 加速度 m/s^2
            UR5_TCP                  TCP 偏移，逗号分隔 6 个数
            UR5_PAYLOAD_MASS         负载质量 kg
            UR5_PAYLOAD_COG          负载质心，逗号分隔 3 个数
            UR5_HOME                 归零关节角，逗号分隔 6 个数
        """
        cfg = cls(
            host=os.environ.get("UR5_HOST", PLACEHOLDER_IP),
            joint_velocity=_env_float("UR5_JOINT_VEL", cls.joint_velocity),
            joint_acceleration=_env_float("UR5_JOINT_ACC", cls.joint_acceleration),
            linear_velocity=_env_float("UR5_LINEAR_VEL", cls.linear_velocity),
            linear_acceleration=_env_float("UR5_LINEAR_ACC", cls.linear_acceleration),
            speed_joint_accel=_env_float("UR5_SPEED_JOINT_ACC", cls.speed_joint_accel),
            speed_linear_accel=_env_float("UR5_SPEED_LINEAR_ACC", cls.speed_linear_accel),
            tcp_offset=_env_float_list("UR5_TCP", cls.tcp_offset),
            payload_mass=_env_float("UR5_PAYLOAD_MASS", cls.payload_mass),
            payload_cog=_env_float_list("UR5_PAYLOAD_COG", cls.payload_cog),
            home_joints=_env_float_list("UR5_HOME", cls.home_joints),
            dry_run=_env_bool("UR5_DRY_RUN", cls.dry_run),
        )
        return cfg

    @classmethod
    def from_overrides(cls, host: str | None = None, dry_run: bool | None = None) -> "RobotConfig":
        """先取环境变量，再用显式参数覆盖。示例脚本常用。"""
        cfg = cls.from_env()
        if host is not None:
            cfg.host = host
        if dry_run is not None:
            cfg.dry_run = dry_run
        return cfg

    def assert_host_configured(self) -> None:
        """若 IP 仍是占位符，抛出带清晰指引的错误。"""
        if not self.host or "xxx" in self.host or self.host == PLACEHOLDER_IP:
            raise RuntimeError(
                "未配置机器人 IP。请：\n"
                "  1) 设置环境变量  export UR5_HOST=192.168.1.102\n"
                "  2) 或在示例脚本加 --host 192.168.1.102\n"
                "  3) 或在代码里  cfg.host = '192.168.1.102'"
            )
