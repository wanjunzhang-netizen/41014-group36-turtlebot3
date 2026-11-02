"""Robot physics loader for the random room."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
WHEEL_RADIUS = 0.033
TRACK = 0.16
def load_scene(scene_transform):
    scene = ET.fromstring((ROOT / "models/room_base.xml").read_text(encoding="utf-8-sig"))
    robot = ET.fromstring((ROOT / "models/02_robot.xml").read_text(encoding="utf-8-sig"))
    scene.append(robot.find("asset"))
    body = robot.find("worldbody/body")
    body.set("pos", "0 0 0.002")
    scene.find("worldbody").append(body)
    scene.append(robot.find("actuator"))
    scene.append(robot.find("sensor"))
    scene.find("option").set("timestep", "0.002")
    if scene_transform is not None:
        scene_transform(scene)
    assets = {p.name: p.read_bytes() for p in (ROOT / "models/turtlebot3_official").rglob("*.stl")}
    model = mujoco.MjModel.from_xml_string(ET.tostring(scene, encoding="unicode"), assets)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def drive(data, v, omega):
    """只允许车体前进；原地转向时两轮可以反向转动。"""
    if not 0 <= v <= 0.15 or abs(omega) > 0.8:
        raise ValueError("Commands must satisfy 0 <= v <= 0.15 and |omega| <= 0.8")
    # LQR 不直接控制图片里的小车位置，而是先产生速度指令，再驱动车轮
    data.ctrl[:] = [(v - TRACK * omega / 2) / WHEEL_RADIUS,
                    (v + TRACK * omega / 2) / WHEEL_RADIUS]


def advance(model, data, seconds, v=0, omega=0):
    drive(data, v, omega)
    for _ in range(round(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)
    if not np.isfinite(data.qpos).all():
        raise RuntimeError("Non-finite physics state")


def yaw(data):
    w, x, y, z = data.qpos[3:7]
    return math.atan2(2 * (w*z + x*y), 1 - 2*(y*y + z*z))


