"""Range sensing and frame transforms. Only sensor simulation reads the true sensor pose."""
from dataclasses import dataclass
import copy
import json
from pathlib import Path
import mujoco
import numpy as np

T_BL_TRANSLATION = np.array([-0.032, 0.0, 0.182])


@dataclass
class Scan:
    time: float
    angles: np.ndarray
    ranges: np.ndarray  # Finite value: return; inf: no return within range; nan: invalid or missing.
    max_range: float

# Convert scan readings into points in the body frame.
def points_body(scan):
    valid = np.isfinite(scan.ranges)
    r, a = scan.ranges[valid], scan.angles[valid]
    local = np.column_stack((r*np.cos(a), r*np.sin(a), np.zeros_like(r)))
    return local + T_BL_TRANSLATION


def points_world_from_estimate(points_b, estimated_pose):
    """The caller supplies estimated [x,y,yaw]; this function does not access ground truth."""
    x, y, theta = estimated_pose
    c, s = np.cos(theta), np.sin(theta)
    return points_b[:, :2] @ np.array([[c, s], [-s, c]]) + [x, y]


# Simulate laser ranges with noise and missing returns.
class SimLidar:
    def __init__(self, model, sigma=.01, dropout=.01, seed=7, max_range=6.0, calibrated=True):
        if sigma < 0 or not 0 <= dropout <= 1 or max_range <= .08:
            raise ValueError("Invalid lidar parameters")
        self.model = model
        self.scale,self.offset=1.,0.
        if calibrated:
            path=Path(__file__).resolve().parents[2]/'calibration/lidar.json'
            config=json.loads(path.read_text(encoding='utf-8'))
            self.scale,self.offset=float(config['scale']),float(config['offset_m'])
            if not np.isfinite([self.scale,self.offset]).all() or self.scale<=0:
                raise ValueError('Invalid LiDAR calibration')
        # Ray filtering uses a separate model copy and leaves render/collision display groups unchanged.
        self.ray_model = copy.copy(model)
        self.robot_geoms = np.zeros(model.ngeom, dtype=bool)
        self.sigma, self.dropout, self.max_range = sigma, dropout, max_range
        self.rng = np.random.default_rng(seed)
        self.angles = np.deg2rad(np.arange(-180, 180, dtype=float))
        self.directions = np.column_stack((np.cos(self.angles), np.sin(self.angles), np.zeros(360)))
        self.site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "lidar_frame")
        robot_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot")
        # Exclude the full robot subtree, including wheels; bodyexclude alone does not exclude child bodies.
        for gid, bid in enumerate(model.geom_bodyid):
            ancestor = int(bid)
            while ancestor and ancestor != robot_id:
                ancestor = int(model.body_parentid[ancestor])
            if ancestor == robot_id:
                self.robot_geoms[gid] = True
                self.ray_model.geom_group[gid] = 5
        self.groups = np.array([1, 1, 1, 1, 1, 0], dtype=np.uint8)

    def read(self, data):
        mujoco.mj_forward(self.model, data)
        origin = data.site_xpos[self.site]
        rotation = data.site_xmat[self.site].reshape(3, 3)
        world_dirs = self.directions @ rotation.T
        ranges = np.full(len(self.angles), np.inf)
        geom = np.empty(1, dtype=np.int32)
        for i, direction in enumerate(world_dirs):
            distance = mujoco.mj_ray(self.ray_model, data, origin, direction,
                                     self.groups, 1, -1, geom)
            if 0 <= distance < .08:
                ranges[i] = np.nan
            elif .08 <= distance <= self.max_range:
                ranges[i] = np.clip(distance + self.rng.normal(0, self.sigma), .08, self.max_range)
        ranges[self.rng.random(len(ranges)) < self.dropout] = np.nan
        valid=np.isfinite(ranges)
        ranges[valid]=np.clip((ranges[valid]-self.offset)/self.scale,.08,self.max_range)
        return Scan(float(data.time), self.angles.copy(), ranges, self.max_range)


def front_status(scan):
    sector = np.abs(scan.angles) <= np.deg2rad(20)
    r = scan.ranges[sector]
    good = ~np.isnan(r)  # inf is a valid reading meaning no object was detected within range.
    if good.mean() < .8:
        return "INVALID SCAN", 0.0
    # Check for a central blind gap even when the overall sector has enough valid readings.
    middle = scan.ranges[np.abs(scan.angles) <= np.deg2rad(3)]
    if np.isnan(middle).all():
        return "INVALID SCAN", 0.0
    nearest = float(np.min(r[good]))
    return ("OBSTACLE STOP" if nearest < .45 else "CLEAR"), nearest
