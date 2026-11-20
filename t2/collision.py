import csv
from math import isfinite
from pathlib import Path
from time import monotonic

import mujoco
import numpy as np


JOINT_NAMES = tuple(
    f'{leg}_{joint}_joint'
    for leg in ('FR', 'FL', 'RR', 'RL')
    for joint in ('hip', 'thigh', 'calf')
)
MIN_DISTANCE = -0.001 + 0.00002
MAX_INSPECTIONS = 63
MAX_DEPTH = 8
CHECK_TIME_BUDGET = 0.006


class CollisionGuard:
    """Certify linear joint-space transitions on the supplied Go1 model."""

    def __init__(self, model_dir):
        model_dir = Path(model_dir)
        self.model = model = mujoco.MjModel.from_xml_path(str(model_dir / 'scene.xml'))
        self.data = mujoco.MjData(model)
        joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINT_NAMES]
        hinges = set(np.flatnonzero(model.jnt_type == mujoco.mjtJoint.mjJNT_HINGE))
        if len(hinges) != 12 or hinges != set(joint_ids) or np.any(model.jnt_pos[joint_ids]):
            raise ValueError('Unexpected joint layout in the Go1 model')
        self.addresses = model.jnt_qposadr[joint_ids]
        self.limits = model.jnt_range[joint_ids].copy()
        home = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'home')
        if home < 0:
            raise ValueError('Missing home keyframe')
        mujoco.mj_resetDataKeyframe(model, self.data, home)
        self.home = self.data.qpos[self.addresses].copy()

        pairs = self._read_pairs(model_dir / 't2_collision_pairs.csv')
        self.first, self.second = pairs.T
        self.pairs = [(int(first), int(second)) for first, second in pairs]
        self.radius_sum = model.geom_rbound[self.first] + model.geom_rbound[self.second]
        reach = self._joint_reach(joint_ids)
        first_reach, second_reach = reach[self.first], reach[self.second]
        # A shared ancestor moves both shapes together and has no effect on their distance.
        self.relative_reach = np.where(
            (first_reach > 0.0) & (second_reach > 0.0), 0.0, first_reach + second_reach
        )

    def _read_pairs(self, path):
        model = self.model
        with path.open(newline='', encoding='utf-8') as stream:
            rows = list(csv.DictReader(stream))
        pairs = []
        primitive_types = {
            mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CAPSULE,
            mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_SPHERE,
        }
        for row in rows:
            if row['diagnostic_policy'] != 'checked':
                continue
            pair = (int(row['geom1_id']), int(row['geom2_id']))
            for index, geom in enumerate(pair, 1):
                if not (0 <= geom < model.ngeom) or model.geom_type[geom] not in primitive_types:
                    raise ValueError(f'Unexpected collision geometry {geom}')
                body = int(model.geom_bodyid[geom])
                if (body != int(row[f'body{index}_id'])
                        or mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) != row[f'body{index}']):
                    raise ValueError(f'Collision table does not match geometry {geom}')
            pairs.append(pair)
        if len(rows) != 1540 or len(pairs) != 655 or len(set(pairs)) != 655:
            raise ValueError('Incomplete Go1 collision table')
        return np.asarray(pairs, dtype=np.intp)

    def _joint_reach(self, joint_ids):
        """Bound every shape point's distance from each of its ancestor hinge axes."""
        model = self.model
        columns = {joint: column for column, joint in enumerate(joint_ids)}
        reach = np.zeros((model.ngeom, len(joint_ids)))
        offset_lengths = np.linalg.norm(model.body_pos, axis=1)
        for geom in set(self.first) | set(self.second):
            body = int(model.geom_bodyid[geom])
            radius = float(model.geom_rbound[geom] + np.linalg.norm(model.geom_pos[geom]))
            while body:
                joint = int(model.body_jntadr[body])
                if joint in columns:
                    reach[geom, columns[joint]] = radius
                radius += offset_lengths[body]
                body = int(model.body_parentid[body])
        return reach

    def _inspect(self, angles, motion):
        """Return whether the midpoint is safe and whether it certifies the interval."""
        self.data.qpos[self.addresses] = angles
        mujoco.mj_kinematics(self.model, self.data)
        xyz = self.data.geom_xpos
        separation = xyz[self.first] - xyz[self.second]
        squared_distance = np.einsum('ij,ij->i', separation, separation)
        bound = self.radius_sum + motion + MIN_DISTANCE
        nearby = np.flatnonzero(squared_distance < np.maximum(bound, 0.0) ** 2)
        certified = True
        for index in nearby:
            first, second = self.pairs[index]
            distance = mujoco.mj_geomDistance(
                self.model, self.data, first, second, float(motion[index] + 0.002), None
            )
            if not isfinite(distance) or distance < MIN_DISTANCE:
                return False, False
            if distance < MIN_DISTANCE + motion[index]:
                certified = False
        return True, certified

    def pose_is_safe(self, angles):
        return self._inspect(angles, np.zeros(len(self.pairs)))[0]

    def transition_is_safe(self, before, after):
        delta = after - before
        if not np.any(delta):
            return True  # Every stored starting pose has already been certified.
        total_motion = self.relative_reach @ np.abs(delta)
        # A point at most r from a hinge moves at most r * abs(delta_angle).
        # The sum of both shapes' bounds controls the change in signed distance.
        intervals = [(0.0, 1.0, 0)]
        deadline = monotonic() + CHECK_TIME_BUDGET
        for _ in range(MAX_INSPECTIONS):
            lo, hi, depth = intervals.pop()
            middle = (lo + hi) / 2.0
            safe, certified = self._inspect(before + middle * delta, total_motion * (hi - lo) / 2.0)
            if not safe:
                return False
            if not certified:
                if depth == MAX_DEPTH:
                    return False
                intervals.extend(((middle, hi, depth + 1), (lo, middle, depth + 1)))
            if not intervals:
                return True
            if monotonic() >= deadline:
                return False
        return False
