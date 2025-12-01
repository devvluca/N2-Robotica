import pybullet as p
import math
import time

class PlanarArm:
    def __init__(self, urdf_path, base_pos, base_ori, link_lengths, num_dofs=2, ee_link_name='ee'):
        self.urdf_path = urdf_path
        self.base_pos = base_pos
        self.base_ori = p.getQuaternionFromEuler(base_ori)
        self.link_lengths = link_lengths
        self.num_dofs = num_dofs
        self.ee_link_name = ee_link_name
        self.dt = 1.0 / 240.0

        # PID gains
        self.Kp = [4.0] * num_dofs
        self.Ki = [0.02] * num_dofs
        self.Kd = [1.0] * num_dofs
        self.pid_int = [0.0] * num_dofs
        self.pid_prev = [0.0] * num_dofs
        self.max_velocity = 2.0

        # Metrics
        self.err_log = []
        self.energy_log = []
        self.max_error = 0.0

        # internal
        self.id = None
        self.arm_joints = []
        self.eef_link_index = None

    def load(self):
        self.id = p.loadURDF(self.urdf_path, self.base_pos, self.base_ori, useFixedBase=True)
        self.arm_joints = []
        for i in range(p.getNumJoints(self.id)):
            info = p.getJointInfo(self.id, i)
            jtype = info[2]
            if jtype != p.JOINT_FIXED:
                self.arm_joints.append(info[0])

        assert len(self.arm_joints) >= self.num_dofs, f"Not enough joints in URDF: {len(self.arm_joints)} found, {self.num_dofs} expected"
        self.arm_joints = self.arm_joints[:self.num_dofs]

        # find eef link
        self.eef_link_index = None
        for i in range(p.getNumJoints(self.id)):
            info = p.getJointInfo(self.id, i)
            child_name = info[12].decode('utf-8')
            if child_name == self.ee_link_name:
                self.eef_link_index = i
                break
        if self.eef_link_index is None:
            self.eef_link_index = p.getNumJoints(self.id) - 1

        # initialize joints and set to velocity control
        for j in self.arm_joints:
            p.resetJointState(self.id, j, 0.0)
            p.setJointMotorControl2(self.id, j, p.VELOCITY_CONTROL, targetVelocity=0.0, force=100)

    def get_joint_angles(self):
        angles = [p.getJointState(self.id, j)[0] for j in self.arm_joints]
        return angles

    def get_ee_position(self):
        # Use FK calculation (fallback) always for correct planar coordinates
        angles = self.get_joint_angles()
        x = 0.0
        y = 0.0
        theta = 0.0
        for i, a in enumerate(angles):
            theta += a
            x += self.link_lengths[i] * math.cos(theta)
            y += self.link_lengths[i] * math.sin(theta)
        return [x, y, self.base_pos[2]], [0, 0, 0, 1]

    def pid_step(self, target_angles):
        work = 0.0
        for i, j in enumerate(self.arm_joints):
            js = p.getJointState(self.id, j)
            q = js[0] if js is not None else 0.0
            dq = js[1] if js is not None else 0.0
            e = target_angles[i] - q
            self.pid_int[i] += e * self.dt
            d = (e - self.pid_prev[i]) / self.dt
            cmd = self.Kp[i] * e + self.Ki[i] * self.pid_int[i] + self.Kd[i] * d
            cmd = max(-self.max_velocity, min(self.max_velocity, cmd))
            self.pid_prev[i] = e
            p.setJointMotorControl2(self.id, j, p.VELOCITY_CONTROL, targetVelocity=cmd, force=100)
            work += abs(cmd * dq * self.dt)
        self.energy_log.append(work)
        err_sq = 0.0
        for i, j in enumerate(self.arm_joints):
            q = p.getJointState(self.id, j)[0]
            err = target_angles[i] - q
            err_sq += err*err
            self.max_error = max(self.max_error, abs(err))
        self.err_log.append(math.sqrt(err_sq / self.num_dofs))

    def ramped_move(self, target_angles, duration=1.0, steps_per_sec=240):
        steps = max(2, int(duration * steps_per_sec))
        current = self.get_joint_angles()
        if len(current) != len(target_angles):
            current = [0.0] * len(target_angles)
        for s in range(1, steps+1):
            alpha = s/steps
            intermediate = [(1-alpha)*c + alpha*t for c, t in zip(current, target_angles)]
            self.pid_step(intermediate)
            p.stepSimulation()
            time.sleep(1.0/steps_per_sec)

    def attach(self, body_id, child_link_index=-1):
        try:
            cid = p.createConstraint(self.id, self.eef_link_index, body_id, child_link_index, p.JOINT_FIXED, [0,0,0], [0,0,0], [0,0,0])
            return cid
        except Exception:
            return None

    def detach(self, cid):
        try:
            p.removeConstraint(cid)
        except Exception:
            pass