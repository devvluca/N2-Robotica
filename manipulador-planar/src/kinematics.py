import math

class PlanarArmKinematics:
    def __init__(self, link_lengths):
        self.link_lengths = link_lengths

    def forward_kinematics(self, joint_angles):
        x = 0.0
        y = 0.0
        theta_cum = 0.0
        for i, theta in enumerate(joint_angles):
            theta_cum += theta
            x += self.link_lengths[i] * math.cos(theta_cum)
            y += self.link_lengths[i] * math.sin(theta_cum)
        return x, y

    def inverse_kinematics_2d(self, target_x, target_y, elbow_up=True):
        # Analytical IK for 2DOF planar arm
        if len(self.link_lengths) < 2:
            raise ValueError('Requires at least 2 link lengths')
        l1 = self.link_lengths[0]
        l2 = self.link_lengths[1]
        r = math.hypot(target_x, target_y)
        if r > l1 + l2 or r < abs(l1 - l2):
            return None
        cos_phi = (r**2 - l1**2 - l2**2) / (2 * l1 * l2)
        cos_phi = max(-1.0, min(1.0, cos_phi))
        sin_phi = math.sqrt(max(0.0, 1 - cos_phi**2))
        if not elbow_up:
            sin_phi = -sin_phi
        theta2 = math.atan2(sin_phi, cos_phi)
        k1 = l1 + l2 * cos_phi
        k2 = l2 * sin_phi
        theta1 = math.atan2(target_y, target_x) - math.atan2(k2, k1)
        return [theta1, theta2]

    def inverse_kinematics(self, target_x, target_y, elbow_up=True):
        # For 3DOF, use combined length of links 2..n to increase reachable area,
        # but return a wrist angle of 0. This keeps IK analytic and simple.
        if len(self.link_lengths) == 2:
            return self.inverse_kinematics_2d(target_x, target_y, elbow_up)
        elif len(self.link_lengths) >= 3:
            # combine the last n-1 link lengths
            l1 = self.link_lengths[0]
            l2 = sum(self.link_lengths[1:])
            r = math.hypot(target_x, target_y)
            if r > l1 + l2 or r < abs(l1 - l2):
                return None
            # use the 2-link formula with l1 and l2 combined
            # reuse inverse_kinematics_2d by temporarily creating a helper
            original = self.link_lengths
            try:
                self.link_lengths = [l1, l2]
                ik2 = self.inverse_kinematics_2d(target_x, target_y, elbow_up)
            finally:
                self.link_lengths = original
            if ik2 is None:
                return None
            # append a wrist angle of 0
            return [ik2[0], ik2[1], 0.0]
        else:
            return None
