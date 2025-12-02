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
        # For 3DOF, solve IK properly distributing angles between joints 2 and 3
        if len(self.link_lengths) == 2:
            return self.inverse_kinematics_2d(target_x, target_y, elbow_up)
        elif len(self.link_lengths) >= 3:
            l1 = self.link_lengths[0]  # 0.5m
            l2 = self.link_lengths[1]  # 0.5m
            l3 = self.link_lengths[2]  # 0.4m
            
            r = math.hypot(target_x, target_y)
            
            # First, try to reach with links 1 and 2 only (ignoring link 3 direction)
            # We want the end of link2 to be at distance l3 from target
            # This puts the wrist joint in position to bend toward target
            
            # Calculate where link2 should end (wrist joint position)
            # Wrist should be at distance l3 from target, along the line from base to target
            target_angle = math.atan2(target_y, target_x)
            
            # Wrist position: back from target by l3 distance
            wrist_x = target_x - l3 * math.cos(target_angle)
            wrist_y = target_y - l3 * math.sin(target_angle)
            
            wrist_r = math.hypot(wrist_x, wrist_y)
            
            # Check if wrist position is reachable with links 1 and 2
            if wrist_r > l1 + l2 or wrist_r < abs(l1 - l2):
                # Fallback: use combined approach
                l2_combined = l2 + l3
                if r > l1 + l2_combined or r < abs(l1 - l2_combined):
                    return None
                # Solve with combined length
                cos_phi = (r**2 - l1**2 - l2_combined**2) / (2 * l1 * l2_combined)
                cos_phi = max(-1.0, min(1.0, cos_phi))
                sin_phi = math.sqrt(max(0.0, 1 - cos_phi**2))
                if not elbow_up:
                    sin_phi = -sin_phi
                theta2 = math.atan2(sin_phi, cos_phi)
                k1 = l1 + l2_combined * cos_phi
                k2 = l2_combined * sin_phi
                theta1 = math.atan2(target_y, target_x) - math.atan2(k2, k1)
                return [theta1, theta2, 0.0]
            
            # Solve 2-link IK for wrist position using links 1 and 2
            cos_phi = (wrist_r**2 - l1**2 - l2**2) / (2 * l1 * l2)
            cos_phi = max(-1.0, min(1.0, cos_phi))
            sin_phi = math.sqrt(max(0.0, 1 - cos_phi**2))
            if not elbow_up:
                sin_phi = -sin_phi
            theta2 = math.atan2(sin_phi, cos_phi)
            
            k1 = l1 + l2 * cos_phi
            k2 = l2 * sin_phi
            theta1 = math.atan2(wrist_y, wrist_x) - math.atan2(k2, k1)
            
            # Calculate theta3: angle from link2 direction to target direction
            # Link2 ends pointing at angle theta1 + theta2
            link2_angle = theta1 + theta2
            # We want link3 to point toward target
            theta3 = target_angle - link2_angle
            
            return [theta1, theta2, theta3]
        else:
            return None
