import math

class PlanarArmKinematics:
    def __init__(self, link_lengths):
        """
        Cinemática direta para braço planar 2 ou 3 DOF.
        link_lengths: lista de comprimentos dos links [l1, l2, ...]
        """
        self.link_lengths = link_lengths

    def forward_kinematics(self, joint_angles):
        """
        Calcula a posição do efetuador final dado os ângulos das juntas.
        joint_angles: lista de ângulos [theta1, theta2, ...] em radianos.
        Retorna: (x, y) para 2D planar.
        """
        x = 0.0
        y = 0.0
        theta_cum = 0.0
        for i, theta in enumerate(joint_angles):
            theta_cum += theta
            x += self.link_lengths[i] * math.cos(theta_cum)
            y += self.link_lengths[i] * math.sin(theta_cum)
        return x, y

    def inverse_kinematics(self, target_x, target_y, elbow_up=True):
        """
        Cinemática inversa simples para 2 DOF.
        Para 3 DOF, retorna [theta1, theta2, 0]
        Retorna lista de ângulos ou None se impossível.
        """
        if len(self.link_lengths) == 2:
            l1, l2 = self.link_lengths
        elif len(self.link_lengths) == 3:
            l1, l2 = self.link_lengths[0], self.link_lengths[1] + self.link_lengths[2]  # approximate
        else:
            raise NotImplementedError("IK só implementado para 2 ou 3 DOF.")

        dist = math.sqrt(target_x**2 + target_y**2)
        if dist > l1 + l2 or dist < abs(l1 - l2):
            return None  # impossível

        cos_theta2 = (target_x**2 + target_y**2 - l1**2 - l2**2) / (2 * l1 * l2)
        sin_theta2 = math.sqrt(1 - cos_theta2**2) if elbow_up else -math.sqrt(1 - cos_theta2**2)
        theta2 = math.atan2(sin_theta2, cos_theta2)

        k1 = l1 + l2 * cos_theta2
        k2 = l2 * sin_theta2
        theta1 = math.atan2(target_y, target_x) - math.atan2(k2, k1)

        if len(self.link_lengths) == 3:
            return [theta1, theta2, 0.0]  # third joint fixed
        else:
            return [theta1, theta2]