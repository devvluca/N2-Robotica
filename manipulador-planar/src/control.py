import pybullet as p
import math

from src.kinematics import PlanarArmKinematics

class ArmController:
    def __init__(self, arm, kinematics: PlanarArmKinematics, lift_height=0.25):
        self.arm = arm
        self.kin = kinematics
        self.lift_height = lift_height
        self.grasp_constraints = {}

    def move_to_xy(self, target_x, target_y, duration=1.0):
        ik = self.kin.inverse_kinematics(target_x, target_y)
        if ik is None:
            print(f'[CONTROLADOR] Posição IK impossível: ({target_x:.2f}, {target_y:.2f})')
            return False
        if len(ik) > self.arm.num_dofs:
            ik = ik[:self.arm.num_dofs]
        print(f'[CONTROLADOR] Movendo para posição: ({target_x:.2f}, {target_y:.2f}) com ângulos: {[f"{a:.2f}" for a in ik]}')
        self.arm.ramped_move(ik, duration)
        ee_pos, _ = self.arm.get_ee_position()
        print(f'[CONTROLADOR] Posição final do efetuador: x={ee_pos[0]:.2f}, y={ee_pos[1]:.2f}, z={ee_pos[2]:.2f}')
        return True

    def check_obstacle_between(self, target_x, target_y):
        ee_pos, _ = self.arm.get_ee_position()
        start = ee_pos
        end = [target_x, target_y, ee_pos[2]]
        hits = p.rayTest(start, end)
        if hits and hits[0][0] >= 0 and hits[0][2] < 0.999:
            return hits[0]
        return None

    def avoid_and_move(self, target_x, target_y, duration=1.0):
        if self.check_obstacle_between(target_x, target_y):
            print(f'[CONTROLADOR] Obstáculo detectado! Desviando para ({target_x:.2f}, {target_y:.2f})')
            ee_pos, _ = self.arm.get_ee_position()
            # Lift up by moving X,Y slightly while maintaining reach
            mid_x = (ee_pos[0] + target_x) / 2.0
            mid_y = (ee_pos[1] + target_y) / 2.0
            
            # clamp targets to reachable workspace
            reach = sum(self.kin.link_lengths)
            def clamp(x, y):
                r = (x**2 + y**2)**0.5
                margin = 0.01
                if r > (reach - margin):
                    scale = (reach - margin) / r
                    return x*scale, y*scale
                return x, y
            
            mx, my = clamp(mid_x, mid_y)
            self.move_to_xy(mx, my, duration*0.5)
            tx, ty = clamp(target_x, target_y)
            return self.move_to_xy(tx, ty, duration*0.5)
        else:
            return self.move_to_xy(target_x, target_y, duration)

    def grasp(self, cube_id):
        ee_pos, _ = self.arm.get_ee_position()
        print(f'[CONTROLADOR] Agarrando objeto {cube_id} na posição ({ee_pos[0]:.2f}, {ee_pos[1]:.2f})')
        cid = self.arm.attach(cube_id, -1)
        self.grasp_constraints[cube_id] = cid
        print(f'[CONTROLADOR] Objeto {cube_id} agarrado com sucesso!')

    def release(self, cube_id):
        ee_pos, _ = self.arm.get_ee_position()
        print(f'[CONTROLADOR] Soltando objeto {cube_id} na posição ({ee_pos[0]:.2f}, {ee_pos[1]:.2f})')
        cid = self.grasp_constraints.pop(cube_id, None)
        if cid:
            self.arm.detach(cid)
            print(f'[CONTROLADOR] Objeto {cube_id} solto com sucesso!')
        else:
            print(f'[CONTROLADOR] Erro: objeto {cube_id} não estava agarrado')

    def log_metrics(self):
        if self.arm.err_log:
            avg_err = sum(self.arm.err_log) / len(self.arm.err_log)
        else:
            avg_err = 0
        total_energy = sum(self.arm.energy_log) if hasattr(self.arm, 'energy_log') else 0
        max_err = self.arm.max_error if hasattr(self.arm, 'max_error') else 0
        print(f'[MÉTRICAS] Erro médio dos motores: {avg_err:.3f} rad | Máx. erro (overshoot): {max_err:.3f} rad | Energia total gasta: {total_energy:.3f} J')
        self.arm.err_log.clear()
        if hasattr(self.arm, 'energy_log'):
            self.arm.energy_log.clear()
        if hasattr(self.arm, 'max_error'):
            self.arm.max_error = 0.0