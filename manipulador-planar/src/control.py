import pybullet as p
import math
import time

from src.kinematics import PlanarArmKinematics

class ArmController:
    def __init__(self, arm, kinematics: PlanarArmKinematics, lift_height=0.25):
        self.arm = arm
        self.kin = kinematics
        self.lift_height = lift_height
        self.grasp_constraints = {}
        self.obstacle_ids = []  # Lista de obstáculos conhecidos
        self.carried_mass = 0.0  # Massa do objeto sendo carregado
        self.perturbation_threshold = 0.05  # Limiar para detectar perturbação (rad)

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

    def register_obstacle(self, obstacle_id):
        """Registra um obstáculo para detecção de colisão."""
        if obstacle_id >= 0 and obstacle_id not in self.obstacle_ids:
            self.obstacle_ids.append(obstacle_id)
            print(f'[CONTROLADOR] Obstáculo {obstacle_id} registrado para detecção de colisão')
    
    def check_obstacle_between(self, target_x, target_y):
        """Verifica se há obstáculo no caminho ou perto do destino."""
        ee_pos, _ = self.arm.get_ee_position()
        
        # Verificar se algum obstáculo está no caminho
        for obs_id in self.obstacle_ids:
            obs_pos, _ = p.getBasePositionAndOrientation(obs_id)
            obs_x, obs_y = obs_pos[0], obs_pos[1]
            
            # Verificar se obstáculo está entre posição atual e destino
            # Usando distância perpendicular do obstáculo à linha de trajetória
            dx = target_x - ee_pos[0]
            dy = target_y - ee_pos[1]
            path_len = math.sqrt(dx*dx + dy*dy)
            
            if path_len < 0.01:
                continue
                
            # Vetor unitário da trajetória
            ux, uy = dx/path_len, dy/path_len
            
            # Vetor do início ao obstáculo
            ox = obs_x - ee_pos[0]
            oy = obs_y - ee_pos[1]
            
            # Projeção do obstáculo na linha de trajetória
            proj = ox*ux + oy*uy
            
            # Se a projeção está entre 0 e path_len, o obstáculo pode estar no caminho
            if 0 < proj < path_len:
                # Distância perpendicular do obstáculo à linha
                perp_dist = abs(ox*uy - oy*ux)
                
                # Se está perto da trajetória (raio de 0.15m)
                if perp_dist < 0.15:
                    print(f'[COLISÃO] Obstáculo detectado a {perp_dist:.2f}m da trajetória!')
                    return (obs_id, None, proj/path_len, [obs_x, obs_y, 0])
        
        return None
    
    def check_physical_collision(self):
        """Verifica colisão física real com obstáculos."""
        for obs_id in self.obstacle_ids:
            contacts = p.getContactPoints(self.arm.id, obs_id)
            if contacts:
                return obs_id
        return None
    
    def check_collision_at_config(self, angles):
        """Verifica se uma configuração de ângulos causa colisão."""
        # Salvar estado atual
        current_angles = self.arm.get_joint_angles()
        
        # Temporariamente mover para a configuração teste (sem física)
        for i, j in enumerate(self.arm.arm_joints):
            p.resetJointState(self.arm.id, j, angles[i])
        
        # Verificar colisão com obstáculos
        collision = False
        for obs_id in self.obstacle_ids:
            contacts = p.getContactPoints(self.arm.id, obs_id)
            if contacts:
                collision = True
                break
        
        # Restaurar estado original
        for i, j in enumerate(self.arm.arm_joints):
            p.resetJointState(self.arm.id, j, current_angles[i])
        
        return collision
    
    def is_position_reachable(self, x, y):
        """Verifica se uma posição está dentro do espaço de trabalho."""
        reach = sum(self.kin.link_lengths)
        min_reach = abs(self.kin.link_lengths[0] - sum(self.kin.link_lengths[1:]))
        distance = math.sqrt(x*x + y*y)
        return min_reach <= distance <= reach

    def plan_trajectory_around_obstacle(self, start_pos, target_x, target_y):
        """Planeja trajetória desviando de obstáculos - recolhe, passa por cima, estica."""
        waypoints = []
        
        # Encontrar posição do obstáculo
        obs_pos = None
        for obs_id in self.obstacle_ids:
            pos, _ = p.getBasePositionAndOrientation(obs_id)
            obs_pos = [pos[0], pos[1]]
            break
        
        if obs_pos is None:
            return waypoints
        
        # Configuração
        retract_distance = 0.45  # Braço curto - força dobra visível
        safe_y = 0.55           # Y seguro acima do obstáculo (obs em Y=0.30)
        
        # Determinar de onde para onde vamos
        start_below_obs = start_pos[1] < obs_pos[1]
        target_below_obs = target_y < obs_pos[1]
        
        print(f'[TRAJETÓRIA] Início Y={start_pos[1]:.2f} ({"ABAIXO" if start_below_obs else "ACIMA"}), Alvo Y={target_y:.2f} ({"ABAIXO" if target_below_obs else "ACIMA"})')
        
        if target_below_obs:
            # ALVO ESTÁ ABAIXO DO OBSTÁCULO (lado DENTRO)
            # 1. Recolher
            wp1 = (retract_distance, max(start_pos[1], safe_y) if not start_below_obs else 0.1)
            # 2. Ir para Y seguro
            wp2 = (retract_distance, safe_y)
            # 3. Descer para Y baixo
            wp3 = (0.60, 0.0)
        else:
            # ALVO ESTÁ ACIMA DO OBSTÁCULO (lado FORA ou tray)
            # 1. Recolher
            wp1 = (retract_distance, min(start_pos[1], 0.1) if start_below_obs else safe_y)
            # 2. Ir para Y seguro
            wp2 = (retract_distance, safe_y)
            # 3. Esticar em Y alto
            wp3 = (0.85, safe_y)
        
        waypoints = [wp1, wp2, wp3]
        
        print(f'[TRAJETÓRIA] 1. Recolhendo: ({wp1[0]:.2f}, {wp1[1]:.2f})')
        print(f'[TRAJETÓRIA] 2. Passando por cima: ({wp2[0]:.2f}, {wp2[1]:.2f})')
        print(f'[TRAJETÓRIA] 3. Preparando: ({wp3[0]:.2f}, {wp3[1]:.2f})')
        
        return waypoints
        
        return waypoints
    
    def avoid_and_move(self, target_x, target_y, duration=1.0):
        """Move para o alvo desviando de obstáculos automaticamente."""
        ee_pos, _ = self.arm.get_ee_position()
        
        # Verificar se posição é alcançável
        if not self.is_position_reachable(target_x, target_y):
            print(f'[CONTROLADOR] ⚠ Posição ({target_x:.2f}, {target_y:.2f}) fora do alcance!')
            # Ajustar para posição alcançável mais próxima
            reach = sum(self.kin.link_lengths)
            r = math.sqrt(target_x**2 + target_y**2)
            if r > reach:
                scale = (reach - 0.05) / r
                target_x *= scale
                target_y *= scale
                print(f'[CONTROLADOR] → Ajustado para ({target_x:.2f}, {target_y:.2f})')
        
        # Verificar obstáculo no caminho
        obstacle = self.check_obstacle_between(target_x, target_y)
        if obstacle:
            hit_pos = obstacle[3]  # Posição do hit
            print(f'[CONTROLADOR] 🚧 Obstáculo detectado em ({hit_pos[0]:.2f}, {hit_pos[1]:.2f})!')
            
            # Planejar trajetória alternativa
            waypoints = self.plan_trajectory_around_obstacle([ee_pos[0], ee_pos[1]], target_x, target_y)
            
            # Executar waypoints com tempo adequado para movimentos suaves
            time_per_wp = max(1.0, duration / (len(waypoints) + 1))
            for wp in waypoints:
                print(f'[CONTROLADOR] → Movendo para waypoint ({wp[0]:.2f}, {wp[1]:.2f})')
                self.move_to_xy(wp[0], wp[1], time_per_wp)
                # Pequena pausa para estabilizar
                for _ in range(30):
                    p.stepSimulation()
                    import time as t
                    t.sleep(1/240)
            
            # Finalmente ir ao destino
            return self.move_to_xy(target_x, target_y, time_per_wp)
        else:
            return self.move_to_xy(target_x, target_y, duration)

    def grasp(self, cube_id):
        """Agarra objeto e compensa seu peso."""
        ee_pos, _ = self.arm.get_ee_position()
        print(f'[CONTROLADOR] Agarrando objeto {cube_id} na posição ({ee_pos[0]:.2f}, {ee_pos[1]:.2f})')
        
        # Obter massa do objeto
        try:
            dynamics = p.getDynamicsInfo(cube_id, -1)
            self.carried_mass = dynamics[0]  # massa
            print(f'[CONTROLADOR] Massa do objeto: {self.carried_mass:.3f} kg')
        except:
            self.carried_mass = 0.1  # massa padrão
        
        cid = self.arm.attach(cube_id, -1)
        self.grasp_constraints[cube_id] = cid
        print(f'[CONTROLADOR] Objeto {cube_id} agarrado com sucesso!')
        
        # Compensar perturbação inicial após agarrar
        self.compensate_perturbation()

    def release(self, cube_id):
        """Solta objeto e reseta compensação de peso."""
        ee_pos, _ = self.arm.get_ee_position()
        print(f'[CONTROLADOR] Soltando objeto {cube_id} na posição ({ee_pos[0]:.2f}, {ee_pos[1]:.2f})')
        cid = self.grasp_constraints.pop(cube_id, None)
        if cid:
            self.arm.detach(cid)
            self.carried_mass = 0.0
            print(f'[CONTROLADOR] Objeto {cube_id} solto com sucesso!')
        else:
            print(f'[CONTROLADOR] Erro: objeto {cube_id} não estava agarrado')
    
    def compensate_perturbation(self):
        """Detecta e corrige perturbações na posição do efetuador."""
        target_angles = self.arm.get_joint_angles()
        
        # Executar alguns passos de estabilização
        max_corrections = 10
        for i in range(max_corrections):
            # Medir erro atual
            current_angles = self.arm.get_joint_angles()
            errors = [abs(t - c) for t, c in zip(target_angles, current_angles)]
            max_error = max(errors)
            
            if max_error < self.perturbation_threshold:
                if i > 0:
                    print(f'[ESTABILIZAÇÃO] ✓ Estabilizado após {i} correções (erro: {max_error:.4f} rad)')
                break
            
            # Aplicar correção PID
            for _ in range(20):
                self.arm.pid_step(target_angles)
                p.stepSimulation()
                time.sleep(1/240)
        else:
            print(f'[ESTABILIZAÇÃO] ⚠ Não estabilizou completamente (erro: {max_error:.4f} rad)')
    
    def apply_perturbation_test(self, force_magnitude=5.0):
        """Aplica uma perturbação externa para testar a compensação."""
        ee_state = p.getLinkState(self.arm.id, self.arm.eef_link_index)
        ee_pos = ee_state[0]
        
        # Aplicar força lateral
        force = [0, force_magnitude, 0]
        p.applyExternalForce(self.arm.id, self.arm.eef_link_index, 
                            force, ee_pos, p.WORLD_FRAME)
        print(f'[TESTE] Perturbação aplicada: {force}')
        
        # Deixar a física reagir
        for _ in range(30):
            p.stepSimulation()
            time.sleep(1/240)
        
        # Compensar
        self.compensate_perturbation()

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