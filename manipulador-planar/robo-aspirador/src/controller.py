"""
Controlador do Robô Aspirador.
Algoritmo de varredura sistemática (boustrophedon) para alta cobertura.
"""

import numpy as np
import math
import time as time_module
from enum import Enum


class RobotState(Enum):
    """Estados do robô."""
    SWEEPING = 1        # Varredura sistemática (vai e volta)
    TURNING_END = 2     # Girando no fim da faixa
    AVOIDING = 3        # Evitando obstáculo
    REVERSING = 4       # Dando ré
    SEEKING = 5         # Buscando área não limpa
    FINISHED = 6        # Terminou limpeza


class NavigationController:
    """Controlador de navegação com varredura sistemática."""
    
    def __init__(self, robot, occupancy_map, environment):
        self.robot = robot
        self.map = occupancy_map
        self.env = environment
        
        # Estado atual
        self.state = RobotState.SWEEPING
        
        # Parâmetros de navegação - RÁPIDO
        self.safe_distance = 0.22      # Distância de segurança
        self.turn_speed = 1.0          # Velocidade de giro (aumentada)
        self.forward_speed = 1.0       # Velocidade máxima frente
        self.reverse_speed = 0.9       # Velocidade de ré (aumentada)
        
        # Parâmetros de varredura
        self.sweep_direction = 1       # 1 = direita (0°), -1 = esquerda (180°)
        self.sweep_row = 0             # Linha atual de varredura
        self.row_spacing = 0.28        # Espaçamento entre linhas (~diâmetro do robô)
        self.target_angle = 0          # Ângulo alvo atual
        
        # Contadores de manobra
        self.maneuver_counter = 0
        self.turn_phase = 0            # Fase do giro em U (0, 1, 2)
        
        # Detecção de travamento
        self.position_history = []
        self.stuck_check_interval = 80
        self.frame_count = 0
        self.last_coverage = 0
        self.coverage_stall_count = 0
        
        # Métricas
        self.collision_count = 0
        self._last_collision_time = 0
        self.time_in_states = {state: 0 for state in RobotState}
        
    def update(self, dt, sensor_readings, current_time):
        """Atualiza o controlador e retorna comando de velocidade."""
        pos, angle = self.robot.get_pose()
        self.frame_count += 1
        
        # Atualizar mapa
        self.map.update_from_sensors(
            pos, angle, sensor_readings,
            self.robot.sensor_angles,
            self.robot.sensor_range
        )
        
        # Marcar área limpa
        self.map.mark_cleaned(pos[0], pos[1], current_time, self.robot.radius)
        
        # Registrar trajetória
        if len(self.map.trajectory) == 0 or current_time - self.map.trajectory[-1]['time'] > 0.2:
            self.map.record_trajectory(pos[0], pos[1], angle, current_time)
        
        # Atualizar tempo no estado
        self.time_in_states[self.state] += dt
        
        # Verificar cobertura
        stats = self.map.get_coverage_stats()
        coverage = stats['coverage_percent']
        
        if coverage >= 92:
            self.state = RobotState.FINISHED
            return 0, 0
        
        # Detectar estagnação de cobertura
        if self.frame_count % 150 == 0:
            if abs(coverage - self.last_coverage) < 0.3:
                self.coverage_stall_count += 1
                if self.coverage_stall_count >= 2:
                    # Mudar estratégia - buscar área não limpa
                    self.state = RobotState.SEEKING
                    self.coverage_stall_count = 0
                    # Alternar direção de busca
                    self.sweep_direction *= -1
            else:
                self.coverage_stall_count = 0
            self.last_coverage = coverage
        
        # ========== MÁQUINA DE ESTADOS ==========
        
        if self.state == RobotState.REVERSING:
            self.maneuver_counter -= 1
            if self.maneuver_counter <= 0:
                self.state = RobotState.TURNING_END
                self.turn_phase = 0
                self.maneuver_counter = 50
            return -self.reverse_speed, -self.reverse_speed
        
        if self.state == RobotState.TURNING_END:
            return self._execute_turn_maneuver(pos)
        
        if self.state == RobotState.SEEKING:
            return self._seek_uncleaned(pos, angle, sensor_readings)
        
        # Verificar obstáculos
        obstacle_info = self._check_obstacles(sensor_readings)
        
        if obstacle_info['blocked']:
            return self._start_escape_maneuver(sensor_readings, pos)
        elif obstacle_info['wall_ahead']:
            # Parede/borda detectada - fazer curva em U
            return self._start_uturn(pos, angle)
        elif obstacle_info['warning']:
            self.state = RobotState.AVOIDING
            return self._avoid_obstacle(sensor_readings)
        else:
            self.state = RobotState.SWEEPING
            return self._sweep(pos, angle, sensor_readings)
    
    def _check_obstacles(self, readings):
        """Verifica proximidade de obstáculos."""
        front = readings[2]
        left45 = readings[1]
        right45 = readings[3]
        left90 = readings[0]
        right90 = readings[4]
        
        threshold = self.safe_distance / self.robot.sensor_range
        
        # Parede/borda à frente (para fazer curva em U)
        wall_ahead = front < threshold * 1.2 and (left90 < 0.3 or right90 < 0.3)
        
        # Muito perto - precisa dar ré
        blocked = front < threshold * 0.5 or left45 < threshold * 0.4 or right45 < threshold * 0.4
        
        # Perto - precisa desviar
        warning = front < threshold * 0.8 or left45 < threshold * 0.7 or right45 < threshold * 0.7
        
        return {'blocked': blocked, 'warning': warning, 'wall_ahead': wall_ahead}
    
    def _start_uturn(self, pos, angle):
        """Inicia curva em U para próxima faixa."""
        self.sweep_direction *= -1
        self.sweep_row += 1
        self.state = RobotState.TURNING_END
        self.turn_phase = 0
        self.maneuver_counter = 60
        
        # Atualizar ângulo alvo
        if self.sweep_direction > 0:
            self.target_angle = 0
        else:
            self.target_angle = math.pi
        
        return self._execute_turn_maneuver(pos)
    
    def _execute_turn_maneuver(self, pos):
        """Executa manobra de curva em U em fases."""
        self.maneuver_counter -= 1
        
        if self.turn_phase == 0:
            # Fase 1: Girar 90°
            if self.maneuver_counter <= 0:
                self.turn_phase = 1
                self.maneuver_counter = 40
            if self.sweep_direction > 0:
                return -self.turn_speed, self.turn_speed  # Girar esquerda
            else:
                return self.turn_speed, -self.turn_speed  # Girar direita
        
        elif self.turn_phase == 1:
            # Fase 2: Avançar um pouco
            if self.maneuver_counter <= 0:
                self.turn_phase = 2
                self.maneuver_counter = 60
            return self.forward_speed * 0.7, self.forward_speed * 0.7
        
        else:
            # Fase 3: Girar mais 90°
            if self.maneuver_counter <= 0:
                self.state = RobotState.SWEEPING
                self.position_history.clear()
            if self.sweep_direction > 0:
                return -self.turn_speed, self.turn_speed
            else:
                return self.turn_speed, -self.turn_speed
    
    def _start_escape_maneuver(self, readings, pos):
        """Inicia manobra de escape: ré + giro."""
        import time
        now = time.time()
        if now - self._last_collision_time > 0.5:
            self.collision_count += 1
            self._last_collision_time = now
        
        self.state = RobotState.REVERSING
        self.maneuver_counter = 50
        
        return -self.reverse_speed, -self.reverse_speed
    
    def _avoid_obstacle(self, readings):
        """Desviar de obstáculo suavemente."""
        left_space = readings[0] + readings[1] * 1.5
        right_space = readings[4] + readings[3] * 1.5
        
        if left_space > right_space:
            return self.forward_speed * 0.4, self.forward_speed * 0.9
        else:
            return self.forward_speed * 0.9, self.forward_speed * 0.4
    
    def _sweep(self, pos, angle, readings):
        """Varredura sistemática em linhas."""
        # Manter direção da varredura
        angle_error = self._normalize_angle(self.target_angle - angle)
        
        # Correção suave de ângulo
        if abs(angle_error) > 0.1:
            correction = np.clip(angle_error * 2.0, -0.4, 0.4)
            left_vel = self.forward_speed - correction
            right_vel = self.forward_speed + correction
        else:
            left_vel = self.forward_speed
            right_vel = self.forward_speed
        
        # Verificar bordas do mapa
        margin = 0.25
        if pos[0] < margin or pos[0] > self.env.width - margin:
            return self._start_uturn(pos, angle)
        if pos[1] < margin or pos[1] > self.env.height - margin:
            return self._start_uturn(pos, angle)
        
        return np.clip(left_vel, 0.2, 1), np.clip(right_vel, 0.2, 1)
    
    def _seek_uncleaned(self, pos, angle, readings):
        """Busca ativamente áreas não limpas."""
        # Encontrar direção para área menos visitada
        best_dir = self._find_uncleaned_direction(pos, angle)
        
        if best_dir is None:
            # Nenhuma área encontrada - voltar a varrer aleatoriamente
            self.state = RobotState.SWEEPING
            self.target_angle = angle + math.pi * 0.5 * self.sweep_direction
            return self.forward_speed, self.forward_speed
        
        # Verificar obstáculos primeiro
        obstacle_info = self._check_obstacles(readings)
        if obstacle_info['blocked']:
            return self._start_escape_maneuver(readings, pos)
        
        # Virar na direção
        angle_error = self._normalize_angle(best_dir - angle)
        
        if abs(angle_error) > 0.2:
            # Precisa girar
            if angle_error > 0:
                return self.forward_speed * 0.2, self.forward_speed * 0.9
            else:
                return self.forward_speed * 0.9, self.forward_speed * 0.2
        else:
            # Ir em frente na direção
            self.target_angle = best_dir
            return self.forward_speed, self.forward_speed
    
    def _find_uncleaned_direction(self, pos, current_angle):
        """Encontra direção para área com menos visitas."""
        from src.mapping import CELL_UNKNOWN, CELL_FREE, CELL_OBSTACLE
        
        best_angle = None
        best_score = float('inf')
        
        # Verificar 24 direções para cobertura mais fina
        for i in range(24):
            check_angle = (i / 24) * 2 * math.pi
            
            # Verificar múltiplas distâncias
            total_score = 0
            valid_checks = 0
            
            for dist in [0.25, 0.4, 0.6, 0.8]:
                check_x = pos[0] + dist * math.cos(check_angle)
                check_y = pos[1] + dist * math.sin(check_angle)
                
                # Verificar se está dentro do mapa
                if check_x < 0.2 or check_x > self.env.width - 0.2:
                    continue
                if check_y < 0.2 or check_y > self.env.height - 0.2:
                    continue
                
                gx, gy = self.map.world_to_grid(check_x, check_y)
                if 0 <= gx < self.map.grid_width and 0 <= gy < self.map.grid_height:
                    cell_state = self.map.grid[gx, gy]
                    
                    if cell_state == CELL_OBSTACLE:
                        total_score += 100  # Evitar obstáculos
                    elif cell_state == CELL_UNKNOWN:
                        total_score -= 10  # Prioridade alta para desconhecido
                    else:
                        visits = self.map.visit_count[gx, gy]
                        total_score += visits * 2  # Evitar áreas muito visitadas
                    
                    valid_checks += 1
            
            if valid_checks > 0:
                avg_score = total_score / valid_checks
                if avg_score < best_score:
                    best_score = avg_score
                    best_angle = check_angle
        
        return best_angle
    
    def _normalize_angle(self, angle):
        """Normaliza ângulo para [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def get_state_name(self):
        """Retorna nome do estado atual."""
        return self.state.name
    
    def get_metrics(self):
        """Retorna métricas do controlador."""
        return {
            'state': self.state.name,
            'collisions': self.collision_count,
            'time_in_states': {k.name: round(v, 2) for k, v in self.time_in_states.items()}
        }


class SmartNavigationController(NavigationController):
    """Controlador inteligente que aprende com execuções anteriores."""
    
    def __init__(self, robot, occupancy_map, environment, previous_map=None):
        super().__init__(robot, occupancy_map, environment)
        
        self.previous_map = previous_map
        self.use_learned = previous_map is not None
        
        if self.use_learned:
            print("[CTRL] Usando mapa anterior para otimização")
            self._import_obstacles_from_previous()
    
    def _import_obstacles_from_previous(self):
        """Importa obstáculos conhecidos do mapa anterior."""
        if self.previous_map is None:
            return
        
        from src.mapping import CELL_OBSTACLE
        
        obstacle_mask = self.previous_map.grid == CELL_OBSTACLE
        self.map.grid[obstacle_mask] = CELL_OBSTACLE
        
        count = np.sum(obstacle_mask)
        print(f"[CTRL] Importados {count} células de obstáculos")
    
    def _explore(self, readings, pos, angle):
        """Exploração otimizada usando conhecimento anterior."""
        if not self.use_learned:
            return super()._explore(readings, pos, angle)
        
        best_direction = 0
        min_score = float('inf')
        
        directions = [0, math.pi/6, math.pi/3, -math.pi/6, -math.pi/3]
        
        for direction in directions:
            check_angle = angle + direction
            check_x = pos[0] + 0.4 * math.cos(check_angle)
            check_y = pos[1] + 0.4 * math.sin(check_angle)
            
            gx, gy = self.map.world_to_grid(check_x, check_y)
            if 0 <= gx < self.map.grid_width and 0 <= gy < self.map.grid_height:
                if self.map.grid[gx, gy] != 3:  # CELL_OBSTACLE
                    prev_visits = self.previous_map.visit_count[gx, gy]
                    curr_visits = self.map.visit_count[gx, gy]
                    
                    score = prev_visits * 0.3 + curr_visits * 2
                    
                    if score < min_score:
                        min_score = score
                        best_direction = direction
        
        # Virar na direção
        error = best_direction
        turn_rate = np.clip(error * 1.5, -0.5, 0.5)
        
        left_vel = self.forward_speed - turn_rate * 0.3
        right_vel = self.forward_speed + turn_rate * 0.3
        
        return np.clip(left_vel, 0.3, 1), np.clip(right_vel, 0.3, 1)
