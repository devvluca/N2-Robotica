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
        
        # Detectar estagnação ou áreas não visitadas
        if self.frame_count % 120 == 0:
            # Verificar progresso
            if abs(coverage - self.last_coverage) < 0.3:
                self.coverage_stall_count += 1
                if self.coverage_stall_count >= 2:
                    # Forçar busca ativa
                    self.state = RobotState.SEEKING
                    self.coverage_stall_count = 0
                    # Mudar direção drasticamente
                    self.target_angle += math.pi / 2
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
        
        # Verificar bordas do mapa - MARGEM MAIOR
        margin = 0.35
        if pos[0] < margin or pos[0] > self.env.width - margin:
            return self._start_uturn(pos, angle)
        if pos[1] < margin or pos[1] > self.env.height - margin:
            return self._start_uturn(pos, angle)
        
        return np.clip(left_vel, 0.2, 1), np.clip(right_vel, 0.2, 1)
    
    def _seek_uncleaned(self, pos, angle, readings):
        """Busca ativamente áreas não limpas."""
        # PRIMEIRO: Verificar se está perto da borda
        margin = 0.35
        if pos[0] < margin or pos[0] > self.env.width - margin:
            return self._start_uturn(pos, angle)
        if pos[1] < margin or pos[1] > self.env.height - margin:
            return self._start_uturn(pos, angle)
        
        # Verificar obstáculos
        obstacle_info = self._check_obstacles(readings)
        if obstacle_info['blocked']:
            return self._start_escape_maneuver(readings, pos)
        if obstacle_info['wall_ahead']:
            return self._start_uturn(pos, angle)
        
        # Encontrar direção para área menos visitada
        best_dir = self._find_uncleaned_direction(pos, angle)
        
        if best_dir is None:
            # Nenhuma área encontrada - voltar a varrer
            self.state = RobotState.SWEEPING
            self.target_angle = angle + math.pi * 0.5 * self.sweep_direction
            return self.forward_speed, self.forward_speed
        
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
        """Encontra direção para área NÃO VISITADA (prioridade máxima)."""
        from src.mapping import CELL_UNKNOWN, CELL_FREE, CELL_OBSTACLE
        
        best_angle = None
        best_score = float('inf')
        found_unvisited = False
        
        # Verificar 24 direções
        for i in range(24):
            check_angle = (i / 24) * 2 * math.pi
            
            # Verificar múltiplas distâncias
            direction_score = 0
            unvisited_count = 0
            valid_checks = 0
            
            for dist in [0.2, 0.35, 0.5, 0.7, 0.9]:
                check_x = pos[0] + dist * math.cos(check_angle)
                check_y = pos[1] + dist * math.sin(check_angle)
                
                # Verificar se está dentro do mapa - MARGEM MAIOR
                if check_x < 0.3 or check_x > self.env.width - 0.3:
                    direction_score += 80  # Penalizar borda forte
                    continue
                if check_y < 0.3 or check_y > self.env.height - 0.3:
                    direction_score += 80
                    continue
                
                gx, gy = self.map.world_to_grid(check_x, check_y)
                if 0 <= gx < self.map.grid_width and 0 <= gy < self.map.grid_height:
                    cell_state = self.map.grid[gx, gy]
                    visits = self.map.visit_count[gx, gy]
                    
                    if cell_state == CELL_OBSTACLE:
                        direction_score += 200  # Evitar obstáculos fortemente
                    elif cell_state == CELL_UNKNOWN or visits == 0:
                        # PRIORIDADE MÁXIMA: área não visitada!
                        direction_score -= 100
                        unvisited_count += 1
                        found_unvisited = True
                    elif visits == 1:
                        direction_score -= 20  # Visitou só 1 vez, ok
                    else:
                        direction_score += visits * 10  # Penalizar muito revisitas
                    
                    valid_checks += 1
            
            if valid_checks > 0:
                # Bônus grande para direções com áreas não visitadas
                if unvisited_count > 0:
                    direction_score -= unvisited_count * 50
                
                if direction_score < best_score:
                    best_score = direction_score
                    best_angle = check_angle
        
        return best_angle
    
    def _normalize_angle(self, angle):
        """Normaliza ângulo para [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _count_unvisited_cells(self):
        """Conta células não visitadas (excluindo bordas e obstáculos)."""
        from src.mapping import CELL_UNKNOWN, CELL_OBSTACLE
        count = 0
        margin = 3  # Ignorar bordas
        for gx in range(margin, self.map.grid_width - margin):
            for gy in range(margin, self.map.grid_height - margin):
                if self.map.grid[gx, gy] == CELL_UNKNOWN:
                    count += 1
                elif self.map.visit_count[gx, gy] == 0 and self.map.grid[gx, gy] != CELL_OBSTACLE:
                    count += 1
        return count
    
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
    """
    Controlador inteligente que APRENDE com execuções anteriores.
    
    Comportamento:
    - Importa obstáculos conhecidos do mapa anterior (não precisa redescobrir)
    - Identifica áreas que NÃO foram limpas na execução anterior
    - Vai DIRETAMENTE para essas áreas primeiro
    - Evita passar por áreas já bem cobertas anteriormente
    """
    
    def __init__(self, robot, occupancy_map, environment, previous_map=None):
        super().__init__(robot, occupancy_map, environment)
        
        self.previous_map = previous_map
        self.use_learned = previous_map is not None
        self.priority_targets = []  # Lista de células prioritárias (não limpas antes)
        self.current_target = None
        
        if self.use_learned:
            print("[CTRL] 🧠 MODO APRENDIZADO ATIVO - usando mapa anterior")
            self._import_knowledge_from_previous()
            self._identify_priority_areas()
    
    def _import_knowledge_from_previous(self):
        """Importa conhecimento do mapa anterior."""
        if self.previous_map is None:
            return
        
        from src.mapping import CELL_OBSTACLE, CELL_FREE
        
        # Importar obstáculos conhecidos
        obstacle_mask = self.previous_map.grid == CELL_OBSTACLE
        self.map.grid[obstacle_mask] = CELL_OBSTACLE
        obstacle_count = np.sum(obstacle_mask)
        
        # Importar áreas livres conhecidas
        free_mask = self.previous_map.grid == CELL_FREE
        self.map.grid[free_mask] = CELL_FREE
        
        print(f"[CTRL] ✓ Importados {obstacle_count} células de obstáculos")
        print(f"[CTRL] ✓ Mapa inicializado com conhecimento prévio")
    
    def _identify_priority_areas(self):
        """Identifica áreas que NÃO foram bem limpas na execução anterior."""
        if self.previous_map is None:
            return
        
        from src.mapping import CELL_OBSTACLE
        
        margin = 4  # Margem das bordas
        self.priority_targets = []
        
        # Encontrar células pouco visitadas na execução anterior
        for gx in range(margin, self.map.grid_width - margin):
            for gy in range(margin, self.map.grid_height - margin):
                # Ignorar obstáculos
                if self.previous_map.grid[gx, gy] == CELL_OBSTACLE:
                    continue
                
                prev_visits = self.previous_map.visit_count[gx, gy]
                
                # Área não visitada ou pouco visitada = PRIORIDADE
                if prev_visits == 0:
                    # Converter para coordenadas do mundo
                    wx, wy = self.map.grid_to_world(gx, gy)
                    self.priority_targets.append({
                        'gx': gx, 'gy': gy,
                        'wx': wx, 'wy': wy,
                        'priority': 10  # Alta prioridade
                    })
                elif prev_visits == 1:
                    wx, wy = self.map.grid_to_world(gx, gy)
                    self.priority_targets.append({
                        'gx': gx, 'gy': gy,
                        'wx': wx, 'wy': wy,
                        'priority': 5  # Média prioridade
                    })
        
        # Ordenar por prioridade (maior primeiro)
        self.priority_targets.sort(key=lambda x: x['priority'], reverse=True)
        
        print(f"[CTRL] 🎯 Identificadas {len(self.priority_targets)} células prioritárias")
        
        if len(self.priority_targets) > 0:
            # Definir primeiro alvo
            self._select_next_target()
    
    def _select_next_target(self):
        """Seleciona próximo alvo prioritário mais próximo."""
        if not self.priority_targets:
            self.current_target = None
            return
        
        pos, _ = self.robot.get_pose()
        
        # Encontrar alvo prioritário mais próximo
        min_dist = float('inf')
        best_idx = 0
        
        for i, target in enumerate(self.priority_targets[:50]):  # Checar os 50 mais prioritários
            dist = math.sqrt((target['wx'] - pos[0])**2 + (target['wy'] - pos[1])**2)
            # Ponderar por prioridade
            score = dist / (target['priority'] + 1)
            if score < min_dist:
                min_dist = score
                best_idx = i
        
        self.current_target = self.priority_targets.pop(best_idx)
        print(f"[CTRL] → Navegando para ({self.current_target['wx']:.2f}, {self.current_target['wy']:.2f})")
    
    def _find_uncleaned_direction(self, pos, current_angle):
        """
        Versão melhorada que usa conhecimento do mapa anterior.
        Prioriza áreas não limpas na execução anterior.
        """
        from src.mapping import CELL_UNKNOWN, CELL_FREE, CELL_OBSTACLE
        
        # Se temos um alvo prioritário, ir para ele
        if self.use_learned and self.current_target:
            # Verificar se chegamos ao alvo
            dist_to_target = math.sqrt(
                (self.current_target['wx'] - pos[0])**2 + 
                (self.current_target['wy'] - pos[1])**2
            )
            
            if dist_to_target < 0.3:
                # Chegou - selecionar próximo
                self._select_next_target()
                if self.current_target is None:
                    print("[CTRL] ✓ Todas as áreas prioritárias cobertas!")
                    return super()._find_uncleaned_direction(pos, current_angle)
            
            # Calcular ângulo para o alvo
            dx = self.current_target['wx'] - pos[0]
            dy = self.current_target['wy'] - pos[1]
            return math.atan2(dy, dx)
        
        # Fallback para busca padrão
        best_angle = None
        best_score = float('inf')
        
        # Verificar 24 direções
        for i in range(24):
            check_angle = (i / 24) * 2 * math.pi
            direction_score = 0
            unvisited_count = 0
            valid_checks = 0
            
            for dist in [0.2, 0.35, 0.5, 0.7, 0.9]:
                check_x = pos[0] + dist * math.cos(check_angle)
                check_y = pos[1] + dist * math.sin(check_angle)
                
                if check_x < 0.3 or check_x > self.env.width - 0.3:
                    direction_score += 80
                    continue
                if check_y < 0.3 or check_y > self.env.height - 0.3:
                    direction_score += 80
                    continue
                
                gx, gy = self.map.world_to_grid(check_x, check_y)
                if 0 <= gx < self.map.grid_width and 0 <= gy < self.map.grid_height:
                    cell_state = self.map.grid[gx, gy]
                    curr_visits = self.map.visit_count[gx, gy]
                    
                    # Considerar visitas anteriores se disponível
                    prev_visits = 0
                    if self.use_learned and self.previous_map is not None:
                        prev_visits = self.previous_map.visit_count[gx, gy]
                    
                    if cell_state == CELL_OBSTACLE:
                        direction_score += 200
                    elif curr_visits == 0 and prev_visits == 0:
                        # Nunca visitada em nenhuma execução = MÁXIMA PRIORIDADE
                        direction_score -= 150
                        unvisited_count += 1
                    elif curr_visits == 0:
                        # Não visitada nesta execução
                        direction_score -= 100
                        unvisited_count += 1
                    else:
                        # Penalizar revisitas, considerando histórico
                        total_visits = curr_visits + prev_visits * 0.5
                        direction_score += total_visits * 10
                    
                    valid_checks += 1
            
            if valid_checks > 0:
                if unvisited_count > 0:
                    direction_score -= unvisited_count * 50
                
                if direction_score < best_score:
                    best_score = direction_score
                    best_angle = check_angle
        
        return best_angle
