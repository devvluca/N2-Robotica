"""
Módulo de Navegação para Robô Aspirador.

Implementa lógicas de navegação, evasão de obstáculos e controle de movimento.
"""

import numpy as np
import logging
from typing import Dict, Tuple, Optional, List
from enum import Enum


class NavigationState(Enum):
    """Estados de navegação do robô."""
    IDLE = "idle"
    MOVING = "moving"
    AVOIDING = "avoiding"
    TURNING = "turning"
    BACKTRACKING = "backtracking"
    CHARGING = "charging"


class NavigationModule:
    """
    Módulo de navegação para controlar movimento do robô.
    
    Implementa:
    - Evasão de obstáculos
    - Seguimento de parede
    - Estratégias de exploração
    - Controle de velocidade
    """
    
    def __init__(self, config: Dict):
        """
        Inicializa módulo de navegação.
        
        Args:
            config: Dicionário com configurações
        """
        self.config = config
        self.logger = logging.getLogger(__name__)
        
        # Estado de navegação
        self.state = NavigationState.IDLE
        self.current_heading = 0.0  # radianos
        self.target_heading = 0.0
        
        # Histórico de movimentos
        self.movement_history = []
        self.stuck_detection = []
        
        # Parâmetros de navegação
        self.obstacle_threshold = config['navigation']['obstacle_threshold']
        self.turn_speed = config['navigation']['turn_speed']
        self.turn_angle = np.radians(config['navigation']['turn_angle'])
    
    def compute_velocity(self, 
                        sensor_readings: List[float],
                        target_heading: Optional[float] = None) -> Tuple[float, float]:
        """
        Calcula velocidades linear e angular baseado em sensores.
        
        Args:
            sensor_readings: Lista de distâncias dos sensores
            target_heading: Heading alvo em radianos (opcional)
            
        Returns:
            Tupla (velocidade_linear, velocidade_angular)
        """
        max_linear = self.config['robot']['max_linear_velocity']
        max_angular = self.config['robot']['max_angular_velocity']
        
        # Verificar detecção de obstáculos
        center_distance = sensor_readings[2] if len(sensor_readings) > 2 else float('inf')
        left_distance = sensor_readings[1] if len(sensor_readings) > 1 else float('inf')
        right_distance = sensor_readings[3] if len(sensor_readings) > 3 else float('inf')
        
        # Lógica de evasão de obstáculos
        if center_distance < self.obstacle_threshold:
            # Obstáculo à frente - virar
            self.state = NavigationState.AVOIDING
            
            # Virar para o lado com mais espaço
            if left_distance > right_distance:
                linear_velocity = 0.0
                angular_velocity = self.turn_speed
            else:
                linear_velocity = 0.0
                angular_velocity = -self.turn_speed
        
        elif center_distance < self.obstacle_threshold * 1.5:
            # Reduzir velocidade se aproximando
            self.state = NavigationState.MOVING
            linear_velocity = max_linear * (center_distance / self.obstacle_threshold)
            angular_velocity = 0.0
        
        else:
            # Movimento livre
            self.state = NavigationState.MOVING
            linear_velocity = max_linear * 0.8  # 80% da velocidade máxima
            angular_velocity = 0.0
        
        # Aplicar wall following se habilitado
        if self.config['navigation']['enable_wall_following']:
            angular_velocity += self._wall_following_correction(left_distance, right_distance)
        
        # Limitar saídas
        linear_velocity = np.clip(linear_velocity, -max_linear, max_linear)
        angular_velocity = np.clip(angular_velocity, -max_angular, max_angular)
        
        return linear_velocity, angular_velocity
    
    def _wall_following_correction(self, 
                                  left_distance: float, 
                                  right_distance: float) -> float:
        """
        Calcula correção angular para seguimento de parede.
        
        Args:
            left_distance: Distância do sensor esquerdo
            right_distance: Distância do sensor direito
            
        Returns:
            Correção angular em rad/s
        """
        # PID simples para manter distância equilibrada
        error = left_distance - right_distance
        max_angular = self.config['robot']['max_angular_velocity']
        
        correction = error * 0.5  # Ganho proporcional
        correction = np.clip(correction, -max_angular * 0.2, max_angular * 0.2)
        
        return correction
    
    def compute_heading_correction(self, 
                                  current_heading: float,
                                  target_heading: float) -> float:
        """
        Calcula velocidade angular para atingir heading alvo.
        
        Args:
            current_heading: Heading atual em radianos
            target_heading: Heading alvo em radianos
            
        Returns:
            Velocidade angular em rad/s
        """
        # Calcular diferença angular (normalizada)
        delta = target_heading - current_heading
        
        # Normalizar para [-π, π]
        while delta > np.pi:
            delta -= 2 * np.pi
        while delta < -np.pi:
            delta += 2 * np.pi
        
        # Controle proporcional
        max_angular = self.config['robot']['max_angular_velocity']
        kp = 2.0
        
        angular_velocity = kp * delta
        angular_velocity = np.clip(angular_velocity, -max_angular, max_angular)
        
        return angular_velocity
    
    def is_stuck(self, 
                position_history: List[Tuple[float, float]], 
                window_size: int = 10) -> bool:
        """
        Detecta se o robô está preso.
        
        Args:
            position_history: Histórico de posições
            window_size: Número de posições para verificar
            
        Returns:
            True se o robô está preso
        """
        if len(position_history) < window_size:
            return False
        
        # Obter últimas N posições
        recent_positions = position_history[-window_size:]
        
        # Calcular distância total percorrida
        total_distance = 0.0
        for i in range(1, len(recent_positions)):
            dx = recent_positions[i][0] - recent_positions[i-1][0]
            dy = recent_positions[i][1] - recent_positions[i-1][1]
            total_distance += np.sqrt(dx**2 + dy**2)
        
        # Considerar preso se moveu menos de 5cm em 10 leituras
        return total_distance < 0.05
    
    def get_recovery_maneuver(self) -> Tuple[float, float]:
        """
        Retorna velocidades para manobra de recuperação.
        
        Returns:
            Tupla (velocidade_linear, velocidade_angular)
        """
        # Marcha à ré e girar
        return -0.2, self.turn_speed
    
    def update_heading(self, new_heading: float) -> None:
        """
        Atualiza heading atual.
        
        Args:
            new_heading: Novo heading em radianos
        """
        # Normalizar para [-π, π]
        while new_heading > np.pi:
            new_heading -= 2 * np.pi
        while new_heading < -np.pi:
            new_heading += 2 * np.pi
        
        self.current_heading = new_heading
    
    def get_state(self) -> Dict:
        """Retorna estado atual de navegação."""
        return {
            'state': self.state.value,
            'current_heading': float(self.current_heading),
            'target_heading': float(self.target_heading)
        }


class ExplorationStrategy:
    """
    Estratégias de exploração para otimização de rota.
    """
    
    @staticmethod
    def spiral(center_x: float, center_y: float, iteration: int = 0) -> Tuple[float, float]:
        """
        Estratégia em espiral.
        
        Args:
            center_x: Centro X em metros
            center_y: Centro Y em metros
            iteration: Número de iterações
            
        Returns:
            Tupla (x, y) próximo alvo em metros
        """
        angle = iteration * 0.3  # incremento angular
        radius = 0.5 + iteration * 0.1  # raio crescente
        
        x = center_x + radius * np.cos(angle)
        y = center_y + radius * np.sin(angle)
        
        return x, y
    
    @staticmethod
    def systematic_lines(room_width: float, 
                        room_height: float,
                        iteration: int = 0,
                        line_width: float = 0.5) -> Tuple[float, float]:
        """
        Estratégia de linhas sistemáticas.
        
        Args:
            room_width: Largura da sala
            room_height: Altura da sala
            iteration: Número da linha
            line_width: Espaçamento entre linhas
            
        Returns:
            Tupla (x, y) próximo alvo
        """
        line_number = int(iteration // 2)
        y = line_width / 2 + line_number * line_width
        
        if iteration % 2 == 0:
            x = line_width / 2
        else:
            x = room_width - line_width / 2
        
        y = np.clip(y, 0, room_height)
        
        return x, y
    
    @staticmethod
    def random_walk(current_x: float, 
                   current_y: float,
                   step_size: float = 0.5) -> Tuple[float, float]:
        """
        Estratégia de passeio aleatório.
        
        Args:
            current_x: Posição X atual
            current_y: Posição Y atual
            step_size: Tamanho do passo em metros
            
        Returns:
            Tupla (x, y) próximo alvo
        """
        angle = np.random.uniform(0, 2 * np.pi)
        x = current_x + step_size * np.cos(angle)
        y = current_y + step_size * np.sin(angle)
        
        return x, y
