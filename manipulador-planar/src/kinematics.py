"""
Módulo de Cinemática para manipulador planar 2 DOF.

Fornece cálculos de cinemática direta, inversa e jacobiano para
um manipulador planar com duas juntas rotacionais.
"""

import numpy as np
from typing import Tuple, Dict
import logging


class PlanarKinematics:
    """
    Classe para cálculos cinemáticos de manipulador planar 2 DOF.
    
    O manipulador é configurado com origem na base, links consecutivos,
    todas as juntas rotacionais no plano (z).
    
    Attributes:
        link_lengths: Vetor com comprimentos dos links [L1, L2]
    """
    
    def __init__(self, link_lengths: Tuple[float, float]):
        """
        Inicializa a cinemática com os comprimentos dos links.
        
        Args:
            link_lengths: Tupla (L1, L2) com comprimentos em metros
        """
        self.link_lengths = np.array(link_lengths)
        self.logger = logging.getLogger(__name__)
        
    def forward_kinematics(self, theta: Tuple[float, float]) -> Dict[str, np.ndarray]:
        """
        Calcula cinemática direta (posição do end-effector).
        
        Para um manipulador planar 2 DOF com juntas rotacionais no plano XY:
        x = L1*cos(θ1) + L2*cos(θ1 + θ2)
        y = L1*sin(θ1) + L2*sin(θ1 + θ2)
        
        Args:
            theta: Tupla (θ1, θ2) com ângulos das juntas em radianos
            
        Returns:
            Dicionário com:
                - 'ee_position': Posição do end-effector [x, y]
                - 'joint2_position': Posição da segunda junta [x, y]
                - 'orientation': Orientação do end-effector (θ1 + θ2)
        """
        theta1, theta2 = theta
        L1, L2 = self.link_lengths
        
        # Ângulo cumulativo
        theta_cum = theta1 + theta2
        
        # Posição da primeira junta (origem)
        j1_pos = np.array([0.0, 0.0])
        
        # Posição da segunda junta (fim do link 1)
        j2_x = L1 * np.cos(theta1)
        j2_y = L1 * np.sin(theta1)
        j2_pos = np.array([j2_x, j2_y])
        
        # Posição do end-effector (fim do link 2)
        ee_x = L1 * np.cos(theta1) + L2 * np.cos(theta_cum)
        ee_y = L1 * np.sin(theta1) + L2 * np.sin(theta_cum)
        ee_pos = np.array([ee_x, ee_y])
        
        return {
            'ee_position': ee_pos,
            'j1_position': j1_pos,
            'j2_position': j2_pos,
            'orientation': theta_cum,
            'ee_distance': np.linalg.norm(ee_pos)
        }
    
    def jacobian(self, theta: Tuple[float, float]) -> np.ndarray:
        """
        Calcula a matriz Jacobiana do manipulador.
        
        Relaciona velocidades angulares das juntas com velocidades cartesianas
        do end-effector.
        
        Args:
            theta: Tupla (θ1, θ2) com ângulos das juntas
            
        Returns:
            Matriz Jacobiana 2x2
        """
        theta1, theta2 = theta
        L1, L2 = self.link_lengths
        
        theta_cum = theta1 + theta2
        
        # Jacobiana
        # J = [∂x/∂θ1  ∂x/∂θ2]
        #     [∂y/∂θ1  ∂y/∂θ2]
        
        j11 = -L1 * np.sin(theta1) - L2 * np.sin(theta_cum)
        j12 = -L2 * np.sin(theta_cum)
        j21 = L1 * np.cos(theta1) + L2 * np.cos(theta_cum)
        j22 = L2 * np.cos(theta_cum)
        
        J = np.array([[j11, j12],
                      [j21, j22]])
        
        return J
    
    def inverse_kinematics(self, 
                          target_pos: Tuple[float, float],
                          theta_init: Tuple[float, float] = None) -> Dict:
        """
        Calcula cinemática inversa usando método analítico.
        
        Para um manipulador planar 2 DOF, há até 2 soluções (elbow-up e elbow-down).
        Este método retorna a solução elbow-down por padrão.
        
        Args:
            target_pos: Tupla (x, y) posição alvo em metros
            theta_init: Tupla (θ1, θ2) posição inicial para seleção de solução
            
        Returns:
            Dicionário com:
                - 'theta': Tupla (θ1, θ2) solução
                - 'valid': Boolean indicando se a solução é válida
                - 'error': Mensagem de erro se inválida
        """
        x, y = target_pos
        L1, L2 = self.link_lengths
        
        # Calcular distância até alvo
        d = np.sqrt(x**2 + y**2)
        
        # Verificar se alvo está no espaço de trabalho
        if d > L1 + L2 or d < abs(L1 - L2):
            return {
                'theta': None,
                'valid': False,
                'error': f'Alvo fora do espaço de trabalho (d={d:.3f}m, '
                        f'limite=[{abs(L1-L2):.3f}, {L1+L2:.3f}]m)'
            }
        
        # Lei dos cossenos para encontrar θ2
        cos_theta2 = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
        
        if abs(cos_theta2) > 1:
            return {
                'theta': None,
                'valid': False,
                'error': 'Solução matemática inválida (cos(θ2) > 1)'
            }
        
        theta2_options = [np.arccos(cos_theta2), -np.arccos(cos_theta2)]
        
        best_theta = None
        best_distance = float('inf')
        
        for theta2 in theta2_options:
            # Calcular θ1
            k1 = L1 + L2 * np.cos(theta2)
            k2 = L2 * np.sin(theta2)
            theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
            
            # Normalizar para [-π, π]
            theta1 = self._normalize_angle(theta1)
            theta2_norm = self._normalize_angle(theta2)
            
            # Se theta_init foi fornecido, selecionar solução mais próxima
            if theta_init is not None:
                distance = abs(theta1 - theta_init[0]) + abs(theta2_norm - theta_init[1])
                if distance < best_distance:
                    best_distance = distance
                    best_theta = (theta1, theta2_norm)
            else:
                best_theta = (theta1, theta2_norm)
        
        return {
            'theta': best_theta,
            'valid': True,
            'error': None
        }
    
    def end_effector_velocity(self, 
                             theta: Tuple[float, float],
                             theta_dot: Tuple[float, float]) -> np.ndarray:
        """
        Calcula velocidade do end-effector baseada em velocidades angulares.
        
        Usa: v = J * ω
        
        Args:
            theta: Posição angular (θ1, θ2)
            theta_dot: Velocidade angular (ω1, ω2) em rad/s
            
        Returns:
            Vetor velocidade cartesiana [vx, vy] em m/s
        """
        J = self.jacobian(theta)
        omega = np.array([theta_dot[0], theta_dot[1]])
        
        velocity = J @ omega
        
        return velocity
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """
        Normaliza ângulo para intervalo [-π, π].
        
        Args:
            angle: Ângulo em radianos
            
        Returns:
            Ângulo normalizado em [-π, π]
        """
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def get_info(self) -> Dict[str, any]:
        """
        Retorna informações sobre o manipulador.
        
        Returns:
            Dicionário com parâmetros do manipulador
        """
        L1, L2 = self.link_lengths
        return {
            'link_lengths': self.link_lengths.tolist(),
            'max_reach': float(L1 + L2),
            'min_reach': float(abs(L1 - L2)),
            'workspace_area': np.pi * (L1 + L2)**2  # Aproximação circular
        }
