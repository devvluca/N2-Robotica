"""
Implementação de Controlador PID para manipulador planar.

Este módulo fornece uma classe de controlador PID com anti-windup
e limites de saída para aplicações robóticas.
"""

import numpy as np
from typing import Dict, Tuple
from dataclasses import dataclass
import logging


@dataclass
class PIDConfig:
    """Configuração de parâmetros PID."""
    kp: float
    ki: float
    kd: float
    integral_max: float = 10.0
    integral_min: float = -10.0
    max_output: float = 50.0


class PIDController:
    """
    Controlador PID com anti-windup e saturação de saída.
    
    Implementa a equação:
    u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
    
    Attributes:
        config: Configuração dos parâmetros PID
        previous_error: Erro no passo anterior
        integral: Acumulador da integral do erro
    """
    
    def __init__(self, config: PIDConfig):
        """
        Inicializa o controlador PID.
        
        Args:
            config: Objeto PIDConfig com os parâmetros
        """
        self.config = config
        self.previous_error = 0.0
        self.integral = 0.0
        self.logger = logging.getLogger(__name__)
        
    def update(self, error: float, dt: float) -> float:
        """
        Calcula o sinal de controle baseado no erro.
        
        Args:
            error: Erro atual (referência - valor atual)
            dt: Intervalo de tempo desde o último update (segundos)
            
        Returns:
            Sinal de controle saturado em [-max_output, max_output]
        """
        if dt <= 0:
            self.logger.warning("Intervalo de tempo inválido: %f", dt)
            return 0.0
        
        # Termo Proporcional
        p_term = self.config.kp * error
        
        # Termo Integral com anti-windup
        self.integral += error * dt
        self.integral = np.clip(
            self.integral,
            self.config.integral_min / self.config.ki if self.config.ki != 0 else self.config.integral_min,
            self.config.integral_max / self.config.ki if self.config.ki != 0 else self.config.integral_max
        )
        i_term = self.config.ki * self.integral
        
        # Termo Derivativo
        d_term = self.config.kd * (error - self.previous_error) / dt
        
        # Sinal de controle
        output = p_term + i_term + d_term
        
        # Saturação de saída
        output_saturated = np.clip(output, -self.config.max_output, self.config.max_output)
        
        # Atualizar erro anterior
        self.previous_error = error
        
        return float(output_saturated)
    
    def reset(self) -> None:
        """Reseta o controlador para seu estado inicial."""
        self.previous_error = 0.0
        self.integral = 0.0
        self.logger.debug("Controlador PID resetado")
    
    def get_state(self) -> Dict[str, float]:
        """
        Retorna o estado interno do controlador.
        
        Returns:
            Dicionário com os valores internos do PID
        """
        return {
            "kp": self.config.kp,
            "ki": self.config.ki,
            "kd": self.config.kd,
            "previous_error": self.previous_error,
            "integral": self.integral,
            "max_output": self.config.max_output
        }


class DualJointPIDController:
    """
    Controlador PID para ambas as juntas do manipulador.
    
    Gerencia dois controladores PID independentes, um para cada junta
    do manipulador planar.
    """
    
    def __init__(self, config_j1: PIDConfig, config_j2: PIDConfig):
        """
        Inicializa os controladores para ambas as juntas.
        
        Args:
            config_j1: Configuração PID para junta 1
            config_j2: Configuração PID para junta 2
        """
        self.joint1_controller = PIDController(config_j1)
        self.joint2_controller = PIDController(config_j2)
        self.logger = logging.getLogger(__name__)
        
    def update(self, 
               errors: Tuple[float, float], 
               dt: float) -> Tuple[float, float]:
        """
        Atualiza ambos os controladores.
        
        Args:
            errors: Tupla (erro_junta1, erro_junta2) em radianos
            dt: Intervalo de tempo (segundos)
            
        Returns:
            Tupla (torque_junta1, torque_junta2) em N.m
        """
        torque_j1 = self.joint1_controller.update(errors[0], dt)
        torque_j2 = self.joint2_controller.update(errors[1], dt)
        
        return torque_j1, torque_j2
    
    def reset(self) -> None:
        """Reseta ambos os controladores."""
        self.joint1_controller.reset()
        self.joint2_controller.reset()
        self.logger.debug("Ambos os controladores PID foram resetados")
    
    def get_state(self) -> Dict[str, Dict]:
        """
        Retorna o estado de ambos os controladores.
        
        Returns:
            Dicionário aninhado com estado de cada junta
        """
        return {
            "joint1": self.joint1_controller.get_state(),
            "joint2": self.joint2_controller.get_state()
        }
    
    def update_gains(self, 
                    joint: int, 
                    kp: float = None, 
                    ki: float = None, 
                    kd: float = None) -> None:
        """
        Atualiza ganhos PID em tempo real.
        
        Args:
            joint: Número da junta (1 ou 2)
            kp: Ganho proporcional (opcional)
            ki: Ganho integral (opcional)
            kd: Ganho derivativo (opcional)
        """
        controller = self.joint1_controller if joint == 1 else self.joint2_controller
        
        if kp is not None:
            controller.config.kp = kp
        if ki is not None:
            controller.config.ki = ki
        if kd is not None:
            controller.config.kd = kd
            
        self.logger.info(f"Ganhos PID da junta {joint} atualizados: "
                        f"Kp={controller.config.kp}, "
                        f"Ki={controller.config.ki}, "
                        f"Kd={controller.config.kd}")
