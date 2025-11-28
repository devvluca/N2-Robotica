"""
Configuração do ambiente físico do robô aspirador.

Carrega obstáculos do arquivo de configuração e cria corpos
no ambiente PyBullet.
"""

import pybullet as p
import numpy as np
import logging
from typing import Dict, List, Tuple, Any
from dataclasses import dataclass


@dataclass
class Obstacle:
    """Estrutura para representar um obstáculo."""
    name: str
    x: float
    y: float
    width: float
    height: float
    mass: float = 0.0  # 0 = estático


class EnvironmentSetup:
    """
    Gerenciador do ambiente PyBullet para o robô aspirador.
    
    Responsabilidades:
    - Carregar obstáculos da configuração
    - Criar paredes e móveis no ambiente
    - Gerenciar colisões e dinâmica
    - Fornecer informações de ocupação
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Inicializa setup do ambiente.
        
        Args:
            config: Dicionário de configuração YAML
        """
        self.config = config
        self.logger = logging.getLogger(__name__)
        
        self.obstacle_bodies = {}
        self.ground_id = None
        self.created_obstacles = []
        
        # Dimensões do ambiente
        env_cfg = config.get('environment', {})
        self.world_width = env_cfg.get('world_width', 10.0)
        self.world_height = env_cfg.get('world_height', 8.0)
        self.wall_thickness = env_cfg.get('wall_thickness', 0.1)
        self.wall_height = env_cfg.get('wall_height', 0.5)
    
    def setup_world(self) -> None:
        """Configura o piso e as paredes do ambiente."""
        # Criar piso
        self._create_ground()
        
        # Criar paredes ao redor
        self._create_walls()
        
        self.logger.info("Ambiente configurado com sucesso")
    
    def _create_ground(self) -> None:
        """Cria o piso do ambiente."""
        # Criar forma plana para o piso
        floor_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[self.world_width / 2, self.world_height / 2, 0.01]
        )
        
        # Criar corpo do piso
        self.ground_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=floor_shape,
            basePosition=[self.world_width / 2, self.world_height / 2, 0],
            baseOrientation=[0, 0, 0, 1]
        )
        
        # Propriedades de superfície
        p.changeDynamics(
            self.ground_id,
            -1,
            lateralFriction=0.8,
            restitution=0.1
        )
        
        self.logger.info(f"Piso criado: {self.world_width}m × {self.world_height}m")
    
    def _create_walls(self) -> None:
        """Cria as paredes ao redor do ambiente."""
        wall_height = self.wall_height
        thickness = self.wall_thickness
        
        # Parede frontal (y=0)
        self._create_wall(
            "wall_front",
            self.world_width / 2,
            -thickness / 2,
            self.world_width,
            thickness,
            wall_height
        )
        
        # Parede traseira (y=world_height)
        self._create_wall(
            "wall_back",
            self.world_width / 2,
            self.world_height + thickness / 2,
            self.world_width,
            thickness,
            wall_height
        )
        
        # Parede esquerda (x=0)
        self._create_wall(
            "wall_left",
            -thickness / 2,
            self.world_height / 2,
            thickness,
            self.world_height,
            wall_height
        )
        
        # Parede direita (x=world_width)
        self._create_wall(
            "wall_right",
            self.world_width + thickness / 2,
            self.world_height / 2,
            thickness,
            self.world_height,
            wall_height
        )
    
    def _create_wall(self,
                    name: str,
                    x: float, y: float,
                    width: float, depth: float,
                    height: float) -> int:
        """
        Cria uma parede individual.
        
        Args:
            name: Nome da parede
            x, y: Posição em metros
            width, depth: Dimensões horizontais
            height: Altura da parede
            
        Returns:
            ID do corpo PyBullet
        """
        # Criar forma de colisão
        shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[width / 2, depth / 2, height / 2]
        )
        
        # Criar corpo
        body_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=shape,
            basePosition=[x, y, height / 2],
            baseOrientation=[0, 0, 0, 1]
        )
        
        # Propriedades de superfície
        p.changeDynamics(body_id, -1, lateralFriction=0.9)
        
        self.obstacle_bodies[name] = body_id
        self.created_obstacles.append(name)
        
        self.logger.debug(f"Parede criada: {name} em ({x}, {y})")
        return body_id
    
    def load_obstacles(self) -> None:
        """Carrega obstáculos da configuração."""
        env_cfg = self.config.get('environment', {})
        obstacles_list = env_cfg.get('obstacles', [])
        
        if not obstacles_list:
            self.logger.warning("Nenhum obstáculo definido na configuração")
            return
        
        for obs_cfg in obstacles_list:
            obstacle = Obstacle(
                name=obs_cfg.get('name', 'obstacle'),
                x=obs_cfg.get('x', 0.0),
                y=obs_cfg.get('y', 0.0),
                width=obs_cfg.get('width', 0.5),
                height=obs_cfg.get('height', 0.5),
                mass=obs_cfg.get('mass', 0.0)
            )
            self.create_obstacle(obstacle)
    
    def create_obstacle(self, obstacle: Obstacle) -> int:
        """
        Cria um obstáculo no ambiente.
        
        Args:
            obstacle: Objeto Obstacle com propriedades
            
        Returns:
            ID do corpo PyBullet
        """
        # Validar posição dentro do mapa
        if not self._is_valid_position(obstacle.x, obstacle.y, obstacle.width, obstacle.height):
            self.logger.warning(f"Posição inválida para obstáculo {obstacle.name}")
            return -1
        
        # Criar forma de colisão
        shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[obstacle.width / 2, obstacle.height / 2, 0.25]
        )
        
        # Criar corpo
        body_id = p.createMultiBody(
            baseMass=obstacle.mass,
            baseCollisionShapeIndex=shape,
            basePosition=[obstacle.x, obstacle.y, 0.25],
            baseOrientation=[0, 0, 0, 1]
        )
        
        # Propriedades de superfície
        p.changeDynamics(
            body_id,
            -1,
            lateralFriction=0.8,
            restitution=0.2
        )
        
        self.obstacle_bodies[obstacle.name] = body_id
        self.created_obstacles.append(obstacle.name)
        
        self.logger.info(f"Obstáculo criado: {obstacle.name} em ({obstacle.x}, {obstacle.y})")
        return body_id
    
    def _is_valid_position(self,
                          x: float, y: float,
                          width: float, height: float) -> bool:
        """
        Verifica se a posição é válida dentro do mapa.
        
        Args:
            x, y: Posição do centro
            width, height: Dimensões do obstáculo
            
        Returns:
            True se dentro dos limites
        """
        margin = 0.2
        
        if x - width/2 < margin or x + width/2 > self.world_width - margin:
            return False
        if y - height/2 < margin or y + height/2 > self.world_height - margin:
            return False
        
        return True
    
    def get_obstacle_positions(self) -> Dict[str, Tuple[float, float, float, float]]:
        """
        Retorna posições dos obstáculos para mapeamento.
        
        Returns:
            Dicionário {nome: (x, y, width, height)}
        """
        positions = {}
        
        for name, body_id in self.obstacle_bodies.items():
            try:
                pos, _ = p.getBasePositionAndOrientation(body_id)
                aabb = p.getAABB(body_id)
                
                x = pos[0]
                y = pos[1]
                width = aabb[1][0] - aabb[0][0]
                height = aabb[1][1] - aabb[0][1]
                
                positions[name] = (x, y, width, height)
            
            except Exception as e:
                self.logger.error(f"Erro ao obter posição de {name}: {e}")
        
        return positions
    
    def get_occupancy_grid(self, grid_resolution: float) -> np.ndarray:
        """
        Gera grade de ocupação baseada nos obstáculos.
        
        Args:
            grid_resolution: Resolução da grade em metros
            
        Returns:
            Array NumPy com valores de ocupação [0, 1]
        """
        # Dimensões da grade
        grid_width = int(self.world_width / grid_resolution)
        grid_height = int(self.world_height / grid_resolution)
        
        # Inicializar grade vazia
        grid = np.zeros((grid_width, grid_height), dtype=np.float32)
        
        # Marcar obstáculos
        for obstacle_name, body_id in self.obstacle_bodies.items():
            try:
                pos, _ = p.getBasePositionAndOrientation(body_id)
                aabb = p.getAABB(body_id)
                
                # Converter posição para índices de grade
                x_min_idx = int((aabb[0][0]) / grid_resolution)
                x_max_idx = int((aabb[1][0]) / grid_resolution)
                y_min_idx = int((aabb[0][1]) / grid_resolution)
                y_max_idx = int((aabb[1][1]) / grid_resolution)
                
                # Limitar aos limites da grade
                x_min_idx = max(0, x_min_idx)
                x_max_idx = min(grid_width, x_max_idx)
                y_min_idx = max(0, y_min_idx)
                y_max_idx = min(grid_height, y_max_idx)
                
                # Marcar células como ocupadas
                grid[x_min_idx:x_max_idx, y_min_idx:y_max_idx] = 1.0
            
            except Exception as e:
                self.logger.debug(f"Erro ao processar obstáculo {obstacle_name}: {e}")
        
        return grid
    
    def get_summary(self) -> Dict[str, Any]:
        """
        Retorna resumo do ambiente.
        
        Returns:
            Dicionário com informações do ambiente
        """
        return {
            'world_size': (self.world_width, self.world_height),
            'wall_thickness': self.wall_thickness,
            'wall_height': self.wall_height,
            'total_obstacles': len(self.created_obstacles),
            'obstacle_names': self.created_obstacles,
            'ground_id': self.ground_id
        }
    
    def cleanup(self) -> None:
        """Remove todos os corpos criados."""
        for body_id in self.obstacle_bodies.values():
            try:
                p.removeBody(body_id)
            except Exception as e:
                self.logger.debug(f"Erro ao remover corpo: {e}")
        
        if self.ground_id is not None:
            try:
                p.removeBody(self.ground_id)
            except Exception as e:
                self.logger.debug(f"Erro ao remover piso: {e}")
        
        self.obstacle_bodies.clear()
        self.created_obstacles.clear()
        self.logger.info("Ambiente limpo")
