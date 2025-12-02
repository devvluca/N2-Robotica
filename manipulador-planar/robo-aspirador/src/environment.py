"""
Ambiente de simulação: planta de casa com paredes e obstáculos.
"""

import pybullet as p
import numpy as np


class Environment:
    """Ambiente de casa para o robô aspirador."""
    
    def __init__(self, physics_client, width=2.5, height=2.5, cell_size=0.05):
        """
        Args:
            physics_client: Cliente PyBullet
            width: Largura do ambiente em metros (2.5m)
            height: Altura do ambiente em metros (2.5m)
            cell_size: Tamanho de cada célula do grid (para mapeamento)
        """
        self.client = physics_client
        self.width = width
        self.height = height
        self.cell_size = cell_size
        
        # Grid dimensions
        self.grid_width = int(width / cell_size)
        self.grid_height = int(height / cell_size)
        
        # Listas de objetos
        self.walls = []
        self.obstacles = []
        self.obstacle_positions = []
        
        # Criar ambiente
        self._create_floor()
        self._create_walls()
        self._create_furniture()
        
        print(f"[AMBIENTE] Criado: {width}x{height}m, grid {self.grid_width}x{self.grid_height}")
        
    def _create_floor(self):
        """Cria o piso."""
        floor_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[self.width/2, self.height/2, 0.01]
        )
        floor_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[self.width/2, self.height/2, 0.01],
            rgbaColor=[0.95, 0.93, 0.88, 1]  # Cor de piso claro
        )
        
        self.floor_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=floor_collision,
            baseVisualShapeIndex=floor_visual,
            basePosition=[self.width/2, self.height/2, -0.01]
        )
        
        # Criar linhas de grid
        self._create_grid_lines()
    
    def _create_grid_lines(self):
        """Cria linhas de grid no piso."""
        # Linhas a cada 0.5 metro
        for i in np.arange(0, self.width + 0.1, 0.5):
            p.addUserDebugLine(
                [i, 0, 0.001],
                [i, self.height, 0.001],
                [0.8, 0.8, 0.8],
                lineWidth=1
            )
        
        for i in np.arange(0, self.height + 0.1, 0.5):
            p.addUserDebugLine(
                [0, i, 0.001],
                [self.width, i, 0.001],
                [0.8, 0.8, 0.8],
                lineWidth=1
            )
    
    def _create_walls(self):
        """Cria as paredes do ambiente."""
        wall_height = 0.2
        wall_thickness = 0.03
        
        # Paredes externas - mais finas e baixas
        walls_config = [
            # Sul
            ([self.width/2, wall_thickness/2, wall_height/2], 
             [self.width/2 + 0.05, wall_thickness/2, wall_height/2]),
            # Norte
            ([self.width/2, self.height - wall_thickness/2, wall_height/2], 
             [self.width/2 + 0.05, wall_thickness/2, wall_height/2]),
            # Oeste
            ([wall_thickness/2, self.height/2, wall_height/2], 
             [wall_thickness/2, self.height/2 + 0.05, wall_height/2]),
            # Leste
            ([self.width - wall_thickness/2, self.height/2, wall_height/2], 
             [wall_thickness/2, self.height/2 + 0.05, wall_height/2]),
        ]
        
        for pos, size in walls_config:
            collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
            visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=size,
                rgbaColor=[0.7, 0.65, 0.6, 1]
            )
            wall_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=collision,
                baseVisualShapeIndex=visual,
                basePosition=pos
            )
            self.walls.append(wall_id)
    
    def _create_furniture(self):
        """Cria móveis/obstáculos - 3 obstáculos bem espaçados."""
        furniture_list = [
            # Mesa de centro (centro do mapa)
            {'pos': [1.25, 1.25], 'size': [0.12, 0.12, 0.05], 'color': [0.55, 0.4, 0.2, 1]},
            # Cadeira (canto superior direito, mas não na borda)
            {'pos': [1.9, 1.85], 'size': [0.1, 0.1, 0.06], 'color': [0.4, 0.3, 0.5, 1]},
            # Vaso/objeto (canto inferior direito, mas não na borda)
            {'pos': [1.8, 0.6], 'size': [0.08, 0.08, 0.08], 'color': [0.3, 0.5, 0.35, 1]},
        ]
        # 3 obstáculos bem distribuídos - corredores amplos entre eles
        
        for furniture in furniture_list:
            pos = furniture['pos']
            size = furniture['size']
            color = furniture['color']
            
            collision = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=size
            )
            visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=size,
                rgbaColor=color
            )
            
            obj_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=collision,
                baseVisualShapeIndex=visual,
                basePosition=[pos[0], pos[1], size[2]]
            )
            self.obstacles.append(obj_id)
            self.obstacle_positions.append((pos[0], pos[1], size[0], size[1]))
    
    def world_to_grid(self, x, y):
        """Converte coordenadas do mundo para índices do grid."""
        gx = int(x / self.cell_size)
        gy = int(y / self.cell_size)
        gx = np.clip(gx, 0, self.grid_width - 1)
        gy = np.clip(gy, 0, self.grid_height - 1)
        return gx, gy
    
    def grid_to_world(self, gx, gy):
        """Converte índices do grid para coordenadas do mundo."""
        x = (gx + 0.5) * self.cell_size
        y = (gy + 0.5) * self.cell_size
        return x, y
    
    def is_valid_position(self, x, y, margin=0.2):
        """Verifica se uma posição é válida."""
        if x < margin or x > self.width - margin:
            return False
        if y < margin or y > self.height - margin:
            return False
        
        for obs_x, obs_y, half_w, half_h in self.obstacle_positions:
            if (abs(x - obs_x) < half_w + margin and 
                abs(y - obs_y) < half_h + margin):
                return False
        
        return True
    
    def get_dimensions(self):
        """Retorna dimensões do ambiente."""
        return {
            'width': self.width,
            'height': self.height,
            'cell_size': self.cell_size,
            'grid_width': self.grid_width,
            'grid_height': self.grid_height
        }
