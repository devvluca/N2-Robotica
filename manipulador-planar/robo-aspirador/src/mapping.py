"""
Sistema de Mapeamento e Memória.
Grid de ocupação 2D + Log de trajetória + Persistência.
"""

import numpy as np
import json
import os
from datetime import datetime


# Estados das células
CELL_UNKNOWN = 0    # Não explorado
CELL_FREE = 1       # Livre e limpo
CELL_DIRTY = 2      # Livre mas sujo
CELL_OBSTACLE = 3   # Obstáculo detectado


class OccupancyMap:
    """Mapa de ocupação 2D para o robô aspirador."""
    
    def __init__(self, width, height, cell_size=0.1):
        self.width = width
        self.height = height
        self.cell_size = cell_size
        
        self.grid_width = int(width / cell_size)
        self.grid_height = int(height / cell_size)
        
        # Grid de ocupação
        self.grid = np.zeros((self.grid_width, self.grid_height), dtype=np.int8)
        
        # Grid de tempo (última visita)
        self.time_grid = np.zeros((self.grid_width, self.grid_height), dtype=np.float32)
        
        # Grid de contagem de visitas
        self.visit_count = np.zeros((self.grid_width, self.grid_height), dtype=np.int32)
        
        # Trajetória
        self.trajectory = []
        
        # Estatísticas
        self.total_time = 0.0
        self.collisions = 0
        
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
    
    def update_from_sensors(self, robot_pos, robot_angle, sensor_readings, sensor_angles, sensor_range):
        """Atualiza o mapa com base nas leituras dos sensores."""
        import math
        
        for reading, sensor_angle_deg in zip(sensor_readings, sensor_angles):
            sensor_angle_rad = math.radians(sensor_angle_deg)
            total_angle = robot_angle + sensor_angle_rad
            
            distance = reading * sensor_range
            
            # Ray marching - marcar células livres
            num_steps = int(distance / self.cell_size)
            for step in range(num_steps):
                ray_dist = step * self.cell_size
                ray_x = robot_pos[0] + ray_dist * math.cos(total_angle)
                ray_y = robot_pos[1] + ray_dist * math.sin(total_angle)
                
                gx, gy = self.world_to_grid(ray_x, ray_y)
                if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                    if self.grid[gx, gy] != CELL_OBSTACLE:
                        self.grid[gx, gy] = CELL_FREE
            
            # Se detectou obstáculo
            if reading < 0.95:
                hit_x = robot_pos[0] + distance * math.cos(total_angle)
                hit_y = robot_pos[1] + distance * math.sin(total_angle)
                gx, gy = self.world_to_grid(hit_x, hit_y)
                if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                    self.grid[gx, gy] = CELL_OBSTACLE
    
    def mark_cleaned(self, x, y, timestamp, robot_radius=0.16):
        """Marca células como limpas na posição do robô."""
        cells_radius = int(robot_radius / self.cell_size) + 1
        center_gx, center_gy = self.world_to_grid(x, y)
        
        for dx in range(-cells_radius, cells_radius + 1):
            for dy in range(-cells_radius, cells_radius + 1):
                gx = center_gx + dx
                gy = center_gy + dy
                
                if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                    cell_x, cell_y = self.grid_to_world(gx, gy)
                    dist = np.sqrt((cell_x - x)**2 + (cell_y - y)**2)
                    
                    if dist <= robot_radius and self.grid[gx, gy] != CELL_OBSTACLE:
                        self.grid[gx, gy] = CELL_FREE
                        self.time_grid[gx, gy] = timestamp
                        self.visit_count[gx, gy] += 1
    
    def record_trajectory(self, x, y, theta, timestamp):
        """Registra ponto na trajetória."""
        self.trajectory.append({
            'x': float(x),
            'y': float(y),
            'theta': float(theta),
            'time': float(timestamp)
        })
    
    def get_coverage_stats(self):
        """Calcula estatísticas de cobertura."""
        total_cells = self.grid_width * self.grid_height
        obstacle_cells = np.sum(self.grid == CELL_OBSTACLE)
        free_cells = np.sum(self.grid == CELL_FREE)
        unknown_cells = np.sum(self.grid == CELL_UNKNOWN)
        
        cleanable_cells = total_cells - obstacle_cells
        if cleanable_cells > 0:
            coverage_percent = (free_cells / cleanable_cells) * 100
        else:
            coverage_percent = 0
        
        covered_area = free_cells * (self.cell_size ** 2)
        total_area = cleanable_cells * (self.cell_size ** 2)
        
        total_visits = np.sum(self.visit_count)
        if free_cells > 0:
            avg_visits = total_visits / free_cells
            efficiency = 1.0 / avg_visits if avg_visits > 0 else 0
        else:
            efficiency = 0
        
        return {
            'total_cells': int(total_cells),
            'obstacle_cells': int(obstacle_cells),
            'free_cells': int(free_cells),
            'unknown_cells': int(unknown_cells),
            'coverage_percent': round(coverage_percent, 2),
            'covered_area_m2': round(covered_area, 3),
            'total_cleanable_area_m2': round(total_area, 3),
            'total_visits': int(total_visits),
            'efficiency': round(efficiency, 4)
        }
    
    def get_nearest_uncleaned(self, robot_x, robot_y):
        """Encontra a célula não limpa mais próxima."""
        robot_gx, robot_gy = self.world_to_grid(robot_x, robot_y)
        
        min_dist = float('inf')
        nearest = None
        
        for gx in range(self.grid_width):
            for gy in range(self.grid_height):
                if self.grid[gx, gy] == CELL_UNKNOWN:
                    dist = abs(gx - robot_gx) + abs(gy - robot_gy)
                    if dist < min_dist:
                        min_dist = dist
                        nearest = (gx, gy)
        
        if nearest:
            return self.grid_to_world(nearest[0], nearest[1])
        return None
    
    def get_least_visited_direction(self, robot_x, robot_y, robot_angle, look_ahead=0.5):
        """Determina a direção com menos células visitadas."""
        import math
        
        directions = [0, math.pi/4, math.pi/2, 3*math.pi/4, 
                     math.pi, -3*math.pi/4, -math.pi/2, -math.pi/4]
        
        best_direction = 0
        min_visits = float('inf')
        
        for direction in directions:
            check_angle = robot_angle + direction
            check_x = robot_x + look_ahead * math.cos(check_angle)
            check_y = robot_y + look_ahead * math.sin(check_angle)
            
            gx, gy = self.world_to_grid(check_x, check_y)
            if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                if self.grid[gx, gy] != CELL_OBSTACLE:
                    visits = self.visit_count[gx, gy]
                    if visits < min_visits:
                        min_visits = visits
                        best_direction = direction
        
        return best_direction
    
    def save(self, filepath):
        """Salva o mapa em arquivo JSON."""
        data = {
            'width': self.width,
            'height': self.height,
            'cell_size': self.cell_size,
            'grid': self.grid.tolist(),
            'time_grid': self.time_grid.tolist(),
            'visit_count': self.visit_count.tolist(),
            'trajectory': self.trajectory,
            'total_time': self.total_time,
            'collisions': self.collisions,
            'timestamp': datetime.now().isoformat()
        }
        
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        with open(filepath, 'w') as f:
            json.dump(data, f)
        
        print(f"[MAPA] Salvo em {filepath}")
    
    def load(self, filepath):
        """Carrega mapa de arquivo JSON."""
        if not os.path.exists(filepath):
            print(f"[MAPA] Arquivo não encontrado: {filepath}")
            return False
        
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            self.grid = np.array(data['grid'], dtype=np.int8)
            self.time_grid = np.array(data['time_grid'], dtype=np.float32)
            self.visit_count = np.array(data['visit_count'], dtype=np.int32)
            self.trajectory = data.get('trajectory', [])
            self.total_time = data.get('total_time', 0)
            self.collisions = data.get('collisions', 0)
            
            print(f"[MAPA] Carregado de {filepath}")
            return True
        except Exception as e:
            print(f"[MAPA] Erro ao carregar: {e}")
            return False
