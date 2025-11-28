"""
Módulo de Mapeamento 2D para Robô Aspirador.

Gerencia a construção dinâmica de um mapa de ocupação 2D,
rastreamento de cobertura e persistência de dados.
"""

import numpy as np
import json
import logging
from typing import Dict, Tuple, List, Optional
from pathlib import Path
from datetime import datetime
import pickle


class OccupancyGrid:
    """
    Grade de ocupação 2D com suporte a cobertura e rastreamento.
    
    Atributos:
        grid: Matriz 2D com valores de ocupação [0, 1]
        coverage: Matriz 2D com cobertura [0, 1]
        resolution: Tamanho de cada célula em metros
    """
    
    def __init__(self, 
                 width: float, 
                 height: float, 
                 resolution: float = 0.1):
        """
        Inicializa a grade de ocupação.
        
        Args:
            width: Largura do ambiente em metros
            height: Altura do ambiente em metros
            resolution: Tamanho de cada célula em metros
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        
        # Dimensões da grid
        self.grid_width = int(np.ceil(width / resolution))
        self.grid_height = int(np.ceil(height / resolution))
        
        # Matrizes
        self.occupancy_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.float32)
        self.coverage_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.float32)
        self.visit_count = np.zeros((self.grid_height, self.grid_width), dtype=np.int32)
        
        self.logger = logging.getLogger(__name__)
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """
        Converte coordenada mundial para índice da grid.
        
        Args:
            x: Coordenada x em metros
            y: Coordenada y em metros
            
        Returns:
            Tupla (grid_x, grid_y) ou None se fora da grid
        """
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)
        
        # Verificar limites
        if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
            return grid_x, grid_y
        
        return None
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """
        Converte índice da grid para coordenada mundial.
        
        Args:
            grid_x: Índice x da grid
            grid_y: Índice y da grid
            
        Returns:
            Tupla (x, y) em metros (centro da célula)
        """
        x = (grid_x + 0.5) * self.resolution
        y = (grid_y + 0.5) * self.resolution
        
        return x, y
    
    def set_occupancy(self, x: float, y: float, occupied: bool, radius: float = 0.1) -> None:
        """
        Define ocupação em uma região.
        
        Args:
            x: Coordenada x em metros
            y: Coordenada y em metros
            occupied: True se ocupado, False se livre
            radius: Raio de efeito em metros
        """
        grid_pos = self.world_to_grid(x, y)
        if grid_pos is None:
            return
        
        grid_x, grid_y = grid_pos
        radius_cells = int(np.ceil(radius / self.resolution))
        
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                nx, ny = grid_x + dx, grid_y + dy
                
                if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                    if occupied:
                        self.occupancy_grid[ny, nx] = 1.0
                    else:
                        self.occupancy_grid[ny, nx] = 0.0
    
    def add_coverage(self, 
                    x: float, 
                    y: float, 
                    amount: float = 1.0,
                    width: float = 0.3) -> None:
        """
        Registra cobertura em uma região (simulando limpeza).
        
        Args:
            x: Coordenada x em metros
            y: Coordenada y em metros
            amount: Quantidade de cobertura a adicionar [0, 1]
            width: Largura da área de limpeza em metros
        """
        grid_pos = self.world_to_grid(x, y)
        if grid_pos is None:
            return
        
        grid_x, grid_y = grid_pos
        width_cells = int(np.ceil(width / self.resolution))
        
        for dx in range(-width_cells, width_cells + 1):
            nx = grid_x + dx
            
            if 0 <= nx < self.grid_width and 0 <= grid_y < self.grid_height:
                # Não limpar sobre obstáculos
                if self.occupancy_grid[grid_y, nx] < 0.5:
                    self.coverage_grid[grid_y, nx] = min(
                        1.0,
                        self.coverage_grid[grid_y, nx] + amount
                    )
                    self.visit_count[grid_y, nx] += 1
    
    def get_coverage_percentage(self) -> float:
        """
        Calcula percentual de cobertura do ambiente.
        
        Returns:
            Percentual de célula limpas [0, 100]
        """
        # Contar apenas células livres (não ocupadas)
        free_cells = np.sum(self.occupancy_grid < 0.5)
        
        if free_cells == 0:
            return 0.0
        
        covered_cells = np.sum((self.coverage_grid > 0) & (self.occupancy_grid < 0.5))
        
        return (covered_cells / free_cells) * 100.0
    
    def get_uncovered_cells(self, threshold: float = 0.5) -> List[Tuple[int, int]]:
        """
        Retorna células não cobertas acima do threshold.
        
        Args:
            threshold: Valor mínimo de cobertura para considerar "coberta"
            
        Returns:
            Lista de (grid_x, grid_y) não cobertas
        """
        uncovered = []
        
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                if (self.occupancy_grid[y, x] < 0.5 and 
                    self.coverage_grid[y, x] < threshold):
                    uncovered.append((x, y))
        
        return uncovered
    
    def get_info(self) -> Dict:
        """
        Retorna informações da grid.
        
        Returns:
            Dicionário com estatísticas
        """
        free_cells = np.sum(self.occupancy_grid < 0.5)
        covered_cells = np.sum((self.coverage_grid > 0) & (self.occupancy_grid < 0.5))
        
        return {
            'grid_width': self.grid_width,
            'grid_height': self.grid_height,
            'resolution': self.resolution,
            'total_free_area': float(free_cells * self.resolution**2),
            'covered_area': float(covered_cells * self.resolution**2),
            'coverage_percentage': self.get_coverage_percentage(),
            'uncovered_cells': len(self.get_uncovered_cells()),
            'mean_revisits': float(np.mean(self.visit_count[self.occupancy_grid < 0.5]))
        }
    
    def save(self, filepath: str) -> bool:
        """
        Salva o mapa em arquivo.
        
        Args:
            filepath: Caminho para arquivo .pkl
            
        Returns:
            True se salvo com sucesso
        """
        try:
            data = {
                'occupancy_grid': self.occupancy_grid,
                'coverage_grid': self.coverage_grid,
                'visit_count': self.visit_count,
                'width': self.width,
                'height': self.height,
                'resolution': self.resolution,
                'timestamp': datetime.now().isoformat()
            }
            
            Path(filepath).parent.mkdir(parents=True, exist_ok=True)
            
            with open(filepath, 'wb') as f:
                pickle.dump(data, f)
            
            self.logger.info(f"Mapa salvo em {filepath}")
            return True
        
        except Exception as e:
            self.logger.error(f"Erro ao salvar mapa: {e}")
            return False
    
    def load(self, filepath: str) -> bool:
        """
        Carrega mapa de arquivo.
        
        Args:
            filepath: Caminho para arquivo .pkl
            
        Returns:
            True se carregado com sucesso
        """
        try:
            with open(filepath, 'rb') as f:
                data = pickle.load(f)
            
            self.occupancy_grid = data['occupancy_grid']
            self.coverage_grid = data['coverage_grid']
            self.visit_count = data['visit_count']
            self.width = data['width']
            self.height = data['height']
            self.resolution = data['resolution']
            
            self.logger.info(f"Mapa carregado de {filepath}")
            return True
        
        except Exception as e:
            self.logger.error(f"Erro ao carregar mapa: {e}")
            return False


class MappingModule:
    """
    Módulo de mapeamento para o robô aspirador.
    
    Gerencia a construção e atualização do mapa de ocupação,
    processamento de dados de sensores e análise de cobertura.
    """
    
    def __init__(self, config: Dict):
        """
        Inicializa módulo de mapeamento.
        
        Args:
            config: Dicionário com configurações
        """
        self.config = config
        self.logger = logging.getLogger(__name__)
        
        # Criar grid de ocupação
        self.occupancy_grid = OccupancyGrid(
            width=config['mapping']['map_width'],
            height=config['mapping']['map_height'],
            resolution=config['mapping']['grid_resolution']
        )
        
        # Rastreamento de exploração
        self.exploration_history = []
        self.coverage_history = []
        
    def process_sensor_data(self, 
                           robot_x: float, 
                           robot_y: float,
                           robot_heading: float,
                           sensor_readings: List[float],
                           sensor_angles: List[float]) -> None:
        """
        Processa leituras de sensores e atualiza o mapa.
        
        Args:
            robot_x: Posição x do robô em metros
            robot_y: Posição y do robô em metros
            robot_heading: Orientação do robô em radianos
            sensor_readings: Distâncias medidas pelos sensores em metros
            sensor_angles: Ângulos dos sensores em graus (relativos)
        """
        sensor_range = self.config['robot']['ultrasonic_sensors']['range']
        
        for reading, angle in zip(sensor_readings, sensor_angles):
            # Converter ângulo para absoluto
            sensor_angle_abs = np.radians(angle) + robot_heading
            
            # Calcular posição de impacto
            if reading < sensor_range * 0.99:  # Detectou obstáculo
                impact_x = robot_x + reading * np.cos(sensor_angle_abs)
                impact_y = robot_y + reading * np.sin(sensor_angle_abs)
                
                # Marcar como ocupado
                self.occupancy_grid.set_occupancy(impact_x, impact_y, True)
            
            # Marcar caminho como livre (ray casting)
            num_steps = int(reading / 0.05)  # 5cm por passo
            for step in range(num_steps):
                dist = step * 0.05
                x = robot_x + dist * np.cos(sensor_angle_abs)
                y = robot_y + dist * np.sin(sensor_angle_abs)
                
                self.occupancy_grid.set_occupancy(x, y, False, radius=0.02)
    
    def update_coverage(self, 
                       robot_x: float, 
                       robot_y: float,
                       cleaner_width: float = 0.3) -> None:
        """
        Registra limpeza em torno da posição do robô.
        
        Args:
            robot_x: Posição x do robô
            robot_y: Posição y do robô
            cleaner_width: Largura do aspirador em metros
        """
        self.occupancy_grid.add_coverage(robot_x, robot_y, amount=0.1, width=cleaner_width)
        
        # Registrar histórico
        self.coverage_history.append({
            'timestamp': datetime.now().isoformat(),
            'coverage_percentage': self.occupancy_grid.get_coverage_percentage(),
            'uncovered_cells': len(self.occupancy_grid.get_uncovered_cells())
        })
    
    def get_coverage_percentage(self) -> float:
        """Retorna percentual de cobertura."""
        return self.occupancy_grid.get_coverage_percentage()
    
    def get_uncovered_priority_cell(self) -> Optional[Tuple[float, float]]:
        """
        Retorna célula não coberta de maior prioridade.
        
        Returns:
            Tupla (x, y) em metros ou None
        """
        uncovered = self.occupancy_grid.get_uncovered_cells(
            threshold=self.config['mapping']['coverage_threshold']
        )
        
        if not uncovered:
            return None
        
        # Selecionar célula menos visitada
        min_visits = float('inf')
        best_cell = uncovered[0]
        
        for grid_x, grid_y in uncovered:
            visits = self.occupancy_grid.visit_count[grid_y, grid_x]
            if visits < min_visits:
                min_visits = visits
                best_cell = (grid_x, grid_y)
        
        # Converter para mundo
        x, y = self.occupancy_grid.grid_to_world(*best_cell)
        return x, y
    
    def get_map_image(self) -> np.ndarray:
        """
        Retorna visualização do mapa como imagem RGB.
        
        Returns:
            Imagem numpy array (H, W, 3)
        """
        h, w = self.occupancy_grid.grid_height, self.occupancy_grid.grid_width
        image = np.zeros((h, w, 3), dtype=np.uint8)
        
        # Células livres não cobertas = azul
        image[self.occupancy_grid.occupancy_grid < 0.5] = [100, 100, 255]
        
        # Células cobertas = branco
        mask_covered = (self.occupancy_grid.coverage_grid > 0) & (self.occupancy_grid.occupancy_grid < 0.5)
        image[mask_covered] = [255, 255, 255]
        
        # Obstáculos = preto
        image[self.occupancy_grid.occupancy_grid >= 0.5] = [0, 0, 0]
        
        return image
    
    def save_map(self, filepath: str) -> bool:
        """Salva o mapa."""
        return self.occupancy_grid.save(filepath)
    
    def load_map(self, filepath: str) -> bool:
        """Carrega o mapa."""
        return self.occupancy_grid.load(filepath)
    
    def get_info(self) -> Dict:
        """Retorna informações do mapeamento."""
        return self.occupancy_grid.get_info()
