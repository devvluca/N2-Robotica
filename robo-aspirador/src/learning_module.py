"""
Módulo de Aprendizado de Rota para Robô Aspirador.

Implementa algoritmos de otimização e aprendizado por repetição
para melhorar a eficiência de limpeza.
"""

import json
import logging
import numpy as np
from typing import Dict, List, Tuple, Optional
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, asdict


@dataclass
class ExecutionMetrics:
    """Métricas de uma execução de limpeza."""
    execution_id: int
    timestamp: str
    duration: float                # segundos
    coverage_percentage: float
    energy_consumed: float          # Joules estimados
    mean_revisit_rate: float
    uncovered_cells: int
    success: bool


class LearningModule:
    """
    Módulo de aprendizado para otimização de rota.
    
    Rastreia execuções anteriores e ajusta estratégia de navegação
    para melhorar eficiência.
    """
    
    def __init__(self, config: Dict, robot_config_path: str = None):
        """
        Inicializa módulo de aprendizado.
        
        Args:
            config: Dicionário com configurações
            robot_config_path: Caminho para arquivo de configuração anterior (opcional)
        """
        self.config = config
        self.logger = logging.getLogger(__name__)
        
        # Histórico de execuções
        self.execution_history: List[ExecutionMetrics] = []
        self.current_execution_id = 0
        
        # Parâmetros de aprendizado
        self.exploration_rate = config['learning']['exploration_rate']
        self.strategy = config['learning']['exploration_strategy']
        self.max_iterations = config['learning']['max_iterations']
        self.convergence_threshold = config['learning']['convergence_threshold']
        
        # Dados aprendidos
        self.best_execution: Optional[ExecutionMetrics] = None
        self.best_strategy_params = {}
        self.learned_paths: Dict[str, List[Tuple[float, float]]] = {}
        
        # Carregar histórico anterior se disponível
        if robot_config_path:
            self._load_history(robot_config_path)
    
    def record_execution(self,
                        duration: float,
                        coverage_percentage: float,
                        energy_consumed: float,
                        mean_revisit_rate: float,
                        uncovered_cells: int,
                        trajectory: List[Tuple[float, float]]) -> ExecutionMetrics:
        """
        Registra uma execução completa.
        
        Args:
            duration: Tempo total em segundos
            coverage_percentage: Percentual de área coberta
            energy_consumed: Energia consumida em Joules
            mean_revisit_rate: Taxa média de revisitas
            uncovered_cells: Número de células não cobertas
            trajectory: Lista de (x, y) da trajetória
            
        Returns:
            Objeto ExecutionMetrics
        """
        success = coverage_percentage > 95.0
        
        metrics = ExecutionMetrics(
            execution_id=self.current_execution_id,
            timestamp=datetime.now().isoformat(),
            duration=duration,
            coverage_percentage=coverage_percentage,
            energy_consumed=energy_consumed,
            mean_revisit_rate=mean_revisit_rate,
            uncovered_cells=uncovered_cells,
            success=success
        )
        
        # Armazenar
        self.execution_history.append(metrics)
        self.learned_paths[f"execution_{self.current_execution_id}"] = trajectory
        
        # Atualizar melhor execução
        if self.best_execution is None or self._is_better_execution(metrics):
            self.best_execution = metrics
            self.logger.info(f"Nova melhor execução: Cobertura={coverage_percentage:.1f}%, "
                           f"Energia={energy_consumed:.2f}J, Duração={duration:.1f}s")
        
        self.current_execution_id += 1
        
        return metrics
    
    def _is_better_execution(self, new_metrics: ExecutionMetrics) -> bool:
        """
        Compara duas execuções para determinar qual é melhor.
        
        Critério: maximizar cobertura e minimizar energia
        """
        if self.best_execution is None:
            return True
        
        # Score de eficiência (cobertura/energia)
        new_score = new_metrics.coverage_percentage / (new_metrics.energy_consumed + 1.0)
        old_score = self.best_execution.coverage_percentage / (self.best_execution.energy_consumed + 1.0)
        
        return new_score > old_score * (1.0 + self.convergence_threshold)
    
    def should_continue_learning(self) -> bool:
        """
        Verifica se deve continuar aprendendo.
        
        Returns:
            True se não atingiu critério de convergência
        """
        if len(self.execution_history) < 2:
            return True
        
        if len(self.execution_history) >= self.max_iterations:
            return False
        
        # Verificar convergência
        recent = self.execution_history[-2:]
        improvement = abs(recent[1].coverage_percentage - recent[0].coverage_percentage)
        
        if improvement < self.convergence_threshold:
            self.logger.info(f"Convergência detectada (melhoria={improvement:.4f}%)")
            return False
        
        return True
    
    def get_next_strategy(self) -> str:
        """
        Determina estratégia para próxima execução.
        
        Returns:
            Nome da estratégia a usar
        """
        # Se ainda aprendendo
        if not self.should_continue_learning():
            return self.strategy
        
        # Exploração vs Exploração
        if np.random.random() < self.exploration_rate:
            strategies = ['spiral', 'systematic', 'random_walk']
            return np.random.choice(strategies)
        else:
            # Usar estratégia com melhor histórico
            if self.best_execution:
                return self.strategy
            else:
                return 'spiral'
    
    def update_strategy_parameters(self, 
                                 coverage: float,
                                 energy: float) -> None:
        """
        Atualiza parâmetros da estratégia baseado em desempenho.
        
        Args:
            coverage: Percentual de cobertura
            energy: Energia consumida
        """
        # Implementação simples de otimização
        efficiency = coverage / (energy + 1.0)
        
        if not self.best_strategy_params:
            self.best_strategy_params['efficiency'] = efficiency
        else:
            # Se houve melhoria
            if efficiency > self.best_strategy_params.get('efficiency', 0):
                self.best_strategy_params['efficiency'] = efficiency
                self.best_strategy_params['last_update'] = datetime.now().isoformat()
    
    def get_performance_summary(self) -> Dict:
        """
        Retorna resumo de desempenho do aprendizado.
        
        Returns:
            Dicionário com estatísticas
        """
        if not self.execution_history:
            return {
                'total_executions': 0,
                'average_coverage': 0.0,
                'best_coverage': 0.0,
                'average_energy': 0.0
            }
        
        coverages = [m.coverage_percentage for m in self.execution_history]
        energies = [m.energy_consumed for m in self.execution_history]
        
        return {
            'total_executions': len(self.execution_history),
            'average_coverage': float(np.mean(coverages)),
            'best_coverage': float(np.max(coverages)),
            'coverage_improvement': float(coverages[-1] - coverages[0]) if len(coverages) > 1 else 0.0,
            'average_energy': float(np.mean(energies)),
            'min_energy': float(np.min(energies)),
            'best_execution_id': self.best_execution.execution_id if self.best_execution else -1,
            'convergence_iterations': len(self.execution_history)
        }
    
    def save_learning_data(self, filepath: str) -> bool:
        """
        Salva dados de aprendizado em arquivo.
        
        Args:
            filepath: Caminho para arquivo JSON
            
        Returns:
            True se salvo com sucesso
        """
        try:
            data = {
                'execution_history': [asdict(m) for m in self.execution_history],
                'best_execution': asdict(self.best_execution) if self.best_execution else None,
                'best_strategy_params': self.best_strategy_params,
                'learned_paths': {k: v for k, v in self.learned_paths.items()},
                'current_execution_id': self.current_execution_id,
                'timestamp': datetime.now().isoformat()
            }
            
            Path(filepath).parent.mkdir(parents=True, exist_ok=True)
            
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2, default=str)
            
            self.logger.info(f"Dados de aprendizado salvos em {filepath}")
            return True
        
        except Exception as e:
            self.logger.error(f"Erro ao salvar dados de aprendizado: {e}")
            return False
    
    def load_learning_data(self, filepath: str) -> bool:
        """
        Carrega dados de aprendizado de arquivo.
        
        Args:
            filepath: Caminho para arquivo JSON
            
        Returns:
            True se carregado com sucesso
        """
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            # Reconstruir histórico de execuções
            self.execution_history = []
            for item in data.get('execution_history', []):
                metrics = ExecutionMetrics(**item)
                self.execution_history.append(metrics)
            
            # Carregar melhor execução
            if data.get('best_execution'):
                self.best_execution = ExecutionMetrics(**data['best_execution'])
            
            self.best_strategy_params = data.get('best_strategy_params', {})
            self.learned_paths = data.get('learned_paths', {})
            self.current_execution_id = data.get('current_execution_id', 0)
            
            self.logger.info(f"Dados de aprendizado carregados de {filepath}")
            return True
        
        except Exception as e:
            self.logger.error(f"Erro ao carregar dados de aprendizado: {e}")
            return False
    
    def _load_history(self, config_path: str) -> None:
        """Carrega histórico anterior se disponível."""
        history_file = Path(config_path).parent / "logs" / "learning_history.json"
        
        if history_file.exists():
            self.load_learning_data(str(history_file))


class RouteOptimizer:
    """
    Otimizador de rotas baseado em histórico de execuções.
    """
    
    @staticmethod
    def calculate_efficiency(execution: ExecutionMetrics) -> float:
        """
        Calcula eficiência de uma execução.
        
        Args:
            execution: Métricas de execução
            
        Returns:
            Score de eficiência [0, 1]
        """
        # Fórmula: (cobertura% / tempo) / (revisitas)
        coverage_factor = execution.coverage_percentage / 100.0
        time_factor = 1.0 / (execution.duration + 1.0)
        revisit_penalty = 1.0 / (1.0 + execution.mean_revisit_rate)
        
        efficiency = coverage_factor * time_factor * revisit_penalty
        
        return float(efficiency)
    
    @staticmethod
    def identify_problem_areas(learned_paths: Dict,
                              execution_id: int) -> List[Tuple[float, float]]:
        """
        Identifica áreas problemáticas da trajetória.
        
        Args:
            learned_paths: Dicionário de trajetórias aprendidas
            execution_id: ID da execução para análise
            
        Returns:
            Lista de pontos problemáticos
        """
        key = f"execution_{execution_id}"
        
        if key not in learned_paths:
            return []
        
        trajectory = learned_paths[key]
        problem_areas = []
        
        # Identificar areas onde robô ficou muito tempo
        for i in range(0, len(trajectory) - 10, 5):
            segment = trajectory[i:i+10]
            
            # Calcular variação de posição
            xs = [p[0] for p in segment]
            ys = [p[1] for p in segment]
            
            variance = np.var(xs) + np.var(ys)
            
            # Se baixa variância, estava preso
            if variance < 0.01:
                center_x = np.mean(xs)
                center_y = np.mean(ys)
                problem_areas.append((center_x, center_y))
        
        return problem_areas
