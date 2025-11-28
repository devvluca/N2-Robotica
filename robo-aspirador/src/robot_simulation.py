"""
Simulador principal do robô aspirador em PyBullet.

Gerencia:
- Simulação física da dinâmica do robô
- Sensors (ultrassônicos)
- Loop de controle com mapeamento/navegação/aprendizado
- Integração com Node-RED
- Coleta de métricas
"""

import pybullet as p
import pybullet_data
import numpy as np
import logging
import time
import csv
from typing import Dict, List, Tuple, Any, Optional
from datetime import datetime
from pathlib import Path

from mapping_module import MappingModule, OccupancyGrid
from navigation_module import NavigationModule, NavigationState
from learning_module import LearningModule, ExecutionMetrics
from node_red_client import VacuumNodeREDClient, TelemetryAggregator, RobotTelemetry
from environment_setup import EnvironmentSetup


class UltrasonicSensor:
    """Simula sensor ultrassônico com ruído realista."""
    
    def __init__(self, robot_id: int, link_index: int, 
                 angle: float, max_range: float = 1.0,
                 noise_std: float = 0.02):
        """
        Inicializa sensor.
        
        Args:
            robot_id: ID do robô
            link_index: Índice do link do robô
            angle: Ângulo do sensor em radianos (relativo ao robô)
            max_range: Alcance máximo em metros
            noise_std: Desvio padrão do ruído em metros
        """
        self.robot_id = robot_id
        self.link_index = link_index
        self.angle = angle
        self.max_range = max_range
        self.noise_std = noise_std
        self.last_reading = max_range
    
    def read(self) -> float:
        """
        Realiza leitura do sensor.
        
        Returns:
            Distância em metros com ruído
        """
        try:
            # Obter posição e orientação do robô
            robot_pos, robot_orn = p.getBasePositionAndOrientation(self.robot_id)
            
            # Obter orientação do link se diferente da base
            if self.link_index >= 0:
                link_state = p.getLinkState(self.robot_id, self.link_index)
                sensor_pos = link_state[0]
                sensor_orn = link_state[1]
            else:
                sensor_pos = robot_pos
                sensor_orn = robot_orn
            
            # Calcular ângulo absoluto do sensor
            euler = p.getEulerFromQuaternion(robot_orn)
            robot_heading = euler[2]
            absolute_angle = robot_heading + self.angle
            
            # Direção de emissão do ray
            ray_dir = np.array([
                np.cos(absolute_angle),
                np.sin(absolute_angle),
                0
            ])
            
            # Ponto final do ray
            ray_end = np.array(sensor_pos) + ray_dir * self.max_range
            
            # Realizar raycast
            results = p.rayTestBatch(
                [sensor_pos],
                [ray_end],
                threadId=0
            )
            
            if results[0][0] != -1:
                # Distância até o obstáculo
                distance = results[0][2] * self.max_range
            else:
                # Nenhum obstáculo detectado
                distance = self.max_range
            
            # Adicionar ruído Gaussiano
            distance += np.random.normal(0, self.noise_std)
            distance = np.clip(distance, 0, self.max_range)
            
            self.last_reading = distance
            return float(distance)
        
        except Exception as e:
            return self.max_range


class VacuumRobotSimulation:
    """
    Simulador principal do robô aspirador.
    
    Características:
    - Robô diferencial com 2 motores
    - 5 sensores ultrassônicos
    - Mapeamento em tempo real
    - Navegação autônoma com aprendizado
    - Integração com Node-RED
    """
    
    def __init__(self, config: Dict[str, Any], use_gui: bool = True):
        """
        Inicializa simulador.
        
        Args:
            config: Dicionário de configuração YAML
            use_gui: Se True, renderiza com GUI
        """
        self.config = config
        self.use_gui = use_gui
        
        # Logger
        self.logger = logging.getLogger(__name__)
        
        # Estado da simulação
        self.robot_id = None
        self.current_time = 0.0
        self.sim_step = 0
        self.is_running = False
        self.current_execution = 0
        
        # Configurações
        robot_cfg = config.get('robot', {})
        sim_cfg = config.get('simulation', {})
        
        self.robot_radius = robot_cfg.get('radius', 0.15)
        self.robot_mass = robot_cfg.get('mass', 5.0)
        self.max_linear_vel = robot_cfg.get('max_linear_velocity', 0.5)
        self.max_angular_vel = robot_cfg.get('max_angular_velocity', 2.0)
        
        self.dt = sim_cfg.get('time_step', 0.01)
        self.gravity = sim_cfg.get('gravity', 9.81)
        
        # Módulos
        self.environment = None
        self.mapping = None
        self.navigation = None
        self.learning = None
        self.node_red = None
        self.telemetry_agg = None
        
        # Sensores
        self.sensors: Dict[str, UltrasonicSensor] = {}
        
        # Métricas
        self.execution_start_time = None
        self.energy_consumed = 0.0
        self.current_coverage = 0.0
        self.trajectory = []
        self.csv_file = None
        self.csv_writer = None
        
        # Estado do robô
        self.robot_state = NavigationState.IDLE
    
    def initialize(self) -> bool:
        """
        Inicializa simulação.
        
        Returns:
            True se inicialização bem-sucedida
        """
        try:
            # Conectar ao PyBullet
            if self.use_gui:
                self.client_id = p.connect(p.GUI)
                p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
                p.resetDebugVisualizerCamera(cameraDistance=15, cameraYaw=0, 
                                             cameraPitch=-60, cameraTargetPosition=[5, 4, 0])
            else:
                self.client_id = p.connect(p.DIRECT)
            
            # Adicionar dados extras do PyBullet
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            
            # Configurar física
            p.setGravity(0, 0, -self.gravity)
            p.setTimeStep(self.dt)
            p.setPhysicsEngineParameter(numSubSteps=1)
            
            # Configurar ambiente
            self.environment = EnvironmentSetup(self.config)
            self.environment.setup_world()
            self.environment.load_obstacles()
            
            # Criar robô
            self._create_robot()
            
            # Inicializar módulos
            grid_res = self.config.get('mapping', {}).get('grid_resolution', 0.1)
            self.mapping = MappingModule(
                grid_resolution=grid_res,
                world_width=self.config.get('environment', {}).get('world_width', 10.0),
                world_height=self.config.get('environment', {}).get('world_height', 8.0)
            )
            
            self.navigation = NavigationModule(self.config)
            self.learning = LearningModule(self.config)
            
            # Node-RED
            node_cfg = self.config.get('node_red', {})
            if node_cfg.get('enabled', False):
                self.node_red = VacuumNodeREDClient(
                    host=node_cfg.get('host', 'localhost'),
                    port=node_cfg.get('port', 1880),
                    endpoint=node_cfg.get('endpoint', '/vacuum/data')
                )
                self.node_red.connect()
                self.telemetry_agg = TelemetryAggregator(node_cfg.get('robot_id', 'vacuum_001'))
            
            # Arquivo de log
            self._setup_logging()
            
            self.logger.info("Simulação inicializada com sucesso")
            return True
        
        except Exception as e:
            self.logger.error(f"Erro na inicialização: {e}", exc_info=True)
            return False
    
    def _create_robot(self) -> None:
        """Cria o robô diferencial no ambiente."""
        # Criar geometria do robô (cilindro)
        cylinder_shape = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=self.robot_radius,
            height=0.1
        )
        
        # Criar corpo
        self.robot_id = p.createMultiBody(
            baseMass=self.robot_mass,
            baseCollisionShapeIndex=cylinder_shape,
            basePosition=[self.robot_radius + 0.5, self.robot_radius + 0.5, 0.05],
            baseOrientation=[0, 0, 0, 1]
        )
        
        # Propriedades de dinâmica
        p.changeDynamics(
            self.robot_id,
            -1,
            lateralFriction=1.0,
            restitution=0.1,
            linearDamping=0.1,
            angularDamping=0.5
        )
        
        # Criar sensores ultrassônicos
        sensor_cfg = self.config.get('robot', {}).get('sensors', {})
        num_sensors = sensor_cfg.get('count', 5)
        sensor_range = sensor_cfg.get('range', 1.0)
        
        for i in range(num_sensors):
            angle = 2 * np.pi * i / num_sensors - np.pi
            self.sensors[f'sensor_{i}'] = UltrasonicSensor(
                self.robot_id,
                -1,
                angle,
                max_range=sensor_range,
                noise_std=0.02
            )
        
        self.logger.info(f"Robô criado com {num_sensors} sensores ultrassônicos")
    
    def _setup_logging(self) -> None:
        """Configura arquivo CSV de logging."""
        log_cfg = self.config.get('logging', {})
        if not log_cfg.get('enabled', False):
            return
        
        log_dir = Path(log_cfg.get('log_directory', './logs'))
        log_dir.mkdir(exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = log_dir / f"execution_{timestamp}.csv"
        
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                'time', 'x', 'y', 'heading', 'linear_vel', 'angular_vel',
                'coverage', 'energy', 'state', 'strategy'
            ]
        )
        self.csv_writer.writeheader()
        self.logger.info(f"Log criado em {csv_path}")
    
    def run_execution(self, max_duration: float = 300.0) -> ExecutionMetrics:
        """
        Executa uma limpeza do robô.
        
        Args:
            max_duration: Duração máxima em segundos
            
        Returns:
            Métricas da execução
        """
        self.is_running = True
        self.current_execution += 1
        self.execution_start_time = time.time()
        self.energy_consumed = 0.0
        self.trajectory = []
        
        # Obter estratégia de navegação
        strategy = self.learning.get_next_strategy()
        self.navigation.set_exploration_strategy(strategy)
        
        self.logger.info(f"Iniciando execução {self.current_execution} com estratégia: {strategy}")
        
        while self.is_running:
            elapsed = time.time() - self.execution_start_time
            
            # Verificar condição de parada
            if elapsed > max_duration:
                self.logger.info(f"Tempo máximo atingido: {elapsed:.1f}s")
                break
            
            # Executar passo da simulação
            self._step()
            
            # Verificar convergência (cobertura > threshold)
            if self.current_coverage > 0.95:
                self.logger.info(f"Cobertura suficiente atingida: {self.current_coverage:.1%}")
                break
        
        # Colher métricas
        metrics = self._collect_metrics(elapsed)
        
        # Registrar no módulo de aprendizado
        self.learning.record_execution(metrics)
        
        self.is_running = False
        self.logger.info(f"Execução {self.current_execution} concluída")
        
        return metrics
    
    def _step(self) -> None:
        """Executa um passo de simulação."""
        # Leitura de sensores
        sensor_readings = {}
        for name, sensor in self.sensors.items():
            sensor_readings[name] = sensor.read()
        
        # Obter posição do robô
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(orn)
        x, y, heading = pos[0], pos[1], euler[2]
        
        # Atualizar mapeamento
        self.mapping.process_sensor_data(
            x=x, y=y,
            heading=heading,
            sensor_readings=sensor_readings
        )
        self.current_coverage = self.mapping.get_coverage_percentage()
        
        # Computar velocidades
        state, linear_vel, angular_vel = self.navigation.compute_velocity(sensor_readings)
        self.robot_state = state
        
        # Aplicar velocidades ao robô
        self._apply_velocity(linear_vel, angular_vel)
        
        # Calcular energia consumida
        energy_step = abs(linear_vel) * 0.01 + abs(angular_vel) * 0.005
        self.energy_consumed += energy_step
        
        # Registrar trajetória
        self.trajectory.append((self.current_time, x, y, heading))
        
        # Enviar telemetria para Node-RED
        if self.node_red and self.sim_step % 5 == 0:  # 5Hz
            telemetry = self.telemetry_agg.create_telemetry(
                x=x, y=y, heading=heading,
                sensor_readings=sensor_readings,
                linear_vel=linear_vel,
                angular_vel=angular_vel,
                coverage=self.current_coverage,
                energy=self.energy_consumed,
                state=self.robot_state.name
            )
            self.node_red.send_telemetry(telemetry)
        
        # Log em CSV
        if self.csv_writer:
            self.csv_writer.writerow({
                'time': self.current_time,
                'x': x,
                'y': y,
                'heading': heading,
                'linear_vel': linear_vel,
                'angular_vel': angular_vel,
                'coverage': self.current_coverage,
                'energy': self.energy_consumed,
                'state': self.robot_state.name,
                'strategy': self.navigation.current_strategy
            })
        
        # Passo de física
        p.stepSimulation()
        
        self.current_time += self.dt
        self.sim_step += 1
    
    def _apply_velocity(self, linear_vel: float, angular_vel: float) -> None:
        """
        Aplica velocidades ao robô usando cinemática diferencial.
        
        Args:
            linear_vel: Velocidade linear em m/s
            angular_vel: Velocidade angular em rad/s
        """
        # Limitar velocidades
        linear_vel = np.clip(linear_vel, -self.max_linear_vel, self.max_linear_vel)
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        
        # Computar velocidades das rodas (cinemática diferencial)
        wheel_radius = self.robot_radius
        wheel_base = 2 * self.robot_radius
        
        v_left = linear_vel - (angular_vel * wheel_base / 2)
        v_right = linear_vel + (angular_vel * wheel_base / 2)
        
        # Aplicar velocidades como força/torque
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        
        # Converter para força/torque
        force = linear_vel * self.robot_mass * 10
        torque = angular_vel * self.robot_mass * 0.1
        
        p.applyExternalForce(self.robot_id, -1, [force, 0, 0], [0, 0, 0], p.WORLD_FRAME)
        p.applyExternalTorque(self.robot_id, -1, [0, 0, torque], p.WORLD_FRAME)
    
    def _collect_metrics(self, execution_time: float) -> ExecutionMetrics:
        """
        Coleta métricas da execução.
        
        Returns:
            Objeto ExecutionMetrics
        """
        coverage = self.current_coverage * 100
        energy = self.energy_consumed
        
        # Calcular taxa média de revisitas
        revisit_rate = self.mapping.get_mean_revisit_rate()
        
        uncovered = self.mapping.get_uncovered_cells()
        
        # Calcular eficiência
        efficiency = self.learning.route_optimizer.calculate_efficiency(
            coverage_percentage=coverage,
            energy_consumed=energy,
            execution_time=execution_time
        )
        
        metrics = ExecutionMetrics(
            execution_id=self.current_execution,
            timestamp=datetime.now().isoformat(),
            duration=execution_time,
            coverage_percentage=coverage,
            energy_consumed=energy,
            mean_revisit_rate=revisit_rate,
            uncovered_cells=uncovered,
            success=(coverage > 80.0)
        )
        
        return metrics
    
    def run_learning_cycles(self, num_cycles: int = 5) -> List[ExecutionMetrics]:
        """
        Executa múltiplos ciclos de aprendizado.
        
        Args:
            num_cycles: Número de ciclos
            
        Returns:
            Lista de métricas de cada ciclo
        """
        all_metrics = []
        
        for cycle in range(num_cycles):
            self.logger.info(f"Ciclo de aprendizado {cycle + 1}/{num_cycles}")
            
            # Resetar mapa e robô
            self.mapping.reset()
            self._reset_robot_position()
            
            # Executar
            metrics = self.run_execution(max_duration=60.0)
            all_metrics.append(metrics)
            
            # Verificar convergência
            if not self.learning.should_continue_learning():
                self.logger.info("Convergência detectada, finalizando aprendizado")
                break
        
        return all_metrics
    
    def _reset_robot_position(self) -> None:
        """Reseta posição do robô para o início."""
        start_x = self.robot_radius + 0.5
        start_y = self.robot_radius + 0.5
        
        p.resetBasePositionAndOrientation(
            self.robot_id,
            [start_x, start_y, 0.05],
            [0, 0, 0, 1]
        )
        
        p.resetBaseVelocity(self.robot_id, [0, 0, 0], [0, 0, 0])
        
        self.current_time = 0.0
        self.sim_step = 0
    
    def shutdown(self) -> None:
        """Desliga simulação e limpa recursos."""
        if self.is_running:
            self.is_running = False
        
        if self.csv_file:
            self.csv_file.close()
        
        if self.mapping:
            self.mapping.save()
        
        if self.learning:
            self.learning.save_learning_data()
        
        if self.node_red:
            self.node_red.disconnect()
        
        if self.environment:
            self.environment.cleanup()
        
        p.disconnect()
        self.logger.info("Simulação encerrada")
    
    def get_summary(self) -> Dict[str, Any]:
        """
        Retorna resumo do estado da simulação.
        
        Returns:
            Dicionário com informações
        """
        return {
            'current_execution': self.current_execution,
            'current_time': self.current_time,
            'current_coverage': self.current_coverage,
            'energy_consumed': self.energy_consumed,
            'robot_state': self.robot_state.name if self.robot_state else 'UNKNOWN',
            'trajectory_points': len(self.trajectory),
            'is_running': self.is_running,
            'environment': self.environment.get_summary() if self.environment else {}
        }
