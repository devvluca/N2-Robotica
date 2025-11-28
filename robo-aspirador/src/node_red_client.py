"""
Cliente Node-RED para envio de dados do Robô Aspirador.

Implementa comunicação HTTP com Node-RED para telemetria em tempo real.
"""

import requests
import json
import logging
import threading
import queue
import time
from typing import Dict, Any, Optional
from datetime import datetime
from dataclasses import dataclass, asdict


@dataclass
class RobotTelemetry:
    """Estrutura de dados de telemetria do robô."""
    timestamp: str
    robot_id: str
    pose: Dict[str, float]              # {'x': float, 'y': float, 'heading': float}
    sensor_readings: Dict[str, float]   # {'sensor_0': dist, ...}
    velocity: Dict[str, float]          # {'linear': float, 'angular': float}
    coverage_percentage: float
    energy_consumed: float
    state: str                          # Estado do robô


@dataclass
class RobotMetrics:
    """Métricas aggregadas do robô."""
    timestamp: str
    execution_id: int
    total_coverage: float
    total_energy: float
    mean_revisits: float
    execution_time: float
    efficiency_score: float
    learning_iteration: int


class VacuumNodeREDClient:
    """
    Cliente HTTP para envio de telemetria do robô aspirador.
    
    Suporta:
    - Envio de telemetria em alta frequência (5Hz)
    - Envio de métricas em baixa frequência (1Hz)
    - Fila de mensagens thread-safe
    - Retry automático com backoff
    """
    
    def __init__(self,
                 host: str = "localhost",
                 port: int = 1880,
                 endpoint: str = "/vacuum/data",
                 protocol: str = "http",
                 timeout: float = 5.0):
        """
        Inicializa cliente Node-RED.
        
        Args:
            host: Endereço do servidor
            port: Porta do servidor
            endpoint: Endpoint para POST
            protocol: http ou https
            timeout: Timeout em segundos
        """
        self.host = host
        self.port = port
        self.endpoint = endpoint
        self.protocol = protocol
        self.timeout = timeout
        self.url = f"{protocol}://{host}:{port}{endpoint}"
        
        self.logger = logging.getLogger(__name__)
        self.is_connected = False
        self.last_error = None
        
        # Filas de mensagens
        self.telemetry_queue = queue.Queue()
        self.metrics_queue = queue.Queue()
        
        # Thread de envio
        self.sender_thread = None
        self.stop_sender = False
    
    def connect(self) -> bool:
        """
        Verifica conectividade com Node-RED.
        
        Returns:
            True se conectado
        """
        try:
            response = requests.get(
                f"{self.protocol}://{self.host}:{self.port}",
                timeout=self.timeout
            )
            self.is_connected = True
            self.logger.info(f"Conectado ao Node-RED em {self.url}")
            return True
        except Exception as e:
            self.logger.error(f"Erro ao conectar Node-RED: {e}")
            self.is_connected = False
            self.last_error = str(e)
            return False
    
    def send_telemetry(self, telemetry: RobotTelemetry, async_send: bool = True) -> bool:
        """
        Envia telemetria do robô.
        
        Args:
            telemetry: Objeto RobotTelemetry
            async_send: Se True, usa fila assíncrona
            
        Returns:
            True se enviado com sucesso
        """
        if async_send:
            self.telemetry_queue.put(telemetry)
            if not self.sender_thread or not self.sender_thread.is_alive():
                self.start_async_sender()
            return True
        else:
            return self._send_data_sync(asdict(telemetry))
    
    def send_metrics(self, metrics: RobotMetrics, async_send: bool = True) -> bool:
        """
        Envia métricas aggregadas.
        
        Args:
            metrics: Objeto RobotMetrics
            async_send: Se True, usa fila assíncrona
            
        Returns:
            True se enviado com sucesso
        """
        if async_send:
            self.metrics_queue.put(metrics)
            if not self.sender_thread or not self.sender_thread.is_alive():
                self.start_async_sender()
            return True
        else:
            return self._send_data_sync(asdict(metrics))
    
    def _send_data_sync(self, data: Dict, retry_count: int = 3) -> bool:
        """
        Envia dados de forma síncrona com retry.
        
        Args:
            data: Dicionário de dados
            retry_count: Número de tentativas
            
        Returns:
            True se enviado com sucesso
        """
        if not self.is_connected:
            return False
        
        for attempt in range(retry_count):
            try:
                response = requests.post(
                    self.url,
                    json=data,
                    timeout=self.timeout
                )
                
                if response.status_code == 200:
                    return True
                else:
                    self.logger.warning(f"Resposta {response.status_code}: {response.text}")
            
            except requests.exceptions.Timeout:
                self.logger.warning(f"Timeout na tentativa {attempt + 1}/{retry_count}")
            
            except Exception as e:
                self.logger.error(f"Erro ao enviar: {e}")
            
            if attempt < retry_count - 1:
                time.sleep(0.1 * (attempt + 1))
        
        return False
    
    def start_async_sender(self) -> None:
        """Inicia thread de envio assíncrono."""
        self.stop_sender = False
        self.sender_thread = threading.Thread(target=self._async_sender_loop, daemon=True)
        self.sender_thread.start()
    
    def stop_async_sender(self) -> None:
        """Para thread de envio assíncrono."""
        self.stop_sender = True
        if self.sender_thread:
            self.sender_thread.join(timeout=2.0)
    
    def _async_sender_loop(self) -> None:
        """Loop de envio assíncrono."""
        while not self.stop_sender:
            try:
                # Tentar telemetria
                try:
                    telemetry = self.telemetry_queue.get(timeout=0.1)
                    self._send_data_sync(asdict(telemetry))
                except queue.Empty:
                    pass
                
                # Tentar métricas
                try:
                    metrics = self.metrics_queue.get(timeout=0.1)
                    self._send_data_sync(asdict(metrics))
                except queue.Empty:
                    pass
            
            except Exception as e:
                self.logger.error(f"Erro no loop assíncrono: {e}")
    
    def get_status(self) -> Dict[str, Any]:
        """
        Retorna status da conexão.
        
        Returns:
            Dicionário com status
        """
        return {
            'connected': self.is_connected,
            'url': self.url,
            'telemetry_queue_size': self.telemetry_queue.qsize(),
            'metrics_queue_size': self.metrics_queue.qsize(),
            'sender_active': self.sender_thread is not None and self.sender_thread.is_alive(),
            'last_error': self.last_error
        }
    
    def disconnect(self) -> None:
        """Desconecta e limpa recursos."""
        self.stop_async_sender()
        self.is_connected = False
        self.logger.info("Cliente Node-RED desconectado")


class TelemetryAggregator:
    """
    Agregador de telemetria para envio eficiente.
    
    Coleta dados da simulação e monta pacotes de telemetria.
    """
    
    def __init__(self, robot_id: str = "vacuum_001"):
        """
        Inicializa agregador.
        
        Args:
            robot_id: ID único do robô
        """
        self.robot_id = robot_id
        self.logger = logging.getLogger(__name__)
    
    def create_telemetry(self,
                        x: float, y: float, heading: float,
                        sensor_readings: Dict[str, float],
                        linear_vel: float, angular_vel: float,
                        coverage: float, energy: float,
                        state: str) -> RobotTelemetry:
        """
        Cria pacote de telemetria.
        
        Args:
            x, y: Posição em metros
            heading: Orientação em radianos
            sensor_readings: Dicionário de leituras dos sensores
            linear_vel: Velocidade linear em m/s
            angular_vel: Velocidade angular em rad/s
            coverage: Percentual de cobertura
            energy: Energia consumida em Joules
            state: Estado do robô
            
        Returns:
            Objeto RobotTelemetry
        """
        return RobotTelemetry(
            timestamp=datetime.now().isoformat(),
            robot_id=self.robot_id,
            pose={
                'x': float(x),
                'y': float(y),
                'heading': float(heading)
            },
            sensor_readings={str(k): float(v) for k, v in sensor_readings.items()},
            velocity={
                'linear': float(linear_vel),
                'angular': float(angular_vel)
            },
            coverage_percentage=float(coverage),
            energy_consumed=float(energy),
            state=state
        )
    
    def create_metrics(self,
                      execution_id: int,
                      total_coverage: float,
                      total_energy: float,
                      mean_revisits: float,
                      execution_time: float,
                      efficiency_score: float,
                      learning_iteration: int) -> RobotMetrics:
        """
        Cria pacote de métricas.
        
        Returns:
            Objeto RobotMetrics
        """
        return RobotMetrics(
            timestamp=datetime.now().isoformat(),
            execution_id=execution_id,
            total_coverage=float(total_coverage),
            total_energy=float(total_energy),
            mean_revisits=float(mean_revisits),
            execution_time=float(execution_time),
            efficiency_score=float(efficiency_score),
            learning_iteration=learning_iteration
        )
