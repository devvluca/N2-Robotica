"""
Cliente HTTP para envio de dados ao Node-RED.

Fornece interface para comunicação com Node-RED via requisições HTTP POST
com suporte a retry e tratamento de erros.
"""

import requests
import json
import logging
from typing import Dict, Any, Optional
from datetime import datetime
from dataclasses import dataclass, asdict
import threading
import queue
import time


@dataclass
class RobotData:
    """Estrutura de dados para transmissão ao Node-RED."""
    timestamp: str
    joint_angles: Dict[str, float]  # {'joint1': θ1, 'joint2': θ2}
    reference_angles: Dict[str, float]
    errors: Dict[str, float]
    torques: Dict[str, float]
    end_effector_position: Dict[str, float]  # {'x': x, 'y': y}
    end_effector_distance: float
    metrics: Dict[str, Any] = None


class NodeREDClient:
    """
    Cliente para envio de dados robóticos ao Node-RED.
    
    Oferece suporte a:
    - Envio HTTP com retry automático
    - Fila de mensagens thread-safe
    - Logging detalhado
    - Validação de conectividade
    """
    
    def __init__(self, 
                 host: str = "localhost",
                 port: int = 1880,
                 endpoint: str = "/data/robot",
                 protocol: str = "http",
                 timeout: float = 5.0):
        """
        Inicializa o cliente Node-RED.
        
        Args:
            host: Endereço do servidor Node-RED
            port: Porta do servidor
            endpoint: Endpoint para POST (ex: "/data/robot")
            protocol: Protocolo HTTP ou HTTPS
            timeout: Timeout em segundos para requisições
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
        
        # Fila de mensagens para envio assíncrono
        self.message_queue = queue.Queue()
        self.sender_thread = None
        self.stop_sender = False
        
    def connect(self) -> bool:
        """
        Verifica conectividade com Node-RED.
        
        Returns:
            True se conectado, False caso contrário
        """
        try:
            # Tentar fazer um GET simples para verificar conectividade
            response = requests.get(
                f"{self.protocol}://{self.host}:{self.port}",
                timeout=self.timeout
            )
            self.is_connected = True
            self.logger.info(f"Conectado ao Node-RED em {self.url}")
            return True
        except requests.exceptions.ConnectionError as e:
            self.logger.error(f"Erro de conexão com Node-RED: {e}")
            self.is_connected = False
            self.last_error = str(e)
            return False
        except Exception as e:
            self.logger.error(f"Erro ao conectar Node-RED: {e}")
            self.is_connected = False
            self.last_error = str(e)
            return False
    
    def send_data(self, data: RobotData, retry_count: int = 3) -> bool:
        """
        Envia dados para Node-RED com retry automático.
        
        Args:
            data: Objeto RobotData com informações do robô
            retry_count: Número de tentativas
            
        Returns:
            True se enviado com sucesso, False caso contrário
        """
        if not self.is_connected:
            self.logger.warning("Cliente desconectado de Node-RED")
            return False
        
        payload = asdict(data)
        
        for attempt in range(retry_count):
            try:
                response = requests.post(
                    self.url,
                    json=payload,
                    timeout=self.timeout
                )
                
                if response.status_code == 200:
                    self.logger.debug(f"Dados enviados com sucesso para Node-RED")
                    return True
                else:
                    self.logger.warning(
                        f"Resposta inesperada do Node-RED: "
                        f"{response.status_code} - {response.text}"
                    )
                    
            except requests.exceptions.Timeout:
                self.logger.warning(f"Timeout na tentativa {attempt + 1}/{retry_count}")
                
            except requests.exceptions.ConnectionError as e:
                self.logger.warning(f"Erro de conexão na tentativa {attempt + 1}/{retry_count}")
                
            except Exception as e:
                self.logger.error(f"Erro ao enviar dados: {e}")
            
            if attempt < retry_count - 1:
                time.sleep(0.1 * (attempt + 1))  # Backoff exponencial
        
        self.logger.error(f"Falha ao enviar dados após {retry_count} tentativas")
        return False
    
    def send_data_async(self, data: RobotData) -> None:
        """
        Envia dados de forma assíncrona (não-bloqueante).
        
        Args:
            data: Objeto RobotData com informações do robô
        """
        if not self.sender_thread or not self.sender_thread.is_alive():
            self.start_async_sender()
        
        self.message_queue.put(data)
    
    def start_async_sender(self) -> None:
        """Inicia thread de envio assíncrono."""
        self.stop_sender = False
        self.sender_thread = threading.Thread(target=self._async_sender_loop, daemon=True)
        self.sender_thread.start()
        self.logger.debug("Thread de envio assíncrono iniciada")
    
    def stop_async_sender(self) -> None:
        """Para thread de envio assíncrono."""
        self.stop_sender = True
        if self.sender_thread:
            self.sender_thread.join(timeout=2.0)
        self.logger.debug("Thread de envio assíncrono parada")
    
    def _async_sender_loop(self) -> None:
        """Loop de envio assíncrono (executado em thread separada)."""
        while not self.stop_sender:
            try:
                data = self.message_queue.get(timeout=0.5)
                self.send_data(data)
            except queue.Empty:
                continue
            except Exception as e:
                self.logger.error(f"Erro no loop assíncrono: {e}")
    
    def get_status(self) -> Dict[str, Any]:
        """
        Retorna status da conexão com Node-RED.
        
        Returns:
            Dicionário com informações de status
        """
        queue_size = self.message_queue.qsize()
        return {
            'connected': self.is_connected,
            'url': self.url,
            'last_error': self.last_error,
            'queue_size': queue_size,
            'async_sender_active': self.sender_thread is not None and self.sender_thread.is_alive()
        }
    
    def disconnect(self) -> None:
        """Desconecta e encerra threads."""
        self.stop_async_sender()
        self.is_connected = False
        self.logger.info("Cliente Node-RED desconectado")


class DataAggregator:
    """
    Agregador de dados para envio eficiente ao Node-RED.
    
    Coleta dados da simulação e monta pacotes RobotData para envio.
    """
    
    def __init__(self, send_frequency: float = 10.0):
        """
        Inicializa agregador de dados.
        
        Args:
            send_frequency: Frequência de envio em Hz
        """
        self.send_frequency = send_frequency
        self.send_period = 1.0 / send_frequency
        self.last_send_time = 0.0
        self.logger = logging.getLogger(__name__)
    
    def create_data_packet(self,
                          timestamp: float,
                          joint_angles: tuple,
                          reference_angles: tuple,
                          errors: tuple,
                          torques: tuple,
                          ee_position: tuple,
                          ee_distance: float,
                          metrics: Dict = None) -> RobotData:
        """
        Cria um pacote de dados para envio.
        
        Args:
            timestamp: Tempo atual em segundos
            joint_angles: (θ1, θ2) ângulos atuais
            reference_angles: (θ1_ref, θ2_ref) ângulos de referência
            errors: (e1, e2) erros angulares
            torques: (τ1, τ2) torques aplicados
            ee_position: (x, y) posição do end-effector
            ee_distance: distância do end-effector da origem
            metrics: dicionário opcional com métricas adicionais
            
        Returns:
            Objeto RobotData formatado
        """
        return RobotData(
            timestamp=datetime.now().isoformat(),
            joint_angles={
                'joint1': float(joint_angles[0]),
                'joint2': float(joint_angles[1])
            },
            reference_angles={
                'joint1': float(reference_angles[0]),
                'joint2': float(reference_angles[1])
            },
            errors={
                'joint1': float(errors[0]),
                'joint2': float(errors[1])
            },
            torques={
                'joint1': float(torques[0]),
                'joint2': float(torques[1])
            },
            end_effector_position={
                'x': float(ee_position[0]),
                'y': float(ee_position[1])
            },
            end_effector_distance=float(ee_distance),
            metrics=metrics or {}
        )
    
    def should_send(self, current_time: float) -> bool:
        """
        Verifica se é hora de enviar dados (baseado em frequência).
        
        Args:
            current_time: Tempo atual em segundos
            
        Returns:
            True se deve enviar, False caso contrário
        """
        if current_time - self.last_send_time >= self.send_period:
            self.last_send_time = current_time
            return True
        return False
