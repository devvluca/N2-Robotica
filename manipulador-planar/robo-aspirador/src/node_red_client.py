"""
Cliente para enviar métricas ao Node-RED via HTTP.
"""

import requests
import json
import time
import os
from threading import Thread
from queue import Queue


DEFAULT_URL = os.environ.get('NODE_RED_URL', 'https://roboaspirador.flowfuse.cloud')


class NodeRedClient:
    """Cliente HTTP para enviar dados ao Node-RED."""
    
    def __init__(self, base_url=None):
        self.base_url = base_url or DEFAULT_URL
        self.endpoint = f"{self.base_url}/aspirador"
        self.enabled = True
        self.queue = Queue()
        self._connected = False
        
        print(f"[NODE-RED] URL: {self.base_url}")
        
        # Thread de envio assíncrono
        self.sender_thread = Thread(target=self._sender_loop, daemon=True)
        self.sender_thread.start()
    
    def _sender_loop(self):
        """Loop de envio assíncrono."""
        while True:
            try:
                data = self.queue.get(timeout=1.0)
                if data is None:
                    break
                self._send(data)
            except:
                pass
    
    def _send(self, payload):
        """Envia dados ao Node-RED."""
        if not self.enabled:
            return False
        
        try:
            response = requests.post(
                self.endpoint,
                json=payload,
                timeout=2.0,
                headers={'Content-Type': 'application/json'}
            )
            if response.status_code == 200:
                if not self._connected:
                    self._connected = True
                    print(f"[NODE-RED] ✓ Conectado")
                return True
            return False
        except requests.exceptions.ConnectionError:
            if self._connected:
                print(f"[NODE-RED] ⚠ Desconectado")
                self._connected = False
            return False
        except:
            return False
    
    def send_start(self, config):
        """Notifica início da execução."""
        payload = {
            'type': 'simulation_start',
            'timestamp': time.time(),
            'execution': config.get('execution', 1),
            'config': config
        }
        self.queue.put(payload)
    
    def send_periodic_update(self, data):
        """Envia atualização periódica."""
        # Formato esperado pelo Node-RED
        payload = {
            'type': 'periodic_update',
            'timestamp': time.time(),
            'execution': data.get('execution', 1),
            'metrics': {
                'coverage_percent': data.get('coverage_percent', 0),
                'covered_area_m2': data.get('covered_area_m2', 0),
                'energy_consumed': data.get('energy', 0),
                'time_elapsed_s': data.get('time_elapsed', 0),
                'distance_traveled': data.get('distance', 0),
                'collisions': data.get('collisions', 0),
                'state': data.get('state', 'UNKNOWN')
            }
        }
        self.queue.put(payload)
    
    def send_trajectory_point(self, x, y, theta, sim_time):
        """Envia ponto da trajetória."""
        payload = {
            'type': 'trajectory_point',
            'timestamp': time.time(),
            'x': x,
            'y': y,
            'theta': theta,
            'sim_time': sim_time
        }
        self.queue.put(payload)
    
    def send_end(self, metrics):
        """Notifica fim da execução."""
        # Formato esperado pelo Node-RED
        payload = {
            'type': 'simulation_end',
            'timestamp': time.time(),
            'execution': metrics.get('execution', 1),
            'summary': {
                'coverage_percent': metrics.get('coverage_percent', 0),
                'covered_area_m2': metrics.get('covered_area_m2', 0),
                'total_time_s': metrics.get('total_time', 0),
                'energy_consumed': metrics.get('energy', 0),
                'distance_traveled': metrics.get('distance', 0),
                'collisions': metrics.get('collisions', 0),
                'efficiency': metrics.get('area_per_energy', 0)
            }
        }
        self.queue.put(payload)
    
    def send_comparison(self, all_metrics):
        """Envia comparativo de execuções."""
        payload = {
            'type': 'comparison',
            'timestamp': time.time(),
            'executions': all_metrics
        }
        self.queue.put(payload)
    
    def close(self):
        """Fecha conexão."""
        self.queue.put(None)
        self.sender_thread.join(timeout=2.0)


# Singleton
_client = None

def get_client(base_url=None):
    """Retorna instância singleton."""
    global _client
    if _client is None:
        _client = NodeRedClient(base_url)
    return _client
