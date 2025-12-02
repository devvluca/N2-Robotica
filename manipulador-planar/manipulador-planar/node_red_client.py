"""
Cliente para enviar métricas do manipulador planar para o Node-RED.
Usa HTTP POST para enviar dados em tempo real.
Suporta Node-RED local ou FlowFuse (cloud).
"""

import requests
import json
import time
import os
from threading import Thread, Lock
from queue import Queue

# URL padrão - FlowFuse cloud
DEFAULT_URL = os.environ.get('NODE_RED_URL', 'https://lucaaguiar.flowfuse.cloud')

class NodeRedClient:
    def __init__(self, base_url=None):
        self.base_url = base_url or DEFAULT_URL
        self.metrics_endpoint = f"{self.base_url}/planar"
        self.status_endpoint = f"{self.base_url}/planar"
        self.enabled = True
        self.queue = Queue()
        self.lock = Lock()
        self._connected = False
        
        print(f"[NODE-RED] Configurado para: {self.base_url}")
        
        # Iniciar thread de envio assíncrono
        self.sender_thread = Thread(target=self._sender_loop, daemon=True)
        self.sender_thread.start()
    
    def _sender_loop(self):
        """Loop que envia métricas da fila de forma assíncrona."""
        while True:
            try:
                data = self.queue.get(timeout=1.0)
                if data is None:
                    break
                self._send_data(data['endpoint'], data['payload'])
            except:
                pass
    
    def _send_data(self, endpoint, payload):
        """Envia dados para o Node-RED."""
        if not self.enabled:
            return False
        
        try:
            response = requests.post(
                endpoint,
                json=payload,
                timeout=2.0,
                headers={'Content-Type': 'application/json'}
            )
            if response.status_code == 200:
                if not self._connected:
                    self._connected = True
                    print(f"[NODE-RED] ✓ Conectado a {self.base_url}")
                return True
            else:
                return False
        except requests.exceptions.ConnectionError:
            if self._connected:
                print(f"[NODE-RED] ⚠ Desconectado (Node-RED não está rodando?)")
                self._connected = False
            return False
        except Exception as e:
            return False
    
    def send_cycle_metrics(self, cycle_num, total_cycles, metrics):
        """
        Envia métricas de um ciclo completo.
        
        Args:
            cycle_num: Número do ciclo atual
            total_cycles: Total de ciclos
            metrics: Dicionário com as métricas:
                - mean_error: Erro médio de posição (rad)
                - max_overshoot: Ângulo máximo de overshoot (rad)
                - total_energy: Energia total gasta (J)
                - stabilization_time: Tempo de estabilização (s)
                - success: Se o ciclo foi bem sucedido
        """
        payload = {
            "type": "cycle_metrics",
            "timestamp": time.time(),
            "cycle": cycle_num,
            "total_cycles": total_cycles,
            "metrics": {
                "mean_error_rad": round(metrics.get('mean_error', 0), 4),
                "mean_error_deg": round(metrics.get('mean_error', 0) * 57.2958, 2),
                "max_overshoot_rad": round(metrics.get('max_overshoot', 0), 4),
                "max_overshoot_deg": round(metrics.get('max_overshoot', 0) * 57.2958, 2),
                "total_energy_j": round(metrics.get('total_energy', 0), 3),
                "stabilization_time_s": round(metrics.get('stabilization_time', 0), 3),
                "success": metrics.get('success', True)
            }
        }
        
        self.queue.put({'endpoint': self.metrics_endpoint, 'payload': payload})
        return True
    
    def send_realtime_state(self, joint_angles, joint_targets, ee_position, 
                            carrying_object=False, obstacle_detected=False):
        """
        Envia estado em tempo real do manipulador.
        
        Args:
            joint_angles: Lista de ângulos atuais das juntas (rad)
            joint_targets: Lista de ângulos alvo das juntas (rad)
            ee_position: Posição [x, y] do efetuador
            carrying_object: Se está carregando objeto
            obstacle_detected: Se detectou obstáculo
        """
        errors = [abs(t - a) for t, a in zip(joint_targets, joint_angles)]
        
        payload = {
            "type": "realtime_state",
            "timestamp": time.time(),
            "joints": {
                "angles_rad": [round(a, 4) for a in joint_angles],
                "angles_deg": [round(a * 57.2958, 2) for a in joint_angles],
                "targets_rad": [round(t, 4) for t in joint_targets],
                "targets_deg": [round(t * 57.2958, 2) for t in joint_targets],
                "errors_rad": [round(e, 4) for e in errors],
                "errors_deg": [round(e * 57.2958, 2) for e in errors]
            },
            "end_effector": {
                "x": round(ee_position[0], 4),
                "y": round(ee_position[1], 4)
            },
            "status": {
                "carrying_object": carrying_object,
                "obstacle_detected": obstacle_detected
            }
        }
        
        # Envio direto para tempo real (não usa fila)
        self._send_data(self.status_endpoint, payload)
    
    def send_simulation_start(self, config):
        """Notifica início da simulação."""
        payload = {
            "type": "simulation_start",
            "timestamp": time.time(),
            "config": {
                "dof": config.get('dof', 3),
                "total_cycles": config.get('cycles', 6),
                "link_lengths": config.get('links', [0.5, 0.5, 0.4]),
                "tray_position": config.get('tray_pos', [1.0, 0.5]),
                "obstacle_position": config.get('obstacle_pos', [1.0, 0.3])
            }
        }
        self.queue.put({'endpoint': self.metrics_endpoint, 'payload': payload})
    
    def send_simulation_end(self, summary):
        """Notifica fim da simulação com resumo."""
        payload = {
            "type": "simulation_end",
            "timestamp": time.time(),
            "summary": {
                "total_cycles": summary.get('total_cycles', 0),
                "successful_cycles": summary.get('successful_cycles', 0),
                "avg_mean_error_rad": round(summary.get('avg_mean_error', 0), 4),
                "avg_max_overshoot_rad": round(summary.get('avg_max_overshoot', 0), 4),
                "total_energy_j": round(summary.get('total_energy', 0), 3),
                "total_time_s": round(summary.get('total_time', 0), 2)
            }
        }
        self.queue.put({'endpoint': self.metrics_endpoint, 'payload': payload})
    
    def send_event(self, event_type, details=None):
        """
        Envia evento específico.
        
        event_type pode ser:
            - 'cube_spawned': Cubo criado
            - 'cube_grasped': Cubo agarrado
            - 'cube_released': Cubo solto
            - 'obstacle_avoided': Obstáculo evitado
            - 'perturbation_applied': Perturbação aplicada
            - 'perturbation_corrected': Perturbação corrigida
        """
        payload = {
            "type": "event",
            "timestamp": time.time(),
            "event": event_type,
            "details": details or {}
        }
        self.queue.put({'endpoint': self.metrics_endpoint, 'payload': payload})
    
    def close(self):
        """Fecha a conexão e para o thread."""
        self.queue.put(None)
        self.sender_thread.join(timeout=2.0)


# Instância global para uso em todo o projeto
_node_red_client = None

def get_client(base_url=None):
    """Retorna instância singleton do cliente Node-RED."""
    global _node_red_client
    if _node_red_client is None:
        _node_red_client = NodeRedClient(base_url)
    return _node_red_client

def send_metrics(cycle_num, total_cycles, mean_error, max_overshoot, total_energy, 
                 stabilization_time=0, success=True):
    """Função de conveniência para enviar métricas de ciclo."""
    client = get_client()
    return client.send_cycle_metrics(cycle_num, total_cycles, {
        'mean_error': mean_error,
        'max_overshoot': max_overshoot,
        'total_energy': total_energy,
        'stabilization_time': stabilization_time,
        'success': success
    })
