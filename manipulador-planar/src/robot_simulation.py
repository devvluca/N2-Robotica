"""
Simulação principal do manipulador planar 2 DOF no PyBullet.

Este módulo implementa:
- Simulação em tempo real com controle PID
- Coleta de métricas de desempenho
- Logging detalhado
- Integração com Node-RED
- Visualização 3D
"""

import pybullet as p
import pybullet_data
import numpy as np
import yaml
import logging
import csv
from typing import Dict, Tuple, List, Optional
from datetime import datetime
import os
import sys
from pathlib import Path

# Importar módulos locais
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from src.pid_controller import PIDController, PIDConfig, DualJointPIDController
from src.kinematics import PlanarKinematics
from src.node_red_client import NodeREDClient, DataAggregator, RobotData


class RobotSimulation:
    """
    Simulação principal do manipulador planar 2 DOF.
    
    Gerencia:
    - Ambiente de simulação PyBullet
    - Controladores PID para cada junta
    - Logging e coleta de dados
    - Comunicação com Node-RED
    """
    
    def __init__(self, config_path: str = None, enable_gui: bool = True):
        """
        Inicializa simulação.
        
        Args:
            config_path: Caminho para arquivo de configuração YAML
            enable_gui: Se True, ativa visualização 3D
        """
        # Inicializar um logger básico antes de carregar config (para reports iniciais)
        self.logger = logging.getLogger(__name__)

        # Carregar configuração
        self.config = self._load_config(config_path)

        # Configurar logging (baseado em config)
        self._setup_logging()
        # Re-obter o logger para aplicar configuração adequada
        self.logger = logging.getLogger(__name__)
        
        # Inicializar PyBullet
        self.enable_gui = self.config['simulation']['gui_enabled']
        self.physics_client = self._connect_pybullet()
        
        # Inicializar componentes
        self.robot_id = None
        self.joint_ids = []
        self._setup_environment()
        self._create_robot()
        
        # Controladores PID
        self.pid_controller = self._setup_pid_controllers()
        
        # Cinemática
        link_lengths = (
            self.config['robot']['link1_length'],
            self.config['robot']['link2_length']
        )
        self.kinematics = PlanarKinematics(link_lengths)
        
        # Node-RED client
        self.node_red_client = self._setup_node_red_client()
        self.data_aggregator = DataAggregator(
            self.config['node_red']['send_frequency']
        )
        
        # Estado da simulação
        self.current_time = 0.0
        self.reference_angles = np.array([0.0, 0.0])
        self.current_angles = np.array([0.0, 0.0])
        self.current_velocities = np.array([0.0, 0.0])
        
        # Métricas
        self.metrics_history = {
            'time': [],
            'joint1_angle': [],
            'joint2_angle': [],
            'joint1_ref': [],
            'joint2_ref': [],
            'joint1_error': [],
            'joint2_error': [],
            'joint1_torque': [],
            'joint2_torque': [],
            'ee_x': [],
            'ee_y': [],
            'ee_distance': []
        }
        
        # Perturbações
        self.perturbation_active = False
        self.perturbation_end_time = 0.0
        
        self.logger.info("Simulação do manipulador planar 2 DOF inicializada")
    
    def _setup_logging(self) -> None:
        """Configura sistema de logging."""
        log_dir = self.config.get('data_logging', {}).get('log_directory', './logs')
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        log_file = os.path.join(
            log_dir,
            f"simulation_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        )
        
        logging.basicConfig(
            level=logging.DEBUG,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_file),
                logging.StreamHandler()
            ]
        )
    
    def _load_config(self, config_path: str = None) -> Dict:
        """
        Carrega configuração do arquivo YAML.
        
        Args:
            config_path: Caminho para arquivo YAML
            
        Returns:
            Dicionário com configurações
        """
        if config_path is None:
            config_path = os.path.join(
                os.path.dirname(__file__),
                '..',
                'config',
                'robot_config.yaml'
            )
        
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            self.logger.info(f"Configuração carregada de {config_path}")
            return config
        except Exception as e:
            self.logger.error(f"Erro ao carregar configuração: {e}")
            raise
    
    def _connect_pybullet(self) -> int:
        """
        Conecta ao servidor PyBullet.
        
        Returns:
            ID da conexão
        """
        if self.enable_gui:
            physics_client = p.connect(p.GUI)
            self.logger.info("Conectado ao PyBullet em modo GUI")
        else:
            physics_client = p.connect(p.DIRECT)
            self.logger.info("Conectado ao PyBullet em modo DIRECT (sem GUI)")
        
        return physics_client
    
    def _setup_environment(self) -> None:
        """Configura ambiente de simulação (gravidade, timestep, etc)."""
        gravity = self.config['simulation']['gravity']
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(gravity[0], gravity[1], gravity[2])
        
        time_step = self.config['simulation']['time_step']
        p.setPhysicsEngineParameter(fixedTimeStep=time_step, numSubSteps=1)
        
        # Carregar plano de solo
        self.plane_id = p.loadURDF("plane.urdf")
        
        # Configurar câmera
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraPitch=45,
            cameraYaw=45,
            cameraTargetPosition=[0.4, 0.15, 0]
        )
        
        self.logger.debug("Ambiente de simulação configurado")
    
    def _create_robot(self) -> None:
        """Cria o manipulador planar 2 DOF dinamicamente."""
        self._create_robot_simple()
    
    def _create_robot_simple(self) -> None:
        """Cria um manipulador planar 2 DOF personalizado."""
        self.logger.info("Criando manipulador planar 2 DOF personalizado...")
        
        # Parâmetros do robô
        link1_length = self.config['robot']['link1_length']
        link2_length = self.config['robot']['link2_length']
        link1_mass = self.config['robot']['link1_mass']
        link2_mass = self.config['robot']['link2_mass']
        link_radius = self.config['robot']['link_radius']
        
        # Link 1: base para junta 1
        link1_collision = p.createCollisionShape(p.GEOM_CYLINDER, 
                                               radius=link_radius, 
                                               height=link1_length)
        link1_visual = p.createVisualShape(p.GEOM_CYLINDER, 
                                         radius=link_radius, 
                                         length=link1_length,
                                         rgbaColor=[0.8, 0.2, 0.2, 1])  # Vermelho
        
        # Link 2: junta 1 para junta 2
        link2_collision = p.createCollisionShape(p.GEOM_CYLINDER, 
                                               radius=link_radius, 
                                               height=link2_length)
        link2_visual = p.createVisualShape(p.GEOM_CYLINDER, 
                                         radius=link_radius, 
                                         length=link2_length,
                                         rgbaColor=[0.2, 0.8, 0.2, 1])  # Verde
        
        # Link 3: junta 2 para end-effector (efetuador)
        ee_collision = p.createCollisionShape(p.GEOM_BOX, 
                                            halfExtents=[0.03, 0.03, 0.02])
        ee_visual = p.createVisualShape(p.GEOM_BOX, 
                                      halfExtents=[0.03, 0.03, 0.02],
                                      rgbaColor=[0.2, 0.2, 0.8, 1])  # Azul
        
        # Criar multi-body
        link_masses = [link1_mass, link2_mass, 0.1]  # massa do efetuador
        link_collisions = [link1_collision, link2_collision, ee_collision]
        link_visuals = [link1_visual, link2_visual, ee_visual]
        
        # Posições dos links (centro de massa)
        link_positions = [
            [link1_length/2, 0, 0],  # Link 1
            [link1_length, 0, link2_length/2],  # Link 2
            [link1_length + link2_length, 0, 0]  # Efetuador
        ]
        
        link_orientations = [[0, 0, 0, 1]] * 3
        joint_types = [p.JOINT_REVOLUTE, p.JOINT_REVOLUTE]
        joint_axes = [0, 0, 1, 0, 0, 1]  # Achatado: [x1,y1,z1, x2,y2,z2]
        parent_indices = [0, 1]
        
        # Posições das juntas
        joint_positions = [
            [link1_length, 0, 0],  # Junta 1
            [link1_length + link2_length, 0, 0]  # Junta 2
        ]
        
        self.robot_id = p.createMultiBody(
            baseMass=10.0,  # Base pesada
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=-1,
            basePosition=[0, 0, 0],
            baseOrientation=[0, 0, 0, 1],
            linkMasses=link_masses,
            linkCollisionShapeIndices=link_collisions,
            linkVisualShapeIndices=link_visuals,
            linkPositions=link_positions,
            linkOrientations=link_orientations,
            linkJointTypes=joint_types,
            linkJointAxis=joint_axes,
            linkParentIndices=parent_indices
        )
        
        self.joint_ids = [0, 1]
        
        # Configurar limites das juntas
        joint_limits = self.config['robot']['joint_limits']
        for i, joint_id in enumerate(self.joint_ids):
            p.setJointMotorControl2(
                self.robot_id, joint_id,
                p.VELOCITY_CONTROL, force=0
            )
            # Reset para posição inicial
            p.resetJointState(self.robot_id, joint_id, 0.0)
        
        self.logger.info("Manipulador planar 2 DOF criado com sucesso")
        
        # Criar alvo para o robô alcançar
        self._create_target()
        
        # Sistema de detecção de obstáculos
        self._create_obstacles()
        
        # Estado do efetuador (gripper)
        self.gripper_state = "open"  # "open" ou "closed"
        self.held_object_id = None
    
    def _create_target(self) -> None:
        """Cria um objeto alvo para o robô alcançar."""
        # Criar uma esfera como alvo
        target_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=0.05)
        target_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, 
                                          rgbaColor=[1.0, 1.0, 0.0, 1])  # Amarelo
        
        # Posição inicial do alvo - MAIS PRÓXIMA E SEM OBSTÁCULOS NO CAMINHO
        target_pos = [0.3, 0.1, 0.0]  # Mais próximo e sem obstáculos
        
        self.target_id = p.createMultiBody(
            baseMass=0.1,  # Massa leve
            baseCollisionShapeIndex=target_collision,
            baseVisualShapeIndex=target_visual,
            basePosition=target_pos
        )
        
        self.target_position = target_pos
        self.logger.info(f"Alvo criado na posição {target_pos}")
    
    def _create_obstacles(self) -> None:
        """Cria obstáculos na cena."""
        self.obstacles = []
        
        # TEMPORARIAMENTE DESABILITANDO OBSTÁCULOS PARA FOCAR NO PICK & PLACE
        # Obstáculo 1: Parede vertical - DESABILITADO
        # wall_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.5, 0.3])
        # wall_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.5, 0.3],
        #                                 rgbaColor=[0.5, 0.5, 0.5, 1])  # Cinza
        # 
        # wall_id = p.createMultiBody(
        #     baseMass=0,  # Estático
        #     baseCollisionShapeIndex=wall_collision,
        #     baseVisualShapeIndex=wall_visual,
        #     basePosition=[0.7, 0.0, 0.3]  # Movido para mais longe
        # )
        # self.obstacles.append(wall_id)
        
        # Obstáculo 2: Caixa pequena - DESABILITADO
        # box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])
        # box_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1],
        #                                rgbaColor=[0.8, 0.4, 0.4, 1])  # Vermelho claro
        # 
        # box_id = p.createMultiBody(
        #     baseMass=0.5,
        #     baseCollisionShapeIndex=box_collision,
        #     baseVisualShapeIndex=box_visual,
        #     basePosition=[0.5, -0.3, 0.1]  # Movido para baixo e longe do caminho
        # )
        # self.obstacles.append(box_id)
        
        self.logger.info(f"{len(self.obstacles)} obstáculos criados (temporariamente desabilitados)")
    
    def detect_target(self) -> Optional[np.ndarray]:
        """
        Detecta a posição atual do alvo.
        
        Returns:
            Posição do alvo [x, y, z] ou None se não encontrado
        """
        try:
            # Obter posição do alvo
            pos, _ = p.getBasePositionAndOrientation(self.target_id)
            self.target_position = np.array(pos)
            return self.target_position
        except:
            return None
    
    def inverse_kinematics_2dof(self, target_pos: np.ndarray) -> Optional[np.ndarray]:
        """
        Resolve cinemática inversa para 2 DOF planar.
        
        Args:
            target_pos: Posição alvo [x, y]
            
        Returns:
            Ângulos das juntas [theta1, theta2] ou None se impossível
        """
        x, y = target_pos[0], target_pos[1]
        l1 = self.config['robot']['link1_length']
        l2 = self.config['robot']['link2_length']
        
        # Distância ao alvo
        r = np.sqrt(x**2 + y**2)
        
        # Verificar se é alcançável
        if r > (l1 + l2) or r < abs(l1 - l2):
            self.logger.warning(f"Posição {target_pos} não é alcançável")
            return None
        
        # Calcular theta2
        cos_theta2 = (r**2 - l1**2 - l2**2) / (2 * l1 * l2)
        cos_theta2 = np.clip(cos_theta2, -1, 1)  # Limitar para evitar erros numéricos
        theta2 = np.arccos(cos_theta2)
        
        # Calcular theta1
        k1 = l1 + l2 * np.cos(theta2)
        k2 = l2 * np.sin(theta2)
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
        
        # Normalizar ângulos para [-π, π]
        theta1 = np.arctan2(np.sin(theta1), np.cos(theta1))
        theta2 = np.arctan2(np.sin(theta2), np.cos(theta2))
        
        return np.array([theta1, theta2])
    
    def check_collision_avoidance(self, target_angles: np.ndarray) -> bool:
        """
        Verifica se a trajetória para os ângulos alvo colide com obstáculos.

        Args:
            target_angles: Ângulos alvo [theta1, theta2]

        Returns:
            True se há colisão, False se caminho livre
        """
        # Simular movimento para verificar colisões
        original_angles = self.current_angles.copy()

        # Testar pontos intermediários da trajetória
        steps = 10
        for i in range(steps + 1):
            t = i / steps
            test_angles = original_angles + t * (target_angles - original_angles)

            # Calcular posição do efetuador
            fk_result = self.kinematics.forward_kinematics(tuple(test_angles))
            ee_pos = fk_result['ee_position']

            # Verificar colisão com obstáculos (apenas coordenadas X,Y para robô planar)
            for obstacle_id in self.obstacles:
                # Obter AABB do obstáculo
                aabb_min, aabb_max = p.getAABB(obstacle_id)

                # Verificar se o efetuador está dentro da AABB (ignorar Z para robô planar)
                if (aabb_min[0] <= ee_pos[0] <= aabb_max[0] and
                    aabb_min[1] <= ee_pos[1] <= aabb_max[1]):
                    return True  # Colisão detectada

        return False  # Sem colisão
    
    def move_to_target(self) -> bool:
        """
        Move o robô para alcançar o alvo detectado.
        
        Returns:
            True se conseguiu alcançar, False caso contrário
        """
        target_pos = self.detect_target()
        if target_pos is None:
            self.logger.warning("Alvo não detectado")
            return False
        
        self.logger.info(f"Alvo detectado na posição: {target_pos}")
        
        # Resolver cinemática inversa (apenas coordenadas X,Y para robô planar)
        target_angles = self.inverse_kinematics_2dof(target_pos[:2])
        if target_angles is None:
            self.logger.warning("Posição do alvo não alcançável")
            return False
        
        self.logger.info(f"Ângulos calculados pela IK: {target_angles}")
        
        # Verificar colisões
        if self.check_collision_avoidance(target_angles):
            self.logger.warning("Trajetória colide com obstáculos")
            # Tentar posição alternativa (mais alta)
            alternative_pos = target_pos + np.array([0, 0, 0.2])
            alt_angles = self.inverse_kinematics_2dof(alternative_pos[:2])
            if alt_angles is not None and not self.check_collision_avoidance(alt_angles):
                target_angles = alt_angles
                self.logger.info("Usando trajetória alternativa")
            else:
                return False
        
        # Definir nova referência
        self.set_reference_angles(target_angles[0], target_angles[1])
        self.logger.info(f"Movendo para alvo em {target_pos[:2]} com ângulos {target_angles}")
        
        return True
    
    def grip_object(self) -> bool:
        """
        Tenta pegar o objeto alvo se estiver próximo.
        CRIA UM GRIPPER VISUAL E MOVE O OBJETO JUNTO COM O ROBÔ
        """
        if self.held_object_id is not None:
            self.logger.info("Já segurando um objeto")
            return True

        # Calcular posição do efetuador
        fk_result = self.kinematics.forward_kinematics(tuple(self.current_angles))
        ee_pos = np.array(fk_result['ee_position'])

        # Verificar distância ao alvo
        target_pos = self.detect_target()
        if target_pos is None:
            return False

        distance = np.linalg.norm(ee_pos - target_pos[:2])  # Comparar apenas X,Y

        if distance < 0.15:  # Distância de captura MAIS GENEROSA
            # Fechar gripper - CRIAR EFEITO VISUAL
            self.gripper_state = "closed"

            # CRIAR UM GRIPPER VISUAL (dedos)
            self._create_gripper_visual()

            # PEGAR O OBJETO REALMENTE - MOVER ELE JUNTO COM O ROBÔ
            self.held_object_id = self.target_id

            # Criar constraint FIXA para segurar o objeto firmemente
            self.gripper_constraint = p.createConstraint(
                parentBodyUniqueId=self.robot_id,
                parentLinkIndex=6,  # Último link do KUKA iiwa (end-effector)
                childBodyUniqueId=self.target_id,
                childLinkIndex=-1,
                jointType=p.JOINT_FIXED,
                jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, 0.1],  # Offset para frente do efetuador
                childFramePosition=[0, 0, 0]
            )

            # MUDAR COR DO OBJETO PARA MOSTRAR QUE FOI PEGO
            p.changeVisualShape(self.target_id, -1, rgbaColor=[0.0, 1.0, 0.0, 1.0])  # Verde = pego

            self.logger.info("🎉 OBJETO PEGO COM SUCESSO! O robô agora está segurando a bola!")
            return True
        else:
            self.logger.debug(f"Objeto muito distante: {distance:.3f}m (precisa ser < 0.15m)")
            return False
    
    def release_object(self) -> bool:
        """
        Solta o objeto segurado se houver um.
        REMOVE O GRIPPER VISUAL E LIBERA O OBJETO
        """
        if self.held_object_id is None:
            self.logger.info("Nenhum objeto para soltar")
            return False

        # Remover constraint
        if hasattr(self, 'gripper_constraint'):
            p.removeConstraint(self.gripper_constraint)
            delattr(self, 'gripper_constraint')

        # MUDAR COR DO OBJETO DE VOLTA PARA AMARELO (não pego)
        p.changeVisualShape(self.held_object_id, -1, rgbaColor=[1.0, 1.0, 0.0, 1.0])  # Amarelo = solto

        # Remover elementos visuais do gripper
        if hasattr(self, 'finger1_id'):
            p.removeBody(self.finger1_id)
            delattr(self, 'finger1_id')
        if hasattr(self, 'finger2_id'):
            p.removeBody(self.finger2_id)
            delattr(self, 'finger2_id')

        # Resetar estado
        self.held_object_id = None
        self.gripper_state = "open"

        self.logger.info("🎉 OBJETO SOLTO COM SUCESSO! O robô liberou a bola!")
        return True
    
    def _create_gripper_visual(self) -> None:
        """Cria elementos visuais para o gripper (dedos)."""
        # Obter posição do efetuador
        fk_result = self.kinematics.forward_kinematics(tuple(self.current_angles))
        ee_pos = fk_result['ee_position']
        
        # Para manipulador planar, adicionar coordenada Z = 0
        if len(ee_pos) == 2:
            ee_pos = np.array([ee_pos[0], ee_pos[1], 0.0])
        
        # Criar dois dedos do gripper
        finger_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.05])
        finger_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.05],
                                          rgbaColor=[0.8, 0.8, 0.8, 1])  # Cinza
        
        # Dedo esquerdo
        self.finger1_id = p.createMultiBody(
            baseMass=0.01,
            baseCollisionShapeIndex=finger_collision,
            baseVisualShapeIndex=finger_visual,
            basePosition=[ee_pos[0] - 0.03, ee_pos[1], ee_pos[2] + 0.05]
        )
        
        # Dedo direito
        self.finger2_id = p.createMultiBody(
            baseMass=0.01,
            baseCollisionShapeIndex=finger_collision,
            baseVisualShapeIndex=finger_visual,
            basePosition=[ee_pos[0] + 0.03, ee_pos[1], ee_pos[2] + 0.05]
        )
        
        self.logger.info("Gripper visual criado")
    
    def apply_external_force(self, force: np.ndarray) -> None:
        """
        Aplica força externa simulando perturbação ou peso diferente.
        
        Args:
            force: Vetor de força [Fx, Fy, Fz]
        """
        if self.held_object_id is not None:
            # Aplicar força no objeto segurado
            p.applyExternalForce(
                self.held_object_id, -1,
                forceObj=force,
                posObj=[0, 0, 0],
                flags=p.WORLD_FRAME
            )
            self.logger.debug(f"Força externa aplicada no objeto: {force}")
        else:
            # Aplicar força no efetuador
            fk_result = self.kinematics.forward_kinematics(tuple(self.current_angles))
            ee_pos = fk_result['ee_position']
            
            p.applyExternalForce(
                self.robot_id, 1,  # Último link
                forceObj=force,
                posObj=ee_pos,
                flags=p.WORLD_FRAME
            )
            self.logger.debug(f"Força externa aplicada no efetuador: {force}")
    
    def _create_planar_robot(self) -> None:
        """Cria um manipulador planar 2 DOF usando URDF."""
        import pybullet_data
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Carregar um robô simples do PyBullet
        self.robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])
        
        # Para simplificar, vamos usar apenas as primeiras 2 juntas
        self.joint_ids = [0, 1]  # Usar apenas 2 juntas do kuka iiwa
        
        # Fixar as outras juntas para evitar interferência
        for i in range(p.getNumJoints(self.robot_id)):
            if i not in self.joint_ids:
                p.setJointMotorControl2(self.robot_id, i, p.POSITION_CONTROL, targetPosition=0, force=0)
                p.resetJointState(self.robot_id, i, 0)
        
    def _create_target(self) -> None:
        """Cria o objeto alvo (bola amarela) para o robô alcançar."""
        # Criar uma esfera como alvo
        target_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=0.05)
        target_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.05,
                                          rgbaColor=[1.0, 1.0, 0.0, 1.0])  # Amarelo
        
        self.target_id = p.createMultiBody(
            baseMass=0.1,  # Massa leve
            baseCollisionShapeIndex=target_collision,
            baseVisualShapeIndex=target_visual,
            basePosition=[0.3, 0.1, 0.0]  # Posição alvo
        )
        
        self.logger.info("Alvo criado: bola amarela em [0.3, 0.1, 0.0]")
        
        # Estado do efetuador (gripper)
        self.gripper_state = "open"  # "open" ou "closed"
        self.held_object_id = None
    
    def _setup_pid_controllers(self) -> DualJointPIDController:
        """Cria controladores PID para ambas as juntas."""
        config_j1 = PIDConfig(
            kp=self.config['pid_controller']['joint1']['kp'],
            ki=self.config['pid_controller']['joint1']['ki'],
            kd=self.config['pid_controller']['joint1']['kd'],
            integral_max=self.config['pid_controller']['joint1']['integral_max'],
            integral_min=self.config['pid_controller']['joint1']['integral_min'],
            max_output=self.config['pid_controller']['max_torque']
        )
        
        config_j2 = PIDConfig(
            kp=self.config['pid_controller']['joint2']['kp'],
            ki=self.config['pid_controller']['joint2']['ki'],
            kd=self.config['pid_controller']['joint2']['kd'],
            integral_max=self.config['pid_controller']['joint2']['integral_max'],
            integral_min=self.config['pid_controller']['joint2']['integral_min'],
            max_output=self.config['pid_controller']['max_torque']
        )
        
        return DualJointPIDController(config_j1, config_j2)
    
    def _setup_node_red_client(self) -> Optional[NodeREDClient]:
        """Cria client para comunicação com Node-RED."""
        if not self.config['node_red']['enabled']:
            self.logger.info("Integração Node-RED desabilitada")
            return None
        
        client = NodeREDClient(
            host=self.config['node_red']['host'],
            port=self.config['node_red']['port'],
            endpoint=self.config['node_red']['endpoint'],
            protocol=self.config['node_red']['protocol'],
            timeout=self.config['node_red']['timeout']
        )
        
        if client.connect():
            self.logger.info("Cliente Node-RED conectado")
        else:
            self.logger.warning("Falha ao conectar Node-RED (continuando sem integração)")
        
        return client
    
    def set_reference_angles(self, theta1: float, theta2: float) -> None:
        """
        Define ângulos de referência para as juntas.
        
        Args:
            theta1: Ângulo de referência para junta 1 (radianos)
            theta2: Ângulo de referência para junta 2 (radianos)
        """
        self.reference_angles = np.array([theta1, theta2])
        self.logger.debug(f"Referência definida: theta1={theta1:.3f}, theta2={theta2:.3f}")
    
    def apply_perturbation(self) -> None:
        """Aplica perturbação externa ao manipulador."""
        if not self.config['perturbation']['enabled']:
            return
        
        if not self.perturbation_active:
            self.perturbation_active = True
            self.perturbation_end_time = self.current_time + self.config['perturbation']['force_duration']
            
            force = self.config['perturbation']['force_magnitude']
            point = self.config['perturbation']['force_application_point']
            
            p.applyExternalForce(
                self.robot_id, -1,
                forceObj=force,
                posObj=point,
                flags=p.WORLD_FRAME
            )
            
            self.logger.info(f"Perturbação aplicada: F={force}N em {point}m")
    
    def step(self, dt: float) -> None:
        """
        Executa um passo de simulação.
        
        Args:
            dt: Intervalo de tempo (segundos)
        """
        # Calcular erros
        errors = self.reference_angles - self.current_angles
        
        # Atualizar controladores PID
        torques = self.pid_controller.update(tuple(errors), dt)
        
        # DEBUG: Log dos torques (agora usado apenas para métricas)
        self.logger.debug(f"Torques calculados: {torques}")
        
        # Aplicar controle de POSIÇÃO nas juntas (mais confiável para URDF)
        if len(self.joint_ids) >= 2:
            p.setJointMotorControl2(
                self.robot_id, self.joint_ids[0],
                p.POSITION_CONTROL, 
                targetPosition=self.reference_angles[0],
                force=1000  # Aumentar força para KUKA iiwa
            )
            p.setJointMotorControl2(
                self.robot_id, self.joint_ids[1],
                p.POSITION_CONTROL,
                targetPosition=self.reference_angles[1], 
                force=1000  # Aumentar força para KUKA iiwa
            )
            self.logger.debug(f"Posições alvo aplicadas: {self.reference_angles}")
        else:
            self.logger.error(f"Juntas insuficientes: {len(self.joint_ids)} disponíveis, 2 necessárias")
        
        # Simular um passo
        p.stepSimulation()
        
        # Atualizar estado do robô
        self._update_robot_state()
        
        # Coletar dados
        self._collect_metrics(torques)
        
        # Enviar para Node-RED
        if self.node_red_client and self.data_aggregator.should_send(self.current_time):
            self._send_to_node_red(torques)
        
        # Atualizar tempo
        self.current_time += dt
    
    def _update_robot_state(self) -> None:
        """Atualiza ângulos e velocidades das juntas."""
        if len(self.joint_ids) >= 2:
            # Obter estados das juntas
            joint1_state = p.getJointState(self.robot_id, self.joint_ids[0])
            joint2_state = p.getJointState(self.robot_id, self.joint_ids[1])
            
            self.current_angles[0] = joint1_state[0]
            self.current_angles[1] = joint2_state[0]
            self.current_velocities[0] = joint1_state[1]
            self.current_velocities[1] = joint2_state[1]
    
    def _collect_metrics(self, torques: Tuple[float, float]) -> None:
        """Coleta métricas de desempenho."""
        if not self.config['data_logging']['enabled']:
            return
        
        errors = self.reference_angles - self.current_angles
        
        # Cinemática direta
        fk_result = self.kinematics.forward_kinematics(tuple(self.current_angles))
        ee_pos = fk_result['ee_position']
        ee_dist = fk_result['ee_distance']
        
        # Armazenar dados
        self.metrics_history['time'].append(self.current_time)
        self.metrics_history['joint1_angle'].append(self.current_angles[0])
        self.metrics_history['joint2_angle'].append(self.current_angles[1])
        self.metrics_history['joint1_ref'].append(self.reference_angles[0])
        self.metrics_history['joint2_ref'].append(self.reference_angles[1])
        self.metrics_history['joint1_error'].append(errors[0])
        self.metrics_history['joint2_error'].append(errors[1])
        self.metrics_history['joint1_torque'].append(torques[0])
        self.metrics_history['joint2_torque'].append(torques[1])
        self.metrics_history['ee_x'].append(ee_pos[0])
        self.metrics_history['ee_y'].append(ee_pos[1])
        self.metrics_history['ee_distance'].append(ee_dist)
    
    def _send_to_node_red(self, torques: Tuple[float, float]) -> None:
        """Envia dados para Node-RED."""
        if not self.node_red_client or not self.node_red_client.is_connected:
            return
        
        errors = self.reference_angles - self.current_angles
        fk_result = self.kinematics.forward_kinematics(tuple(self.current_angles))
        
        data = self.data_aggregator.create_data_packet(
            timestamp=self.current_time,
            joint_angles=tuple(self.current_angles),
            reference_angles=tuple(self.reference_angles),
            errors=tuple(errors),
            torques=torques,
            ee_position=tuple(fk_result['ee_position']),
            ee_distance=fk_result['ee_distance']
        )
        
        self.node_red_client.send_data_async(data)
    
    def save_metrics(self) -> str:
        """
        Salva métricas em arquivo CSV.
        
        Returns:
            Caminho do arquivo salvo
        """
        if not self.metrics_history['time']:
            self.logger.warning("Nenhuma métrica para salvar")
            return None
        
        log_dir = self.config['data_logging']['log_directory']
        filename = os.path.join(
            log_dir,
            f"metrics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        
        try:
            with open(filename, 'w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=self.metrics_history.keys())
                writer.writeheader()
                
                for i in range(len(self.metrics_history['time'])):
                    row = {}
                    for key in self.metrics_history.keys():
                        row[key] = self.metrics_history[key][i]
                    writer.writerow(row)
            
            self.logger.info(f"Métricas salvas em {filename}")
            return filename
        except Exception as e:
            self.logger.error(f"Erro ao salvar métricas: {e}")
            return None
    
    def calculate_performance_metrics(self) -> Dict[str, any]:
        """
        Calcula métricas de desempenho do controlador.
        
        Returns:
            Dicionário com métricas calculadas
        """
        metrics = {
            'settling_time_j1': None,
            'settling_time_j2': None,
            'overshoot_j1': None,
            'overshoot_j2': None,
            'steady_state_error_j1': None,
            'steady_state_error_j2': None,
            'energy_j1': 0.0,
            'energy_j2': 0.0
        }
        
        if not self.metrics_history['time']:
            return metrics
        
        times = np.array(self.metrics_history['time'])
        errors_j1 = np.array(self.metrics_history['joint1_error'])
        errors_j2 = np.array(self.metrics_history['joint2_error'])
        torques_j1 = np.array(self.metrics_history['joint1_torque'])
        torques_j2 = np.array(self.metrics_history['joint2_torque'])
        
        threshold = self.config['metrics']['settling_time_threshold']
        
        # Tempo de acomodação (settling time)
        for i, error in enumerate(errors_j1):
            if abs(error) < threshold and i > 0:
                metrics['settling_time_j1'] = times[i]
                break
        
        for i, error in enumerate(errors_j2):
            if abs(error) < threshold and i > 0:
                metrics['settling_time_j2'] = times[i]
                break
        
        # Erro em estado estacionário
        if len(errors_j1) > 0:
            metrics['steady_state_error_j1'] = float(errors_j1[-1])
        if len(errors_j2) > 0:
            metrics['steady_state_error_j2'] = float(errors_j2[-1])
        
        # Energia consumida (integral do torque ao quadrado)
        if len(times) > 1:
            dt = np.diff(times)
            metrics['energy_j1'] = float(np.sum(torques_j1[:-1]**2 * dt))
            metrics['energy_j2'] = float(np.sum(torques_j2[:-1]**2 * dt))
        
        return metrics
    
    def reset(self) -> None:
        """Reseta a simulação."""
        self.pid_controller.reset()
        self.current_time = 0.0
        self.reference_angles = np.array([0.0, 0.0])
        self.current_angles = np.array([0.0, 0.0])
        self.current_velocities = np.array([0.0, 0.0])
        self.perturbation_active = False
        
        self.metrics_history = {
            'time': [],
            'joint1_angle': [],
            'joint2_angle': [],
            'joint1_ref': [],
            'joint2_ref': [],
            'joint1_error': [],
            'joint2_error': [],
            'joint1_torque': [],
            'joint2_torque': [],
            'ee_x': [],
            'ee_y': [],
            'ee_distance': []
        }
        
        self.logger.info("Simulação resetada")
    
    def shutdown(self) -> None:
        """Encerra simulação e libera recursos."""
        if self.node_red_client:
            self.node_red_client.disconnect()
        
        p.disconnect(self.physics_client)
        self.logger.info("Simulação encerrada")

    def run_simulation(self, duration: float, description: str) -> None:
        """
        Executa simulação por tempo determinado.
        
        Args:
            duration: Duração em segundos
            description: Descrição da fase
        """
        steps = int(duration / self.config['simulation']['time_step'])
        
        print(f"\n{description} ({duration}s)...")
        for step in range(steps):
            # Aplicar perturbação se ativa
            self.apply_perturbation()
            
            self.step(self.config['simulation']['time_step'])
            
            # Imprimir progresso a cada 100 passos
            if step % 100 == 0:
                errors = self.reference_angles - self.current_angles
                fk_result = self.kinematics.forward_kinematics(tuple(self.current_angles))
                ee_pos = fk_result['ee_position']
                
                print(f"Passo {step}/{steps} | "
                      f"θ1={self.current_angles[0]:.3f} θ2={self.current_angles[1]:.3f} | "
                      f"Erro1={errors[0]:.3f} Erro2={errors[1]:.3f} | "
                      f"EE: [{ee_pos[0]:.2f}, {ee_pos[1]:.2f}] | "
                      f"Gripper: {self.gripper_state}")


def main():
    """Função principal para demonstração completa do manipulador planar."""
    print("=" * 70)
    print("SIMULAÇÃO DO MANIPULADOR PLANAR 2 DOF COM SISTEMA DE PICK & PLACE")
    print("=" * 70)
    
    # Criar simulação
    sim = RobotSimulation(enable_gui=True)
    
    try:
        print("\n=== FASE 1: MOVIMENTO PARA POSIÇÃO INICIAL ===")
        # Mover para posição inicial
        sim.set_reference_angles(0.0, 0.0)
        sim.run_simulation(3.0, "Movendo para posição inicial")
        
        print("\n=== FASE 2: DETECÇÃO E ALCANCE DO ALVO ===")
        # Detectar e mover para o alvo
        if sim.move_to_target():
            print("Alvo detectado! Movendo o robô...")
            sim.run_simulation(5.0, "Alcançando o alvo")
        else:
            print("Não foi possível alcançar o alvo")
        
        print("\n=== FASE 3: PEGAR O OBJETO ===")
        # Tentar pegar o objeto
        if sim.grip_object():
            print("Objeto pego com sucesso!")
            
            print("\n=== FASE 4: MOVER OBJETO PARA NOVA POSIÇÃO ===")
            # Mover o objeto para uma nova posição
            sim.set_reference_angles(np.pi/3, -np.pi/4)  # Nova posição
            sim.run_simulation(4.0, "Movendo objeto pego")
            
            print("\n=== FASE 5: SOLTAR OBJETO ===")
            sim.release_object()
            print("Objeto solto!")
            
            print("\n=== FASE 6: TESTE DE PERTURBAÇÃO ===")
            # Aplicar perturbação para testar robustez
            sim.apply_perturbation()
            sim.run_simulation(3.0, "Testando resposta a perturbação")
            
        else:
            print("Não foi possível pegar o objeto")
        
        print("\n=== FASE 7: RETORNO À POSIÇÃO INICIAL ===")
        sim.set_reference_angles(0.0, 0.0)
        sim.run_simulation(3.0, "Retornando à posição inicial")
        
        # Calcular e exibir métricas completas
        print("\n" + "=" * 70)
        print("MÉTRICAS DE DESEMPENHO COMPLETAS")
        print("=" * 70)
        
        metrics = sim.calculate_performance_metrics()
        for key, value in metrics.items():
            if value is not None:
                if isinstance(value, float):
                    print(f"{key:.<45} {value:.6f}")
                else:
                    print(f"{key:.<45} {value}")
            else:
                print(f"{key:.<45} N/A")
        
        # Salvar dados
        csv_file = sim.save_metrics()
        print(f"\nDados salvos em: {csv_file}")
        
        # Visualização final
        if sim.enable_gui:
            print("\nVisualização final (30 segundos)...")
            print("Feche a janela do PyBullet para finalizar")
            for _ in range(3000):
                p.stepSimulation()
    
    except KeyboardInterrupt:
        print("\nSimulação interrompida pelo usuário")
    
    finally:
        sim.shutdown()
        print("\nPrograma finalizado")


if __name__ == "__main__":
    main()

