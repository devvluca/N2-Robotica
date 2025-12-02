"""
Robô Aspirador Diferencial com Sensores Ultrassônicos.
Usa modelo URDF para aparência realista.
"""

import pybullet as p
import numpy as np
import math
import os


class VacuumRobot:
    """Robô aspirador diferencial com sensores ultrassônicos."""
    
    def __init__(self, physics_client, start_pos=[0, 0], start_angle=0):
        self.client = physics_client
        self.start_pos = start_pos
        self.start_angle = start_angle
        
        # Parâmetros do robô
        self.radius = 0.16  # Raio do robô (32cm diâmetro - como um Roomba)
        self.height = 0.08  # Altura
        self.wheel_radius = 0.03
        self.axle_length = 0.24  # Distância entre rodas
        
        # Parâmetros dos sensores (5 sensores ultrassônicos)
        self.num_sensors = 5
        self.sensor_range = 1.2  # Alcance máximo (1.2 metros)
        self.sensor_angles = [-90, -45, 0, 45, 90]  # Graus relativos à frente
        
        # Parâmetros de controle - VELOCIDADE ALTA
        self.max_velocity = 1.5  # m/s (bem rápido!)
        self.max_angular_velocity = 4.0  # rad/s
        
        # Estado
        self.position = np.array([start_pos[0], start_pos[1], self.height/2])
        self.orientation = start_angle
        self.velocity = [0, 0]  # [left_wheel, right_wheel]
        
        # Criar o robô
        self.robot_id = self._create_robot()
        
        # Contadores de energia
        self.energy_consumed = 0.0
        self.distance_traveled = 0.0
        self.last_position = np.array(start_pos)
        
    def _create_robot(self):
        """Cria o robô aspirador com visual mais realista."""
        # Corpo principal - disco achatado como um Roomba
        body_collision = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=self.radius,
            height=self.height
        )
        body_visual = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=self.radius,
            length=self.height,
            rgbaColor=[0.2, 0.2, 0.2, 1]  # Cinza escuro (corpo principal)
        )
        
        # Criar corpo base
        robot_id = p.createMultiBody(
            baseMass=2.0,
            baseCollisionShapeIndex=body_collision,
            baseVisualShapeIndex=body_visual,
            basePosition=[self.start_pos[0], self.start_pos[1], self.height/2 + 0.005],
            baseOrientation=p.getQuaternionFromEuler([0, 0, self.start_angle])
        )
        
        # Adicionar tampa superior (visual apenas) - verde como iRobot
        top_visual = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=self.radius * 0.85,
            length=0.02,
            rgbaColor=[0.2, 0.6, 0.3, 1]  # Verde iRobot
        )
        p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=top_visual,
            basePosition=[self.start_pos[0], self.start_pos[1], self.height + 0.01]
        )
        
        # Botão central (visual)
        button_visual = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=0.03,
            length=0.015,
            rgbaColor=[0.9, 0.9, 0.9, 1]  # Branco
        )
        p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=button_visual,
            basePosition=[self.start_pos[0], self.start_pos[1], self.height + 0.025]
        )
        
        # Indicador de direção (frente) - barra laranja
        front_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.02, 0.06, 0.015],
            rgbaColor=[1.0, 0.5, 0.0, 1]  # Laranja
        )
        p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=front_visual,
            basePosition=[self.start_pos[0] + self.radius * 0.7, self.start_pos[1], self.height/2 + 0.02]
        )
        
        # Sensor frontal (visual) - preto
        sensor_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.01, 0.08, 0.02],
            rgbaColor=[0.1, 0.1, 0.1, 1]  # Preto
        )
        p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=sensor_visual,
            basePosition=[self.start_pos[0] + self.radius * 0.95, self.start_pos[1], self.height/2]
        )
        
        # Configurar física para bom movimento
        p.changeDynamics(robot_id, -1, 
                        lateralFriction=0.4,
                        spinningFriction=0.02,
                        rollingFriction=0.01,
                        linearDamping=0.05,
                        angularDamping=0.05)
        
        print(f"[ROBÔ] Criado na posição ({self.start_pos[0]}, {self.start_pos[1]})")
        
        return robot_id
    
    def _create_simple_robot(self):
        """Cria robô com geometria simples (fallback)."""
        collision_shape = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=self.radius,
            height=self.height
        )
        visual_shape = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=self.radius,
            length=self.height,
            rgbaColor=[0.2, 0.6, 0.8, 1]
        )
        
        robot_id = p.createMultiBody(
            baseMass=2.5,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=[self.start_pos[0], self.start_pos[1], self.height/2 + 0.01],
            baseOrientation=p.getQuaternionFromEuler([0, 0, self.start_angle])
        )
        
        return robot_id
    
    def _find_wheel_joints(self):
        """Encontra os índices das juntas das rodas."""
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            if 'left_wheel' in joint_name:
                self.left_wheel_joint = i
            elif 'right_wheel' in joint_name:
                self.right_wheel_joint = i
        
        if self.left_wheel_joint is not None:
            print(f"[ROBÔ] Juntas encontradas: esquerda={self.left_wheel_joint}, direita={self.right_wheel_joint}")
    
    def get_sensor_readings(self):
        """
        Lê os 5 sensores ultrassônicos.
        Retorna distâncias normalizadas [0-1] onde 1 = sensor_range.
        """
        readings = []
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(orn)
        robot_angle = euler[2]
        
        for sensor_angle_deg in self.sensor_angles:
            sensor_angle_rad = math.radians(sensor_angle_deg)
            total_angle = robot_angle + sensor_angle_rad
            
            # Ponto de origem (na borda do robô)
            ray_start = [
                pos[0] + self.radius * math.cos(total_angle),
                pos[1] + self.radius * math.sin(total_angle),
                pos[2] + 0.03  # Altura dos sensores
            ]
            
            # Ponto final do raio
            ray_end = [
                pos[0] + (self.radius + self.sensor_range) * math.cos(total_angle),
                pos[1] + (self.radius + self.sensor_range) * math.sin(total_angle),
                pos[2] + 0.03
            ]
            
            # Raycast
            result = p.rayTest(ray_start, ray_end)
            
            if result[0][0] != -1 and result[0][0] != self.robot_id:
                # Objeto detectado
                hit_fraction = result[0][2]
                distance = hit_fraction * self.sensor_range
            else:
                # Nada detectado
                distance = self.sensor_range
            
            readings.append(distance / self.sensor_range)  # Normalizado
        
        return np.array(readings)
    
    def get_pose(self):
        """Retorna posição (x, y) e orientação (theta) do robô."""
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(orn)
        return np.array([pos[0], pos[1]]), euler[2]
    
    def set_wheel_velocities(self, left_vel, right_vel):
        """
        Define velocidades das rodas (controle diferencial).
        
        Args:
            left_vel: Velocidade da roda esquerda (-1 a 1)
            right_vel: Velocidade da roda direita (-1 a 1)
        """
        # Limitar velocidades
        left_vel = np.clip(left_vel, -1, 1) * self.max_velocity
        right_vel = np.clip(right_vel, -1, 1) * self.max_velocity
        
        self.velocity = [left_vel, right_vel]
        
        # Calcular velocidade linear e angular do robô
        linear_vel = (left_vel + right_vel) / 2
        angular_vel = (right_vel - left_vel) / self.axle_length
        
        # Limitar velocidade angular
        angular_vel = np.clip(angular_vel, -self.max_angular_velocity, self.max_angular_velocity)
        
        # Obter orientação atual
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(orn)
        theta = euler[2]
        
        # Velocidade no frame global
        vx = linear_vel * math.cos(theta)
        vy = linear_vel * math.sin(theta)
        
        # Aplicar velocidade diretamente à base (método mais confiável)
        p.resetBaseVelocity(
            self.robot_id,
            linearVelocity=[vx, vy, 0],
            angularVelocity=[0, 0, angular_vel]
        )
        
        # Calcular energia consumida
        power = abs(left_vel) + abs(right_vel)
        self.energy_consumed += power * (1/240)
    
    def move_forward(self, speed=0.7):
        """Move o robô para frente."""
        self.set_wheel_velocities(speed, speed)
    
    def turn_left(self, speed=0.5):
        """Vira o robô para a esquerda."""
        self.set_wheel_velocities(-speed, speed)
    
    def turn_right(self, speed=0.5):
        """Vira o robô para a direita."""
        self.set_wheel_velocities(speed, -speed)
    
    def stop(self):
        """Para o robô."""
        self.set_wheel_velocities(0, 0)
    
    def update_metrics(self):
        """Atualiza métricas de distância percorrida."""
        pos, _ = self.get_pose()
        dist = np.linalg.norm(pos - self.last_position)
        self.distance_traveled += dist
        self.last_position = pos.copy()
    
    def get_metrics(self):
        """Retorna métricas do robô."""
        return {
            'energy_consumed': self.energy_consumed,
            'distance_traveled': self.distance_traveled
        }
    
    def reset(self):
        """Reseta o robô para a posição inicial."""
        p.resetBasePositionAndOrientation(
            self.robot_id,
            [self.start_pos[0], self.start_pos[1], 0.0],
            p.getQuaternionFromEuler([0, 0, self.start_angle])
        )
        p.resetBaseVelocity(self.robot_id, [0, 0, 0], [0, 0, 0])
        self.energy_consumed = 0.0
        self.distance_traveled = 0.0
        self.last_position = np.array(self.start_pos)
