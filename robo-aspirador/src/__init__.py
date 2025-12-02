"""
Módulo src do Robô Aspirador Inteligente.
"""

from .robot import VacuumRobot
from .environment import Environment
from .mapping import OccupancyMap, CELL_UNKNOWN, CELL_FREE, CELL_DIRTY, CELL_OBSTACLE
from .controller import NavigationController, SmartNavigationController, RobotState
from . import node_red_client

__all__ = [
    'VacuumRobot',
    'Environment', 
    'OccupancyMap',
    'CELL_UNKNOWN', 'CELL_FREE', 'CELL_DIRTY', 'CELL_OBSTACLE',
    'NavigationController', 'SmartNavigationController', 'RobotState',
    'node_red_client'
]
