import pybullet as p
import pybullet_data
import random
import math

def setup_simulation(gui=True):
    if gui:
        p.connect(p.GUI)
        # Set camera for better view: isometric view showing whole arm and table
        p.resetDebugVisualizerCamera(cameraDistance=2.0, 
                                      cameraPitch=-45.0, 
                                      cameraYaw=45.0, 
                                      cameraTargetPosition=[0.5, 0.0, 0.0])
    else:
        p.connect(p.DIRECT)
    
    p.setGravity(0, 0, -9.8)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Ground plane
    p.loadURDF("plane.urdf")

    # Table removed as requested
    # table_pos = [0.5, 0.0, 0.0]
    # p.loadURDF("table/table.urdf", table_pos, p.getQuaternionFromEuler([0, 0, 0]))

    # Target tray removed as requested
    tray_pos = [1.0, 0.50, 0.1]  # Position within arm reach (~1.12m from origin)
    # tray_orn = p.getQuaternionFromEuler([0, 0, 0])
    # p.loadURDF("tray/tray.urdf", tray_pos, tray_orn)

    # Obstacle - cubo vermelho como obstáculo
    # Posicionado diretamente no caminho para a tray (1.0, 0.5)
    obstacle_pos = [1.0, 0.30, 0.0]  # Bem no caminho
    obstacle_id = p.loadURDF("cube_small.urdf", obstacle_pos, p.getQuaternionFromEuler([0, 0, 0]))
    p.changeVisualShape(obstacle_id, -1, rgbaColor=[1.0, 0.2, 0.2, 1.0])  # Vermelho
    # Escalar o obstáculo para ser maior e mais visível
    # Tornar obstáculo estático (massa = 0)
    p.changeDynamics(obstacle_id, -1, mass=0)
    print(f'[SIMULAÇÃO] Obstáculo criado em ({obstacle_pos[0]:.2f}, {obstacle_pos[1]:.2f})')
    
    return tray_pos, obstacle_id

# Contador global para alternar posição do cubo
_cube_spawn_counter = 0

def create_random_cube():
    """Spawn cubo alternando entre lado de dentro e lado de fora do obstáculo."""
    global _cube_spawn_counter
    
    x = random.uniform(1.0, 1.2)
    
    # Alternar: par = dentro (Y negativo), ímpar = fora (Y positivo, após obstáculo)
    if _cube_spawn_counter % 2 == 0:
        # Lado de DENTRO (Y negativo) - antes do obstáculo
        y = random.uniform(-0.15, -0.05)
        lado = "DENTRO (antes do obstáculo)"
    else:
        # Lado de FORA (Y positivo) - logo após o obstáculo em Y=0.3
        # Posição acessível: Y entre 0.35 e 0.40
        y = random.uniform(0.35, 0.40)
        lado = "FORA (depois do obstáculo)"
    
    _cube_spawn_counter += 1
    
    pos = [x, y, 0.025]
    print(f'[SPAWN] Cubo criado em X={x:.3f}, Y={y:.3f} - {lado}')
    orn = p.getQuaternionFromEuler([0, 0, 0])
    cube_id = p.loadURDF("cube_small.urdf", pos, orn)
    p.changeVisualShape(cube_id, -1, rgbaColor=[0.2, 0.4, 1.0, 1.0])
    return cube_id, pos[:2]

def remove_body(body_id):
    try:
        p.removeBody(body_id)
    except Exception:
        pass

def step_many(steps, sleep_time=1/240):
    for _ in range(steps):
        p.stepSimulation()
        import time
        time.sleep(sleep_time)