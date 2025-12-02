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

    # Obstacle removed temporarily for debugging
    return tray_pos, -1

def create_random_cube():
    """Spawn a single cube in front of the arm's hand (extended position)."""
    # Braço estendido: mão em ~(1.4, 0). Cubo deve nascer perto da mão.
    # Links reais: 0.5 + 0.5 + 0.4 = 1.4m de alcance
    x = random.uniform(1.1, 1.3)  # Na frente, perto do alcance da mão (~1.4m)
    y = random.uniform(-0.15, 0.15)  # Pequena variação lateral
    pos = [x, y, 0.0]
    print(f'[SPAWN] Cubo criado em X={x:.3f}, Y={y:.3f}')
    orn = p.getQuaternionFromEuler([0, 0, 0])
    cube_id = p.loadURDF("cube_small.urdf", pos, orn)
    # Random color for visual distinction
    color = [random.random(), random.random(), random.random(), 1.0]
    p.changeVisualShape(cube_id, -1, rgbaColor=color)
    return cube_id, pos

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