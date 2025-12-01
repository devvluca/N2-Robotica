import argparse
import time
import math
from src.kinematics import PlanarArmKinematics
from src.arm import PlanarArm
from src.control import ArmController
from src.simulation import setup_simulation, create_random_cube, remove_body, step_many
import pybullet as p

def calculate_distance(pos1, pos2):
    """Calculate 2D distance between two positions."""
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

def approach_cube_smoothly(controller, cube_pos, cube_id, max_distance=0.08):
    """Move arm smoothly towards cube directly with IK."""
    print(f'\n[BUSCA] ===== ABORDAGEM AO CUBO =====')
    print(f'[BUSCA] Alvo: ({cube_pos[0]:.3f}, {cube_pos[1]:.3f})')
    
    # Calcular IK direto para o cubo
    print(f'[BUSCA] → Calculando IK...')
    ik = controller.kin.inverse_kinematics(cube_pos[0], cube_pos[1])
    
    if ik is None:
        print(f'[BUSCA] ✗ IK IMPOSSÍVEL')
        return False
    
    print(f'[BUSCA] ✓ IK OK: θ1={ik[0]:.3f}, θ2={ik[1]:.3f}, θ3={ik[2]:.3f}')
    
    # Movimento suave ramped com PID
    print(f'[BUSCA] → Movimento suave (ramped 1.0s)...')
    
    # Log antes
    angles_before = controller.arm.get_joint_angles()
    print(f'[BUSCA] - Ângulos antes: θ1={angles_before[0]:.3f}, θ2={angles_before[1]:.3f}, θ3={angles_before[2]:.3f}')
    
    controller.arm.ramped_move(ik, duration=1.0)
    step_many(60)
    
    # Verificar distância
    ee_pos, _ = controller.arm.get_ee_position()
    distance = calculate_distance([ee_pos[0], ee_pos[1]], cube_pos)
    angles = controller.arm.get_joint_angles()
    
    # Calcular FK usando kinematics
    fk_pos = controller.kin.forward_kinematics(angles)
    print(f'[BUSCA] Resultado:')
    print(f'[BUSCA] - PyBullet EE: ({ee_pos[0]:.3f}, {ee_pos[1]:.3f})')
    print(f'[BUSCA] - FK calculado: ({fk_pos[0]:.3f}, {fk_pos[1]:.3f})')
    print(f'[BUSCA] - Ângulos: θ1={angles[0]:.3f}, θ2={angles[1]:.3f}, θ3={angles[2]:.3f}')
    print(f'[BUSCA] - Distância: {distance:.3f}m (threshold: {max_distance:.3f}m)')
    
    if distance < max_distance:
        print(f'[BUSCA] ✓✓✓ SUCESSO!')
        return True
    else:
        print(f'[BUSCA] ⚠ Tentando agarrar mesmo assim...')
        return True

def run(doF=3, cycles=6, gui=True):
    if doF == 2:
        links = [0.2, 0.2]
        urdf = './models/planar_arm_2dof.urdf'
    else:
        links = [0.2, 0.2, 0.15]
        urdf = './models/planar_arm_3dof.urdf'

    tray_pos, obstacle_id = setup_simulation(gui)
    arm = PlanarArm(urdf, [0,0,0.0], [0,0,0], links, num_dofs=len(links))
    arm.load()
    kin = PlanarArmKinematics(links)
    controller = ArmController(arm, kin)

    # spawn limits to ensure IK reach
    reach = sum(links)
    x_range = (0.2, min(1.0, reach - 0.05))
    y_range = (-min(0.4, reach - 0.05), min(0.4, reach - 0.05))
    counter = 0
    try:
        while counter < cycles:
            print(f'\n[SIMULAÇÃO] === Iniciando ciclo {counter + 1}/{cycles} ===')
            cube_id, cube_pos = create_random_cube()
            print(f'[SIMULAÇÃO] Cubo criado na posição: ({cube_pos[0]:.2f}, {cube_pos[1]:.2f})')
            
            # Aproximação suave ao cubo
            if approach_cube_smoothly(controller, cube_pos, cube_id, max_distance=0.08):
                controller.grasp(cube_id)
                step_many(30)
                print(f'[SIMULAÇÃO] Cubo agarrado! Levantando...')
                controller.avoid_and_move(cube_pos[0], cube_pos[1]+0.3, duration=1.0)
                step_many(60)
                print(f'[SIMULAÇÃO] Movendo para destino (tray)...')
                controller.avoid_and_move(tray_pos[0], tray_pos[1], duration=1.5)
                step_many(60)
                controller.release(cube_id)
                print(f'[SIMULAÇÃO] Cubo solto!')
            else:
                print(f'[SIMULAÇÃO] AVISO: Não conseguiu chegar perto do cubo em tempo hábil.')
            
            remove_body(cube_id)
            controller.log_metrics()
            counter += 1
            print(f'[SIMULAÇÃO] Ciclo {counter}/{cycles} concluído\n')
            time.sleep(0.5)
    finally:
        import pybullet as p
        p.disconnect()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--dof', type=int, default=3, choices=[2,3])
    parser.add_argument('--cycles', type=int, default=6)
    parser.add_argument('--nogui', action='store_true')
    args = parser.parse_args()
    run(doF=args.dof, cycles=args.cycles, gui=(not args.nogui))

# End of file. Use the run() entrypoint above when invoked from CLI.
