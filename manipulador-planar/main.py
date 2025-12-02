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

def approach_cube_smoothly(controller, cube_pos, cube_id, max_distance=0.10, max_iterations=20):
    """Move arm smoothly towards cube using iterative approach."""
    print(f'\n[BUSCA] ===== ABORDAGEM AO CUBO =====')
    print(f'[BUSCA] Alvo: ({cube_pos[0]:.3f}, {cube_pos[1]:.3f})')
    
    for iteration in range(max_iterations):
        # Pegar posição REAL do efetuador no PyBullet
        ee_state = p.getLinkState(controller.arm.id, controller.arm.eef_link_index)
        ee_pos = ee_state[0]  # Posição global XYZ
        
        distance = calculate_distance([ee_pos[0], ee_pos[1]], cube_pos)
        print(f'[BUSCA] Iteração {iteration+1}: Mão em ({ee_pos[0]:.3f}, {ee_pos[1]:.3f}), Distância: {distance:.3f}m')
        
        if distance < max_distance:
            print(f'[BUSCA] ✓✓✓ SUCESSO! Distância: {distance:.3f}m < {max_distance:.3f}m')
            return True
        
        # Mover 50% em direção ao cubo
        move_fraction = 0.5
        target_x = ee_pos[0] + (cube_pos[0] - ee_pos[0]) * move_fraction
        target_y = ee_pos[1] + (cube_pos[1] - ee_pos[1]) * move_fraction
        
        # Calcular IK
        ik = controller.kin.inverse_kinematics(target_x, target_y)
        if ik is None:
            print(f'[BUSCA] ✗ IK impossível para ({target_x:.3f}, {target_y:.3f})')
            continue
        
        # Movimento suave
        controller.arm.ramped_move(ik, duration=0.5)
    
    # Última tentativa - ir direto ao cubo
    print(f'[BUSCA] → Tentativa final: ir direto ao cubo')
    ik = controller.kin.inverse_kinematics(cube_pos[0], cube_pos[1])
    if ik:
        controller.arm.ramped_move(ik, duration=0.8)
    
    ee_state = p.getLinkState(controller.arm.id, controller.arm.eef_link_index)
    ee_pos = ee_state[0]
    final_distance = calculate_distance([ee_pos[0], ee_pos[1]], cube_pos)
    print(f'[BUSCA] Distância final: {final_distance:.3f}m')
    
    if final_distance < max_distance * 2:  # Tolerância maior
        print(f'[BUSCA] ⚠ Tolerância relaxada OK')
        return True
    return False

def run(doF=3, cycles=6, gui=True):
    if doF == 2:
        links = [0.5, 0.5]
        urdf = './models/planar_arm_2dof.urdf'
    else:
        links = [0.5, 0.5, 0.4]  # Real URDF lengths: 0.5 + 0.5 + 0.4 = 1.4m
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
                step_many(20)
                print(f'[SIMULAÇÃO] Cubo agarrado! Levantando...')
                controller.avoid_and_move(cube_pos[0], cube_pos[1]+0.3, duration=1.2)
                print(f'[SIMULAÇÃO] Movendo para destino (tray)...')
                controller.avoid_and_move(tray_pos[0], tray_pos[1], duration=1.5)
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
