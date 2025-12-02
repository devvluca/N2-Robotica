import argparse
import time
import math
from src.kinematics import PlanarArmKinematics
from src.arm import PlanarArm
from src.control import ArmController
from src.simulation import setup_simulation, create_random_cube, remove_body, step_many
import pybullet as p

# Cliente Node-RED para supervisão
try:
    from node_red_client import get_client as get_node_red_client
    NODE_RED_AVAILABLE = True
except ImportError:
    NODE_RED_AVAILABLE = False
    print("[AVISO] Cliente Node-RED não disponível")

def calculate_distance(pos1, pos2):
    """Calculate 2D distance between two positions."""
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

def approach_cube_smoothly(controller, cube_pos, cube_id, max_distance=0.15, max_iterations=10):
    """Move arm smoothly towards cube, avoiding obstacles."""
    print(f'\n[BUSCA] ===== ABORDAGEM AO CUBO =====')
    print(f'[BUSCA] Alvo: ({cube_pos[0]:.3f}, {cube_pos[1]:.3f})')
    
    # Primeiro, verificar se há obstáculo no caminho até o cubo
    obstacle = controller.check_obstacle_between(cube_pos[0], cube_pos[1])
    
    if obstacle:
        print(f'[BUSCA] 🚧 Obstáculo no caminho para o cubo!')
        # Usar navegação com desvio para chegar ao cubo
        controller.avoid_and_move(cube_pos[0], cube_pos[1], duration=3.0)
        
        # Verificar distância final
        ee_pos, _ = controller.arm.get_ee_position()
        final_distance = calculate_distance([ee_pos[0], ee_pos[1]], cube_pos)
        print(f'[BUSCA] Distância após desvio: {final_distance:.3f}m')
        
        if final_distance < max_distance:
            print(f'[BUSCA] ✓✓✓ SUCESSO com desvio!')
            return True
        
        # Se ainda não chegou, fazer movimento direto com tempo maior
        print(f'[BUSCA] Movimento direto final ao cubo...')
        controller.move_to_xy(cube_pos[0], cube_pos[1], duration=2.0)
        step_many(50)
        
        ee_pos, _ = controller.arm.get_ee_position()
        final_distance = calculate_distance([ee_pos[0], ee_pos[1]], cube_pos)
        if final_distance < max_distance * 1.5:
            print(f'[BUSCA] ✓✓✓ SUCESSO após ajuste!')
            return True
        return False
    
    # Abordagem iterativa: ir direto ao cubo com movimentos longos
    for iteration in range(max_iterations):
        ee_pos, _ = controller.arm.get_ee_position()
        
        distance = calculate_distance([ee_pos[0], ee_pos[1]], cube_pos)
        print(f'[BUSCA] Iteração {iteration+1}: Mão em ({ee_pos[0]:.3f}, {ee_pos[1]:.3f}), Distância: {distance:.3f}m')
        
        if distance < max_distance:
            print(f'[BUSCA] ✓✓✓ SUCESSO! Distância: {distance:.3f}m < {max_distance:.3f}m')
            return True
        
        # Movimento com duração proporcional à distância
        move_duration = max(1.0, distance * 2)
        controller.move_to_xy(cube_pos[0], cube_pos[1], duration=move_duration)
        
        # Estabilização
        step_many(40)
    
    # Última tentativa - ir direto ao cubo com mais tempo
    print(f'[BUSCA] → Tentativa final: ir direto ao cubo')
    controller.move_to_xy(cube_pos[0], cube_pos[1], duration=1.2)
    
    ee_pos, _ = controller.arm.get_ee_position()
    final_distance = calculate_distance([ee_pos[0], ee_pos[1]], cube_pos)
    print(f'[BUSCA] Distância final: {final_distance:.3f}m')
    
    if final_distance < max_distance * 1.5:  # Tolerância um pouco maior
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
    
    # Registrar obstáculo para detecção de colisão
    if obstacle_id >= 0:
        controller.register_obstacle(obstacle_id)

    # Inicializar cliente Node-RED para supervisão
    node_red = None
    if NODE_RED_AVAILABLE:
        node_red = get_node_red_client()
        node_red.send_simulation_start({
            'dof': doF,
            'cycles': cycles,
            'links': links,
            'tray_pos': list(tray_pos),
            'obstacle_pos': [1.0, 0.3]
        })

    # spawn limits to ensure IK reach
    reach = sum(links)
    x_range = (0.2, min(1.0, reach - 0.05))
    y_range = (-min(0.4, reach - 0.05), min(0.4, reach - 0.05))
    counter = 0
    
    # Estatísticas globais para resumo
    all_metrics = []
    simulation_start_time = time.time()
    try:
        while counter < cycles:
            print(f'\n[SIMULAÇÃO] === Iniciando ciclo {counter + 1}/{cycles} ===')
            cube_id, cube_pos = create_random_cube()
            print(f'[SIMULAÇÃO] Cubo criado na posição: ({cube_pos[0]:.2f}, {cube_pos[1]:.2f})')
            
            # Aproximação suave ao cubo
            if approach_cube_smoothly(controller, cube_pos, cube_id, max_distance=0.12):
                controller.grasp(cube_id)
                step_many(20)
                print(f'[SIMULAÇÃO] Cubo agarrado! Levantando...')
                
                # Teste de perturbação: aplicar força externa e verificar correção
                if counter == 0:  # Apenas no primeiro ciclo para demonstrar
                    print(f'[SIMULAÇÃO] === TESTE DE PERTURBAÇÃO ===')
                    controller.apply_perturbation_test(force_magnitude=10.0)
                
                # Levantar o cubo primeiro (posição segura)
                # Se o cubo estava do lado negativo, precisa desviar do obstáculo
                print(f'[SIMULAÇÃO] Levantando cubo...')
                obstacle_y = 0.30  # Posição Y do obstáculo
                if cube_pos[1] < obstacle_y:  # Cubo estava ABAIXO do obstáculo
                    # Retrair primeiro para não bater no obstáculo
                    print(f'[SIMULAÇÃO] Cubo estava abaixo do obstáculo, desviando...')
                    controller.move_to_xy(0.5, 0.0, duration=1.0)  # Retrair
                    step_many(30)
                    controller.move_to_xy(0.5, 0.55, duration=1.0)  # Subir por cima
                    step_many(30)
                    controller.move_to_xy(0.8, 0.55, duration=1.0)  # Avançar acima
                    step_many(30)
                else:
                    # Cubo já estava do lado certo, levantar diretamente
                    controller.move_to_xy(0.8, 0.4, duration=1.5)
                    step_many(50)
                
                # Mover para posição da bandeja (múltiplas tentativas)
                print(f'[SIMULAÇÃO] Movendo para bandeja ({tray_pos[0]:.2f}, {tray_pos[1]:.2f})...')
                for attempt in range(3):
                    controller.move_to_xy(tray_pos[0], tray_pos[1], duration=1.5)
                    step_many(60)
                    
                    # Verificar posição
                    ee_pos, _ = controller.arm.get_ee_position()
                    tray_distance = calculate_distance([ee_pos[0], ee_pos[1]], [tray_pos[0], tray_pos[1]])
                    print(f'[SIMULAÇÃO] Tentativa {attempt+1}: pos=({ee_pos[0]:.2f}, {ee_pos[1]:.2f}), dist={tray_distance:.3f}m')
                    
                    if tray_distance < 0.15:
                        break
                
                controller.release(cube_id)
                print(f'[SIMULAÇÃO] Cubo solto!')
                cycle_success = True
                
                # Enviar evento para Node-RED
                if node_red:
                    node_red.send_event('cube_released', {
                        'position': [round(ee_pos[0], 3), round(ee_pos[1], 3)],
                        'tray_distance': round(tray_distance, 3)
                    })
            else:
                print(f'[SIMULAÇÃO] AVISO: Não conseguiu chegar perto do cubo em tempo hábil.')
                cycle_success = False
            
            remove_body(cube_id)
            
            # Coletar e enviar métricas do ciclo
            mean_error = sum(arm.err_log) / len(arm.err_log) if arm.err_log else 0
            max_overshoot = arm.max_error
            total_energy = sum(arm.energy_log) if arm.energy_log else 0
            
            cycle_metrics = {
                'mean_error': mean_error,
                'max_overshoot': max_overshoot,
                'total_energy': total_energy,
                'stabilization_time': 0,  # TODO: calcular tempo de estabilização
                'success': cycle_success
            }
            all_metrics.append(cycle_metrics)
            
            # Enviar métricas para Node-RED
            if node_red:
                node_red.send_cycle_metrics(counter + 1, cycles, cycle_metrics)
            
            controller.log_metrics()
            counter += 1
            print(f'[SIMULAÇÃO] Ciclo {counter}/{cycles} concluído\n')
            time.sleep(0.5)
        
        # Enviar resumo final para Node-RED
        if node_red and all_metrics:
            total_time = time.time() - simulation_start_time
            successful = sum(1 for m in all_metrics if m['success'])
            node_red.send_simulation_end({
                'total_cycles': cycles,
                'successful_cycles': successful,
                'avg_mean_error': sum(m['mean_error'] for m in all_metrics) / len(all_metrics),
                'avg_max_overshoot': sum(m['max_overshoot'] for m in all_metrics) / len(all_metrics),
                'total_energy': sum(m['total_energy'] for m in all_metrics),
                'total_time': total_time
            })
            print(f'\n[NODE-RED] Resumo enviado: {successful}/{cycles} ciclos bem sucedidos')
            
    finally:
        if node_red:
            node_red.close()
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
