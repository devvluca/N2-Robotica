"""
Simulação Principal do Robô Aspirador Inteligente.
Executa múltiplas sessões demonstrando aprendizado de rotas.
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import argparse
import os
import sys
import json

# Adicionar src ao path
sys.path.insert(0, os.path.dirname(__file__))

from src.robot import VacuumRobot
from src.environment import Environment
from src.mapping import OccupancyMap, CELL_OBSTACLE
from src.controller import NavigationController, RobotState
from src import node_red_client


# Diretório para salvar mapas
MAPS_DIR = os.path.join(os.path.dirname(__file__), 'saved_maps')
os.makedirs(MAPS_DIR, exist_ok=True)


def setup_pybullet(gui=True):
    """Configura o PyBullet."""
    if gui:
        client = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    else:
        client = p.connect(p.DIRECT)
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1/240)
    
    # Câmera vista de cima - ajustada para mapa menor
    p.resetDebugVisualizerCamera(
        cameraDistance=4.0,
        cameraYaw=0,
        cameraPitch=-70,
        cameraTargetPosition=[1.25, 1.25, 0]
    )
    
    return client


def run_execution(execution_num, total_executions, previous_map=None, 
                  max_time=120, gui=True, send_to_node_red=True):
    """
    Executa uma sessão de limpeza.
    
    Returns:
        OccupancyMap: Mapa final desta execução
        dict: Métricas finais
    """
    print(f"\n{'='*60}")
    print(f"EXECUÇÃO {execution_num}/{total_executions}")
    print(f"{'='*60}")
    
    # Configurar PyBullet
    client = setup_pybullet(gui)
    
    # Criar ambiente (2.5x2.5 metros - mapa reduzido)
    env = Environment(client, width=2.5, height=2.5, cell_size=0.05)
    
    # Criar mapa de ocupação
    occupancy_map = OccupancyMap(
        width=env.width,
        height=env.height,
        cell_size=env.cell_size
    )
    
    # Criar robô (posição inicial num canto livre - longe dos obstáculos)
    robot = VacuumRobot(client, start_pos=[0.4, 0.4], start_angle=0.785)
    
    # Criar controlador (agora com varredura sistemática)
    controller = NavigationController(robot, occupancy_map, env)
    print("[SIM] Usando controlador com VARREDURA SISTEMÁTICA")
    
    # Cliente Node-RED
    if send_to_node_red:
        nr_client = node_red_client.get_client()
        nr_client.send_start({
            'execution': execution_num,
            'total_executions': total_executions,
            'width': env.width,
            'height': env.height,
            'robot_radius': robot.radius,
            'use_previous_map': previous_map is not None
        })
    
    # Variáveis de controle
    sim_time = 0
    dt = 1/240
    last_log_time = 0
    log_interval = 2.0  # Log a cada 2 segundos
    
    start_real_time = time.time()
    
    print(f"[SIM] Iniciando limpeza... (máx {max_time}s)")
    
    # Loop principal
    while sim_time < max_time:
        # Ler sensores
        sensor_readings = robot.get_sensor_readings()
        
        # Atualizar controlador
        left_vel, right_vel = controller.update(dt, sensor_readings, sim_time)
        
        # Aplicar velocidades
        robot.set_wheel_velocities(left_vel, right_vel)
        
        # Atualizar métricas
        robot.update_metrics()
        
        # Step da simulação
        p.stepSimulation()
        sim_time += dt
        
        # Verificar se terminou
        if controller.state == RobotState.FINISHED:
            print(f"[SIM] ✓ Limpeza completa em {sim_time:.1f}s!")
            break
        
        # Log periódico
        if sim_time - last_log_time >= log_interval:
            last_log_time = sim_time
            
            stats = occupancy_map.get_coverage_stats()
            robot_metrics = robot.get_metrics()
            
            print(f"[{sim_time:5.1f}s] Cobert: {stats['coverage_percent']:5.1f}% | "
                  f"Estado: {controller.state.name:10} | "
                  f"Energia: {robot_metrics['energy_consumed']:6.1f}J | "
                  f"Dist: {robot_metrics['distance_traveled']:5.1f}m")
            
            # Enviar ao Node-RED
            if send_to_node_red:
                nr_client.send_periodic_update({
                    'execution': execution_num,
                    'coverage_percent': stats['coverage_percent'],
                    'covered_area_m2': stats['covered_area_m2'],
                    'time_elapsed': sim_time,
                    'energy': robot_metrics['energy_consumed'],
                    'distance': robot_metrics['distance_traveled'],
                    'collisions': controller.get_metrics()['collisions'],
                    'state': controller.state.name
                })
        
        # Delay para visualização
        if gui:
            time.sleep(dt * 0.3)  # 30% da velocidade real
    
    # Métricas finais
    stats = occupancy_map.get_coverage_stats()
    robot_metrics = robot.get_metrics()
    controller_metrics = controller.get_metrics()
    
    real_time = time.time() - start_real_time
    
    # Eficiência
    if robot_metrics['energy_consumed'] > 0:
        area_per_energy = stats['covered_area_m2'] / robot_metrics['energy_consumed']
    else:
        area_per_energy = 0
    
    final_metrics = {
        'execution': execution_num,
        'total_time': round(sim_time, 2),
        'real_time': round(real_time, 2),
        'coverage_percent': stats['coverage_percent'],
        'covered_area_m2': stats['covered_area_m2'],
        'total_cleanable_area_m2': stats['total_cleanable_area_m2'],
        'energy': round(robot_metrics['energy_consumed'], 2),
        'distance': round(robot_metrics['distance_traveled'], 2),
        'collisions': controller_metrics['collisions'],
        'efficiency': round(stats['efficiency'], 4),
        'area_per_energy': round(area_per_energy, 4)
    }
    
    # Enviar resumo ao Node-RED
    if send_to_node_red:
        nr_client.send_end(final_metrics)
    
    # Salvar mapa
    occupancy_map.total_time = sim_time
    occupancy_map.collisions = controller_metrics['collisions']
    map_path = os.path.join(MAPS_DIR, f'map_execution_{execution_num}.json')
    occupancy_map.save(map_path)
    
    # Imprimir resumo
    print(f"\n{'─'*50}")
    print(f"RESUMO EXECUÇÃO {execution_num}")
    print(f"{'─'*50}")
    print(f"  Tempo total:      {sim_time:.1f}s")
    print(f"  Cobertura:        {stats['coverage_percent']:.1f}%")
    print(f"  Área coberta:     {stats['covered_area_m2']:.2f} m²")
    print(f"  Energia gasta:    {robot_metrics['energy_consumed']:.2f} J")
    print(f"  Distância:        {robot_metrics['distance_traveled']:.2f} m")
    print(f"  Colisões:         {controller_metrics['collisions']}")
    print(f"  Eficiência:       {area_per_energy:.4f} m²/J")
    print(f"{'─'*50}")
    
    # Desconectar PyBullet
    p.disconnect()
    
    return occupancy_map, final_metrics


def run_multiple_executions(num_executions=3, max_time=120, gui=True, send_to_node_red=True):
    """Executa múltiplas sessões demonstrando aprendizado."""
    all_metrics = []
    previous_map = None
    
    for i in range(1, num_executions + 1):
        # Tentar carregar mapa anterior
        if i > 1:
            prev_map_path = os.path.join(MAPS_DIR, f'map_execution_{i-1}.json')
            if os.path.exists(prev_map_path):
                previous_map = OccupancyMap(2.5, 2.5, 0.05)
                previous_map.load(prev_map_path)
        
        # Executar
        final_map, metrics = run_execution(
            execution_num=i,
            total_executions=num_executions,
            previous_map=previous_map,
            max_time=max_time,
            gui=gui,
            send_to_node_red=send_to_node_red
        )
        
        all_metrics.append(metrics)
        previous_map = final_map
        
        # Pausa entre execuções
        if i < num_executions:
            print(f"\n⏳ Próxima execução em 2 segundos...")
            time.sleep(2)
    
    # Comparativo final
    print(f"\n{'='*60}")
    print("COMPARATIVO DE EXECUÇÕES")
    print(f"{'='*60}")
    print(f"{'Exec':<6} {'Tempo':>8} {'Cobert.':>10} {'Energia':>10} {'Efic.':>12}")
    print(f"{'─'*50}")
    
    for m in all_metrics:
        print(f"{m['execution']:<6} {m['total_time']:>7.1f}s {m['coverage_percent']:>9.1f}% "
              f"{m['energy']:>9.2f}J {m['area_per_energy']:>11.4f}")
    
    # Calcular melhorias
    if len(all_metrics) > 1:
        first = all_metrics[0]
        last = all_metrics[-1]
        
        if first['total_time'] > 0:
            time_improvement = (first['total_time'] - last['total_time']) / first['total_time'] * 100
        else:
            time_improvement = 0
            
        if first['energy'] > 0:
            energy_improvement = (first['energy'] - last['energy']) / first['energy'] * 100
        else:
            energy_improvement = 0
            
        if first['area_per_energy'] > 0:
            efficiency_improvement = (last['area_per_energy'] - first['area_per_energy']) / first['area_per_energy'] * 100
        else:
            efficiency_improvement = 0
        
        print(f"\n📈 MELHORIAS (1ª → {len(all_metrics)}ª execução):")
        print(f"  Tempo:      {time_improvement:+.1f}%")
        print(f"  Energia:    {energy_improvement:+.1f}%")
        print(f"  Eficiência: {efficiency_improvement:+.1f}%")
    
    # Enviar comparativo ao Node-RED
    if send_to_node_red:
        nr_client = node_red_client.get_client()
        nr_client.send_comparison(all_metrics)
    
    return all_metrics


def main():
    """Função principal."""
    parser = argparse.ArgumentParser(description='Robô Aspirador Inteligente')
    parser.add_argument('--executions', '-e', type=int, default=3,
                       help='Número de execuções (padrão: 3)')
    parser.add_argument('--time', '-t', type=int, default=90,
                       help='Tempo máximo por execução em segundos (padrão: 90)')
    parser.add_argument('--no-gui', action='store_true',
                       help='Executar sem interface gráfica')
    parser.add_argument('--no-nodred', action='store_true',
                       help='Não enviar métricas ao Node-RED')
    parser.add_argument('--clean', action='store_true',
                       help='Limpar mapas salvos antes de iniciar')
    
    args = parser.parse_args()
    
    # Limpar mapas se solicitado
    if args.clean:
        import shutil
        if os.path.exists(MAPS_DIR):
            shutil.rmtree(MAPS_DIR)
            os.makedirs(MAPS_DIR)
            print("[SETUP] Mapas anteriores removidos")
    
    print("╔════════════════════════════════════════════════════════════╗")
    print("║         ROBÔ ASPIRADOR INTELIGENTE COM APRENDIZADO         ║")
    print("╠════════════════════════════════════════════════════════════╣")
    print(f"║  Execuções: {args.executions:<3}  |  Tempo máx: {args.time}s  |  GUI: {not args.no_gui:<5}   ║")
    print("╚════════════════════════════════════════════════════════════╝")
    
    # Executar
    all_metrics = run_multiple_executions(
        num_executions=args.executions,
        max_time=args.time,
        gui=not args.no_gui,
        send_to_node_red=not args.no_nodred
    )
    
    # Salvar métricas gerais
    metrics_path = os.path.join(MAPS_DIR, 'all_metrics.json')
    with open(metrics_path, 'w') as f:
        json.dump(all_metrics, f, indent=2)
    
    print(f"\n✅ Simulação completa! Métricas salvas em {metrics_path}")
    
    return all_metrics


if __name__ == '__main__':
    main()
