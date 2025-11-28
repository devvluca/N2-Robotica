"""
Exemplos de uso da simulação do manipulador planar 2 DOF.

Este arquivo demonstra casos de uso comuns e técnicas avançadas.
"""

import sys
import os
import numpy as np
from pathlib import Path

# Adicionar pasta src ao path
sys.path.insert(0, os.path.dirname(__file__))

from src.robot_simulation import RobotSimulation
from src.kinematics import PlanarKinematics
from src.pid_controller import PIDConfig, DualJointPIDController


def exemplo_1_simulacao_basica():
    """
    Exemplo 1: Simulação básica com referência fixa.
    
    Mantém o manipulador em uma posição angular fixa por 5 segundos.
    """
    print("\n" + "="*60)
    print("EXEMPLO 1: Simulação Básica")
    print("="*60)
    
    # Criar simulação
    sim = RobotSimulation(enable_gui=True)
    
    # Definir referência angular fixa
    sim.set_reference_angles(np.pi/4, -np.pi/6)  # 45°, -30°
    
    # Configurações
    dt = sim.config['simulation']['time_step']
    simulation_time = 5.0
    steps = int(simulation_time / dt)
    
    # Executar simulação
    for step in range(steps):
        sim.step(dt)
        
        # Imprimir a cada 100 passos
        if step % 100 == 0:
            errors = sim.reference_angles - sim.current_angles
            print(f"Passo {step:5d} | θ1={sim.current_angles[0]:6.3f} | "
                  f"θ2={sim.current_angles[1]:6.3f} | "
                  f"E1={errors[0]:6.3f} | E2={errors[1]:6.3f}")
    
    # Calcular e exibir métricas
    metrics = sim.calculate_performance_metrics()
    print("\nMétricas de Desempenho:")
    print(f"  Tempo de acomodação J1: {metrics.get('settling_time_j1', 'N/A'):.3f}s")
    print(f"  Tempo de acomodação J2: {metrics.get('settling_time_j2', 'N/A'):.3f}s")
    print(f"  Erro final J1: {metrics.get('steady_state_error_j1', 0):.6f} rad")
    print(f"  Erro final J2: {metrics.get('steady_state_error_j2', 0):.6f} rad")
    print(f"  Energia consumida J1: {metrics.get('energy_j1', 0):.3f} J")
    print(f"  Energia consumida J2: {metrics.get('energy_j2', 0):.3f} J")
    
    # Salvar dados
    csv_file = sim.save_metrics()
    print(f"\nDados salvos em: {csv_file}")
    
    # Encerrar
    sim.shutdown()


def exemplo_2_trajetoria_senoidal():
    """
    Exemplo 2: Seguimento de trajetória senoidal.
    
    O manipulador segue referências que variam sinusoidalmente,
    testando a resposta dinâmica do controlador.
    """
    print("\n" + "="*60)
    print("EXEMPLO 2: Seguimento de Trajetória Senoidal")
    print("="*60)
    
    sim = RobotSimulation(enable_gui=True)
    
    # Parâmetros da trajetória
    dt = sim.config['simulation']['time_step']
    total_time = 10.0
    frequency = 0.5  # Hz
    amplitude = 0.5  # radianos
    
    steps = int(total_time / dt)
    
    print(f"Trajetória: A={amplitude:.2f}rad, f={frequency}Hz, T={total_time}s\n")
    
    for step in range(steps):
        # Gerar referência senoidal
        t = step * dt
        theta1_ref = amplitude * np.sin(2 * np.pi * frequency * t)
        theta2_ref = amplitude * np.cos(2 * np.pi * frequency * t)
        
        sim.set_reference_angles(theta1_ref, theta2_ref)
        sim.step(dt)
        
        if step % 500 == 0:
            print(f"t={t:.2f}s | Ref: θ1={theta1_ref:6.3f} θ2={theta2_ref:6.3f} | "
                  f"Atual: θ1={sim.current_angles[0]:6.3f} θ2={sim.current_angles[1]:6.3f}")
    
    # Salvar e encerrar
    sim.save_metrics()
    sim.shutdown()


def exemplo_3_teste_robustez():
    """
    Exemplo 3: Teste de robustez com perturbações.
    
    Aplica perturbações externas e verifica a recuperação do controlador.
    """
    print("\n" + "="*60)
    print("EXEMPLO 3: Teste de Robustez com Perturbações")
    print("="*60)
    
    sim = RobotSimulation(enable_gui=True)
    sim.set_reference_angles(0.3, -0.2)
    
    dt = sim.config['simulation']['time_step']
    total_time = 8.0
    steps = int(total_time / dt)
    perturbation_time = total_time * 0.5  # Aplicar no meio
    
    print(f"Simulação: {total_time}s com perturbação em t={perturbation_time}s\n")
    
    for step in range(steps):
        t = step * dt
        
        # Aplicar perturbação
        if abs(t - perturbation_time) < 0.01:
            print(f"[{t:.2f}s] Aplicando perturbação...")
            sim.apply_perturbation()
        
        sim.step(dt)
        
        if step % 200 == 0:
            errors = sim.reference_angles - sim.current_angles
            print(f"t={t:.2f}s | θ1={sim.current_angles[0]:6.3f} | "
                  f"E1={errors[0]:6.3f} | Perturbação: {sim.perturbation_active}")
    
    sim.save_metrics()
    sim.shutdown()


def exemplo_4_cinemática_inversa():
    """
    Exemplo 4: Uso de cinemática inversa para alcançar ponto alvo.
    
    Calcula os ângulos necessários para atingir uma posição cartesiana,
    depois segue essa trajetória de ângulos.
    """
    print("\n" + "="*60)
    print("EXEMPLO 4: Cinemática Inversa e Trajetória")
    print("="*60)
    
    # Criar cinemática
    kinematics = PlanarKinematics((0.5, 0.3))
    
    # Posições alvo em cartesiano
    targets = [
        (0.6, 0.2),  # Ponto 1
        (0.3, 0.5),  # Ponto 2
        (0.4, 0.1),  # Ponto 3
    ]
    
    sim = RobotSimulation(enable_gui=True)
    
    dt = sim.config['simulation']['time_step']
    time_per_target = 3.0
    steps_per_target = int(time_per_target / dt)
    
    print(f"Alvo | X (m) | Y (m) | θ1 (rad) | θ2 (rad) | Status")
    print("-" * 60)
    
    for i, target in enumerate(targets):
        x, y = target
        
        # Calcular cinemática inversa
        ik_result = kinematics.inverse_kinematics(target, sim.current_angles)
        
        if not ik_result['valid']:
            print(f" {i}  | {x:.2f}  | {y:.2f}  | INVÁLIDO | {ik_result['error']}")
            continue
        
        theta1_ref, theta2_ref = ik_result['theta']
        
        # Verificar com cinemática direta
        fk = kinematics.forward_kinematics((theta1_ref, theta2_ref))
        fk_pos = fk['ee_position']
        
        print(f" {i}  | {x:.2f}  | {y:.2f}  | {theta1_ref:7.3f}  | "
              f"{theta2_ref:7.3f}  | Verificação: ({fk_pos[0]:.2f}, {fk_pos[1]:.2f})")
        
        # Seguir para esse ponto
        sim.set_reference_angles(theta1_ref, theta2_ref)
        
        for _ in range(steps_per_target):
            sim.step(dt)
    
    print("\nCinemática finalizada")
    sim.save_metrics()
    sim.shutdown()


def exemplo_5_ajuste_pid():
    """
    Exemplo 5: Comparação de diferentes ganhos PID.
    
    Testa o mesmo movimento com diferentes configurações de PID
    para encontrar a melhor resposta.
    """
    print("\n" + "="*60)
    print("EXEMPLO 5: Comparação de Ganhos PID")
    print("="*60)
    
    # Diferentes configurações de ganhos
    configurations = [
        {"name": "Conservador", "kp": 30, "ki": 5, "kd": 3},
        {"name": "Padrão",      "kp": 50, "ki": 10, "kd": 5},
        {"name": "Agressivo",   "kp": 80, "ki": 15, "kd": 8},
    ]
    
    results = {}
    
    for config in configurations:
        print(f"\nTestando: {config['name']} (Kp={config['kp']}, Ki={config['ki']}, Kd={config['kd']})")
        
        sim = RobotSimulation(enable_gui=False)
        
        # Atualizar ganhos
        sim.pid_controller.update_gains(1, kp=config['kp'], ki=config['ki'], kd=config['kd'])
        sim.pid_controller.update_gains(2, kp=config['kp'], ki=config['ki'], kd=config['kd'])
        
        # Executar teste
        sim.set_reference_angles(np.pi/4, -np.pi/6)
        
        dt = sim.config['simulation']['time_step']
        for _ in range(int(3.0 / dt)):
            sim.step(dt)
        
        # Coletar métricas
        metrics = sim.calculate_performance_metrics()
        results[config['name']] = metrics
        
        print(f"  Settling time J1: {metrics.get('settling_time_j1', 'N/A'):.3f}s")
        print(f"  Steady-state error J1: {metrics.get('steady_state_error_j1', 0):.6f} rad")
        print(f"  Energia J1: {metrics.get('energy_j1', 0):.3f} J")
        
        sim.shutdown()
    
    # Resumo comparativo
    print("\n" + "-" * 60)
    print("RESUMO COMPARATIVO")
    print("-" * 60)
    print(f"{'Config':<15} {'Settling (s)':<15} {'SS Error (rad)':<15} {'Energia (J)':<15}")
    print("-" * 60)
    
    for name, metrics in results.items():
        settling = metrics.get('settling_time_j1', 0) or 0
        ss_error = metrics.get('steady_state_error_j1', 0)
        energy = metrics.get('energy_j1', 0)
        print(f"{name:<15} {settling:<15.4f} {ss_error:<15.6f} {energy:<15.3f}")


def exemplo_6_espaco_trabalho():
    """
    Exemplo 6: Exploração do espaço de trabalho.
    
    Mapeia a posição do end-effector para diferentes ângulos,
    gerando uma visualização do espaço de trabalho.
    """
    print("\n" + "="*60)
    print("EXEMPLO 6: Análise do Espaço de Trabalho")
    print("="*60)
    
    kinematics = PlanarKinematics((0.5, 0.3))
    
    # Gerar grid de ângulos
    theta1_range = np.linspace(-np.pi, np.pi, 20)
    theta2_range = np.linspace(-np.pi, np.pi, 20)
    
    workspace_points = []
    min_x, max_x = float('inf'), float('-inf')
    min_y, max_y = float('inf'), float('-inf')
    
    for theta1 in theta1_range:
        for theta2 in theta2_range:
            fk = kinematics.forward_kinematics((theta1, theta2))
            pos = fk['ee_position']
            
            workspace_points.append(pos)
            
            min_x = min(min_x, pos[0])
            max_x = max(max_x, pos[0])
            min_y = min(min_y, pos[1])
            max_y = max(max_y, pos[1])
    
    info = kinematics.get_info()
    
    print(f"\nInformações do Manipulador:")
    print(f"  Link 1: {info['link_lengths'][0]:.2f}m")
    print(f"  Link 2: {info['link_lengths'][1]:.2f}m")
    print(f"  Alcance máximo: {info['max_reach']:.3f}m")
    print(f"  Alcance mínimo: {info['min_reach']:.3f}m")
    print(f"  Área aproximada: {info['workspace_area']:.3f}m²")
    
    print(f"\nEspaço de Trabalho (End-Effector):")
    print(f"  X: [{min_x:.3f}, {max_x:.3f}] m")
    print(f"  Y: [{min_y:.3f}, {max_y:.3f}] m")
    print(f"  Dimensões: {max_x - min_x:.3f} m × {max_y - min_y:.3f} m")
    
    # Salvar dados para visualização
    workspace_file = "logs/workspace.csv"
    os.makedirs("logs", exist_ok=True)
    
    with open(workspace_file, 'w') as f:
        f.write("x,y\n")
        for point in workspace_points:
            f.write(f"{point[0]:.4f},{point[1]:.4f}\n")
    
    print(f"\nPontos do espaço de trabalho salvos em: {workspace_file}")


def exemplo_7_resposta_frequencia():
    """
    Exemplo 7: Teste de resposta em frequência.
    
    Aplica referências com diferentes frequências e mede a resposta
    do controlador, simulando análise de bode.
    """
    print("\n" + "="*60)
    print("EXEMPLO 7: Resposta em Frequência")
    print("="*60)
    
    sim = RobotSimulation(enable_gui=False)
    
    frequencies = [0.1, 0.5, 1.0, 2.0, 5.0]  # Hz
    amplitude = 0.2  # radianos
    
    dt = sim.config['simulation']['time_step']
    
    print(f"Amplitude: {amplitude:.2f} rad\n")
    print(f"{'Freq (Hz)':<12} {'Amplitude Medida':<20} {'Atraso de Fase':<20}")
    print("-" * 60)
    
    for freq in frequencies:
        sim.reset()
        
        # Duração suficiente para estabilizar
        duration = 2.0 / freq  # 2 períodos
        steps = int(duration / dt)
        
        peak_actual = 0
        phase_delay = 0
        
        for step in range(steps):
            t = step * dt
            
            # Referência senoidal
            theta1_ref = amplitude * np.sin(2 * np.pi * freq * t)
            sim.set_reference_angles(theta1_ref, 0)
            sim.step(dt)
            
            # Registrar pico da resposta real
            if step > steps // 2:  # Depois de estabilizado
                peak_actual = max(peak_actual, abs(sim.current_angles[0]))
        
        # Calcular amplitude relativa
        amplitude_ratio = peak_actual / amplitude if amplitude > 0 else 0
        
        print(f"{freq:<12.1f} {amplitude_ratio:<20.3f} {'TBD':<20}")
    
    sim.shutdown()


# Menu de exemplos
def main():
    """Menu principal para executar exemplos."""
    
    exemplos = {
        '1': ('Simulação Básica', exemplo_1_simulacao_basica),
        '2': ('Trajetória Senoidal', exemplo_2_trajetoria_senoidal),
        '3': ('Teste de Robustez', exemplo_3_teste_robustez),
        '4': ('Cinemática Inversa', exemplo_4_cinemática_inversa),
        '5': ('Comparação de Ganhos PID', exemplo_5_ajuste_pid),
        '6': ('Espaço de Trabalho', exemplo_6_espaco_trabalho),
        '7': ('Resposta em Frequência', exemplo_7_resposta_frequencia),
    }
    
    print("\n" + "="*60)
    print("EXEMPLOS - MANIPULADOR PLANAR 2 DOF")
    print("="*60)
    
    for key, (name, _) in exemplos.items():
        print(f"  {key}. {name}")
    
    print(f"  0. Executar todos os exemplos")
    print(f"  q. Sair")
    
    choice = input("\nEscolha uma opção: ").strip()
    
    try:
        if choice == 'q':
            print("Encerrando...")
            return
        elif choice == '0':
            for key in sorted(exemplos.keys()):
                exemplos[key][1]()
        elif choice in exemplos:
            exemplos[choice][1]()
        else:
            print("Opção inválida!")
    
    except KeyboardInterrupt:
        print("\n\nPrograma interrompido pelo usuário")
    except Exception as e:
        print(f"\nErro durante execução: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
