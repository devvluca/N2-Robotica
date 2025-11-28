"""
Exemplos práticos de uso do robô aspirador.

Demonstra diferentes cenários de limpeza e aprendizado.
"""

import sys
from pathlib import Path
import yaml
import logging

# Setup path
sys.path.insert(0, str(Path(__file__).parent / 'src'))

from robot_simulation import VacuumRobotSimulation


def setup_logging():
    """Configura logging."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )


def load_config():
    """Carrega configuração."""
    with open('config/robot_config.yaml', 'r') as f:
        return yaml.safe_load(f)


def example_1_basic_cleaning():
    """Exemplo 1: Limpeza básica com GUI."""
    print("\n" + "="*60)
    print("Exemplo 1: Limpeza Básica")
    print("="*60)
    print("Objetivo: Visualizar robô em ação com PyBullet GUI")
    print("O robô navegará usando estratégia padrão (espiral)")
    
    config = load_config()
    
    # Usar GUI
    config['simulation']['gui_enabled'] = True
    config['node_red']['enabled'] = False
    
    sim = VacuumRobotSimulation(config, use_gui=True)
    
    if not sim.initialize():
        print("❌ Erro na inicialização")
        return False
    
    try:
        print("\nRobô iniciando limpeza...")
        print("Pressione Ctrl+C para parar")
        
        metrics = sim.run_execution(max_duration=60.0)
        
        print(f"\n✓ Execução concluída!")
        print(f"  Cobertura: {metrics.coverage_percentage:.1f}%")
        print(f"  Energia: {metrics.energy_consumed:.2f}J")
        print(f"  Tempo: {metrics.duration:.1f}s")
        
        return True
    
    except KeyboardInterrupt:
        print("\n⏸ Execução interrompida")
        return True
    
    finally:
        sim.shutdown()


def example_2_headless_simulation():
    """Exemplo 2: Simulação sem GUI (mais rápida)."""
    print("\n" + "="*60)
    print("Exemplo 2: Simulação em Modo Headless")
    print("="*60)
    print("Objetivo: Executar simulação mais rápida sem visualização")
    
    config = load_config()
    config['simulation']['gui_enabled'] = False
    config['node_red']['enabled'] = False
    
    sim = VacuumRobotSimulation(config, use_gui=False)
    
    if not sim.initialize():
        print("❌ Erro na inicialização")
        return False
    
    try:
        print("\nExecutando 10 passos de simulação...")
        
        for i in range(10):
            metrics = sim.run_execution(max_duration=30.0)
            print(f"  [{i+1}] Cobertura: {metrics.coverage_percentage:.1f}% - "
                  f"Energia: {metrics.energy_consumed:.2f}J")
        
        print("\n✓ Múltiplas execuções concluídas!")
        return True
    
    finally:
        sim.shutdown()


def example_3_learning_progression():
    """Exemplo 3: Aprendizado progressivo."""
    print("\n" + "="*60)
    print("Exemplo 3: Ciclos de Aprendizado")
    print("="*60)
    print("Objetivo: Demonstrar otimização da rota entre execuções")
    print("O robô tenta 5 estratégias e aprende qual é melhor")
    
    config = load_config()
    config['simulation']['gui_enabled'] = False
    config['node_red']['enabled'] = False
    config['learning']['max_iterations'] = 5
    
    sim = VacuumRobotSimulation(config, use_gui=False)
    
    if not sim.initialize():
        print("❌ Erro na inicialização")
        return False
    
    try:
        print("\nExecutando ciclos de aprendizado...")
        metrics_list = sim.run_learning_cycles(num_cycles=5)
        
        print("\n" + "─"*60)
        print(f"{'Ciclo':<8} {'Cobertura':<12} {'Energia':<12} {'Eficiência':<12}")
        print("─"*60)
        
        for i, metrics in enumerate(metrics_list, 1):
            efficiency = metrics.coverage_percentage / max(metrics.energy_consumed, 0.1)
            print(f"{i:<8} {metrics.coverage_percentage:>9.1f}%  "
                  f"{metrics.energy_consumed:>10.2f}J  {efficiency:>10.2f}")
        
        print("─"*60)
        print(f"\n✓ Convergência: {sim.learning.get_convergence_percentage():.1f}%")
        return True
    
    finally:
        sim.shutdown()


def example_4_strategy_comparison():
    """Exemplo 4: Comparação de estratégias."""
    print("\n" + "="*60)
    print("Exemplo 4: Comparação de Estratégias")
    print("="*60)
    print("Objetivo: Comparar 3 estratégias de navegação")
    
    config = load_config()
    config['simulation']['gui_enabled'] = False
    config['node_red']['enabled'] = False
    
    strategies = ['spiral', 'systematic', 'random']
    results = {}
    
    for strategy in strategies:
        print(f"\nTestando estratégia: {strategy}")
        
        sim = VacuumRobotSimulation(config, use_gui=False)
        
        if not sim.initialize():
            print(f"❌ Erro inicializando para {strategy}")
            continue
        
        try:
            sim.navigation.set_exploration_strategy(strategy)
            metrics = sim.run_execution(max_duration=45.0)
            
            results[strategy] = {
                'coverage': metrics.coverage_percentage,
                'energy': metrics.energy_consumed,
                'duration': metrics.duration
            }
            
            print(f"  ✓ Cobertura: {metrics.coverage_percentage:.1f}%")
            print(f"    Energia: {metrics.energy_consumed:.2f}J")
            print(f"    Tempo: {metrics.duration:.1f}s")
        
        finally:
            sim.shutdown()
    
    # Comparação
    print("\n" + "─"*70)
    print(f"{'Estratégia':<15} {'Cobertura':<15} {'Energia':<15} {'Eficiência':<15}")
    print("─"*70)
    
    for strategy, data in results.items():
        efficiency = data['coverage'] / max(data['energy'], 0.1)
        print(f"{strategy:<15} {data['coverage']:>12.1f}%  "
              f"{data['energy']:>12.2f}J  {efficiency:>12.2f}")
    
    # Melhor estratégia
    if results:
        best = max(results.items(), 
                  key=lambda x: x[1]['coverage'] / max(x[1]['energy'], 0.1))
        print("─"*70)
        print(f"\n✓ Melhor estratégia: {best[0].upper()}")
    
    return True


def example_5_node_red_integration():
    """Exemplo 5: Integração com Node-RED."""
    print("\n" + "="*60)
    print("Exemplo 5: Integração com Node-RED")
    print("="*60)
    print("Objetivo: Enviar telemetria para Node-RED")
    print("\nPré-requisito: Node-RED rodando em localhost:1880")
    
    config = load_config()
    config['simulation']['gui_enabled'] = False
    config['node_red']['enabled'] = True
    config['node_red']['host'] = 'localhost'
    config['node_red']['port'] = 1880
    
    sim = VacuumRobotSimulation(config, use_gui=False)
    
    if not sim.initialize():
        print("❌ Erro na inicialização")
        return False
    
    if not sim.node_red or not sim.node_red.is_connected:
        print("⚠ Node-RED não disponível. Continuando sem telemetria.")
        return True
    
    try:
        print("\nEnviando telemetria para Node-RED por 30 segundos...")
        metrics = sim.run_execution(max_duration=30.0)
        
        print(f"\n✓ Execução concluída!")
        print(f"  Telemetria enviada: {sim.node_red.telemetry_queue.qsize()} mensagens pendentes")
        
        return True
    
    finally:
        sim.shutdown()


def example_6_performance_analysis():
    """Exemplo 6: Análise de desempenho."""
    print("\n" + "="*60)
    print("Exemplo 6: Análise de Desempenho")
    print("="*60)
    print("Objetivo: Coletar métricas detalhadas de desempenho")
    
    config = load_config()
    config['simulation']['gui_enabled'] = False
    config['node_red']['enabled'] = False
    config['logging']['enabled'] = True
    
    sim = VacuumRobotSimulation(config, use_gui=False)
    
    if not sim.initialize():
        print("❌ Erro na inicialização")
        return False
    
    try:
        print("\nExecutando com coleta de métricas detalhadas...")
        metrics = sim.run_execution(max_duration=30.0)
        
        print("\n" + "─"*50)
        print("Métricas Detalhadas:")
        print("─"*50)
        print(f"Cobertura: {metrics.coverage_percentage:.2f}%")
        print(f"Energia: {metrics.energy_consumed:.2f}J")
        print(f"Duração: {metrics.duration:.2f}s")
        print(f"Taxa média de revisita: {metrics.mean_revisit_rate:.4f}")
        print(f"Células não cobertas: {metrics.uncovered_cells}")
        print(f"Sucesso: {'Sim' if metrics.success else 'Não'}")
        
        # Análise de trajetória
        print(f"\nTrajetória: {len(sim.trajectory)} pontos")
        if sim.trajectory:
            traj_data = sim.trajectory
            print(f"  Tempo inicial: {traj_data[0][0]:.2f}s")
            print(f"  Tempo final: {traj_data[-1][0]:.2f}s")
            print(f"  X: [{min(t[1] for t in traj_data):.2f}, {max(t[1] for t in traj_data):.2f}]")
            print(f"  Y: [{min(t[2] for t in traj_data):.2f}, {max(t[2] for t in traj_data):.2f}]")
        
        print("\n✓ Análise concluída!")
        print(f"  Logs salvos em: logs/")
        
        return True
    
    finally:
        sim.shutdown()


def main():
    """Menu principal."""
    setup_logging()
    
    examples = [
        ("Limpeza Básica com GUI", example_1_basic_cleaning),
        ("Simulação Headless", example_2_headless_simulation),
        ("Ciclos de Aprendizado", example_3_learning_progression),
        ("Comparação de Estratégias", example_4_strategy_comparison),
        ("Integração Node-RED", example_5_node_red_integration),
        ("Análise de Desempenho", example_6_performance_analysis),
    ]
    
    print("\n" + "="*60)
    print("Robô Aspirador - Exemplos")
    print("="*60)
    print("\nExemplos disponíveis:")
    for i, (name, _) in enumerate(examples, 1):
        print(f"  {i}. {name}")
    print(f"  0. Sair")
    
    while True:
        try:
            choice = input("\nEscolha um exemplo (0-6): ").strip()
            
            if choice == '0':
                print("Encerrando...")
                break
            
            idx = int(choice) - 1
            if 0 <= idx < len(examples):
                name, func = examples[idx]
                try:
                    if func():
                        print(f"\n✓ {name} concluído com sucesso!")
                    else:
                        print(f"\n❌ {name} falhou")
                except Exception as e:
                    print(f"\n❌ Erro: {e}")
            else:
                print("❌ Opção inválida")
        
        except KeyboardInterrupt:
            print("\n\nEncerrando...")
            break
        except Exception as e:
            print(f"❌ Erro: {e}")


if __name__ == '__main__':
    main()
