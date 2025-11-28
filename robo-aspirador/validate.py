"""
Script de validação da instalação do robô aspirador.

Verifica dependências, compatibilidade e pré-requisitos.
"""

import sys
import subprocess
from pathlib import Path


def check_python_version() -> bool:
    """Verifica versão do Python."""
    if sys.version_info < (3, 8):
        print(f"❌ Python 3.8+ requerido. Encontrado: {sys.version}")
        return False
    print(f"✓ Python {sys.version.split()[0]}")
    return True


def check_dependencies() -> bool:
    """Verifica dependências instaladas."""
    dependencies = {
        'pybullet': 'PyBullet (simulador físico)',
        'numpy': 'NumPy (computação numérica)',
        'pyyaml': 'PyYAML (configuração)',
        'requests': 'Requests (HTTP client)',
        'scipy': 'SciPy (processamento de sinais)',
    }
    
    all_ok = True
    for module, description in dependencies.items():
        try:
            __import__(module)
            print(f"✓ {description}")
        except ImportError:
            print(f"❌ {description} - não instalado")
            all_ok = False
    
    return all_ok


def check_project_structure() -> bool:
    """Verifica estrutura do projeto."""
    required_files = [
        'config/robot_config.yaml',
        'src/robot_simulation.py',
        'src/mapping_module.py',
        'src/navigation_module.py',
        'src/learning_module.py',
        'src/node_red_client.py',
        'src/environment_setup.py',
        'requirements.txt',
    ]
    
    all_ok = True
    for file_path in required_files:
        if Path(file_path).exists():
            print(f"✓ {file_path}")
        else:
            print(f"❌ {file_path} - não encontrado")
            all_ok = False
    
    return all_ok


def check_config_file() -> bool:
    """Verifica se o arquivo de configuração é válido."""
    try:
        import yaml
        with open('config/robot_config.yaml', 'r') as f:
            config = yaml.safe_load(f)
        
        required_sections = ['robot', 'simulation', 'mapping', 'navigation', 'learning', 'environment']
        missing = [s for s in required_sections if s not in config]
        
        if missing:
            print(f"❌ Seções faltando em robot_config.yaml: {missing}")
            return False
        
        print("✓ Arquivo de configuração válido")
        return True
    
    except Exception as e:
        print(f"❌ Erro ao ler configuração: {e}")
        return False


def install_dependencies() -> bool:
    """Instala dependências faltantes."""
    try:
        print("\nInstalando dependências...")
        subprocess.check_call(
            [sys.executable, '-m', 'pip', 'install', '-r', 'requirements.txt']
        )
        print("✓ Dependências instaladas")
        return True
    except Exception as e:
        print(f"❌ Erro ao instalar: {e}")
        return False


def main():
    """Executa validação completa."""
    print("=" * 50)
    print("Validação - Robô Aspirador")
    print("=" * 50)
    
    checks = [
        ("Python Version", check_python_version),
        ("Project Structure", check_project_structure),
        ("Configuration File", check_config_file),
        ("Dependencies", check_dependencies),
    ]
    
    results = {}
    for name, check_func in checks:
        print(f"\n{name}:")
        try:
            results[name] = check_func()
        except Exception as e:
            print(f"❌ Erro: {e}")
            results[name] = False
    
    # Resumo
    print("\n" + "=" * 50)
    print("Resumo:")
    for name, result in results.items():
        status = "✓ PASSOU" if result else "❌ FALHOU"
        print(f"  {name}: {status}")
    
    all_passed = all(results.values())
    
    if not all_passed:
        print("\nDeseja instalar dependências faltantes? (s/n) ", end="")
        if input().lower() == 's':
            install_dependencies()
    
    print("\n" + "=" * 50)
    if all_passed:
        print("✓ Todas as verificações passaram!")
        print("\nPróximos passos:")
        print("  1. Revisar config/robot_config.yaml")
        print("  2. Executar: python examples.py")
        print("  3. Ver resultados em logs/")
    else:
        print("❌ Resolva os problemas acima antes de continuar")
    
    return all_passed


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
