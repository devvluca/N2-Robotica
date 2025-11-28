# Robô Aspirador - Simulador com Aprendizado

Simulador de robô aspirador inteligente em PyBullet com mapeamento, navegação autônoma e aprendizado por reforço.

## 🤖 Características

### Simulação Física
- **PyBullet 3D**: Simulação realista de dinâmica física
- **Robô Diferencial**: Dois motores independentes com cinemática realista
- **Sensores Ultrassônicos**: 5 sensores com ruído simulado para detecção de obstáculos
- **Ambiente Configurável**: Paredes, móveis e obstáculos ajustáveis

### Mapeamento e Navegação
- **Ocupancy Grid 2D**: Mapeamento em grade com resolução configurável
- **Cobertura Dinâmica**: Rastreamento de áreas visitadas e limpas
- **Detecção de Obstáculos**: Ray-casting com processamento de sensores
- **Evitação de Colisões**: Algoritmos reativos baseados em sensores

### Exploração e Aprendizado
- **3 Estratégias de Exploração**:
  - Espiral (raio crescente)
  - Sistemática (varredura de linhas)
  - Aleatória (caminhada aleatória)
- **Otimização de Rotas**: Aprendizado da melhor estratégia entre execuções
- **Detecção de Convergência**: Parada automática ao atingir eficiência máxima
- **Persistência**: Salvamento de mapas e rotas aprendidas

### Integração com Ferramentas
- **Node-RED HTTP**: Envio de telemetria em tempo real
- **Logging CSV**: Registro completo de trajetória e sensores
- **Visualização PyBullet**: GUI interativa opcional

## 📋 Requisitos

- Python 3.8+
- PyBullet 3.2.5
- NumPy, SciPy
- PyYAML
- Requests

## 🚀 Instalação Rápida

```bash
# 1. Clonar/copiar projeto
cd robo_aspirador

# 2. Instalar dependências
pip install -r requirements.txt

# 3. Validar instalação
python validate.py

# 4. Executar exemplos
python examples.py
```

## 📁 Estrutura do Projeto

```
robo_aspirador/
├── config/
│   └── robot_config.yaml          # Configuração centralizada
├── src/
│   ├── robot_simulation.py         # Simulador principal
│   ├── mapping_module.py           # Mapeamento ocupancy grid
│   ├── navigation_module.py        # Navegação e exploração
│   ├── learning_module.py          # Aprendizado e otimização
│   ├── node_red_client.py          # Cliente HTTP Node-RED
│   └── environment_setup.py        # Setup do ambiente PyBullet
├── logs/                           # Arquivos de log CSV
├── maps/                           # Mapas persistidos
├── examples.py                     # Exemplos de uso
├── validate.py                     # Validação de instalação
├── requirements.txt                # Dependências
└── README.md                       # Este arquivo
```

## ⚙️ Configuração

Editar `config/robot_config.yaml` para personalizar:

```yaml
robot:
  radius: 0.15              # Raio em metros
  mass: 5.0                 # Massa em kg
  max_linear_velocity: 0.5  # Velocidade max (m/s)
  max_angular_velocity: 2.0 # Velocidade angular max (rad/s)
  sensors:
    count: 5                # Número de sensores
    range: 1.0              # Alcance dos sensores (m)

simulation:
  time_step: 0.01           # Passo de tempo (s)
  gravity: 9.81             # Gravidade (m/s²)
  gui_enabled: true         # Mostrar visualização

mapping:
  grid_resolution: 0.1      # Resolução da grade (m)
  occupancy_threshold: 0.5  # Limiar de ocupação

navigation:
  obstacle_threshold: 0.3   # Distância para evitar (m)
  wall_following: true      # Seguir paredes

learning:
  max_iterations: 5         # Máximo de ciclos
  convergence_threshold: 0.01  # Threshold de convergência

environment:
  world_width: 10.0         # Largura do mundo (m)
  world_height: 8.0         # Altura do mundo (m)
  obstacles:
    - name: "furniture_1"
      x: 5.0
      y: 4.0
      width: 1.0
      height: 1.0
```

## 📖 Exemplos de Uso

### 1. Limpeza Básica com Visualização

```python
from src.robot_simulation import VacuumRobotSimulation
import yaml

with open('config/robot_config.yaml') as f:
    config = yaml.safe_load(f)

sim = VacuumRobotSimulation(config, use_gui=True)
sim.initialize()

metrics = sim.run_execution(max_duration=60.0)
print(f"Cobertura: {metrics.coverage_percentage:.1f}%")

sim.shutdown()
```

### 2. Simulação Rápida sem GUI

```python
sim = VacuumRobotSimulation(config, use_gui=False)
sim.initialize()

# 5 ciclos de execução
all_metrics = sim.run_learning_cycles(num_cycles=5)

# Analisar progresso
for i, metrics in enumerate(all_metrics):
    print(f"Ciclo {i+1}: {metrics.coverage_percentage:.1f}%")

sim.shutdown()
```

### 3. Comparar Estratégias

```python
strategies = ['spiral', 'systematic', 'random']

for strategy in strategies:
    sim = VacuumRobotSimulation(config, use_gui=False)
    sim.initialize()
    
    sim.navigation.set_exploration_strategy(strategy)
    metrics = sim.run_execution(max_duration=45.0)
    
    print(f"{strategy}: {metrics.coverage_percentage:.1f}% - "
          f"{metrics.energy_consumed:.2f}J")
    
    sim.shutdown()
```

### 4. Menu Interativo

Executar `python examples.py` para menu com 6 exemplos práticos.

## 📊 Análise de Resultados

Os resultados são salvos em:

- **logs/execution_YYYYMMDD_HHMMSS.csv**: Trajetória completa e sensores
- **maps/occupancy_grid.npy**: Grade de ocupação final
- **maps/learned_routes.json**: Rotas aprendidas

### Colunas do CSV

```
time          - Timestamp em segundos
x, y, heading - Posição e orientação do robô
linear_vel, angular_vel - Velocidades
coverage      - Percentual de cobertura até este ponto
energy        - Energia total consumida
state         - Estado do robô (IDLE, MOVING, AVOIDING, etc)
strategy      - Estratégia de navegação ativa
```

### Métricas Principais

- **Cobertura (%)**: Percentual da área explorada
- **Energia (J)**: Energia gasta na limpeza
- **Eficiência**: Cobertura / Energia
- **Taxa de Revisita**: Quantas vezes células são revisitadas
- **Tempo**: Duração total da execução

## 🔗 Integração com Node-RED

### Setup Node-RED

1. Instalar Node-RED: `npm install -g node-red`
2. Iniciar: `node-red`
3. Acessar: http://localhost:1880
4. Importar flow em `NODE_RED_BASIC_FLOW.json`

### Enviar Dados

```python
config['node_red']['enabled'] = True

sim = VacuumRobotSimulation(config, use_gui=False)
sim.initialize()

# Telemetria enviada automaticamente a 5Hz
metrics = sim.run_execution()

sim.shutdown()
```

### Visualização no Node-RED

O flow configura:
- Gráfico de cobertura vs tempo
- Posição em tempo real (XY)
- Leituras de sensores (distâncias)
- Métricas de energia
- Heatmap de ocupação

## 🧪 Testes e Validação

### Rodar Validação

```bash
python validate.py
```

Verifica:
- ✓ Versão do Python
- ✓ Estrutura do projeto
- ✓ Arquivo de configuração
- ✓ Dependências instaladas

### Rodar Testes

```bash
# Teste rápido de 10 segundos
python -c "
from src.robot_simulation import VacuumRobotSimulation
import yaml
with open('config/robot_config.yaml') as f:
    config = yaml.safe_load(f)
sim = VacuumRobotSimulation(config, use_gui=False)
sim.initialize()
sim.run_execution(max_duration=10.0)
sim.shutdown()
print('✓ Teste passou!')
"
```

## 📈 Desempenho

### Tempos Típicos

- **Inicialização**: ~2-5 segundos
- **Passo de simulação**: ~0.01-0.05 segundos
- **Execução 60s (GUI)**: ~2-3 minutos real
- **Execução 60s (headless)**: ~10-15 segundos

### Hardware Recomendado

- CPU: 2+ núcleos (Intel i5/Ryzen 5+)
- RAM: 4GB+
- GPU: Opcional (sem suporte atualmente)

## 🐛 Troubleshooting

### Erro: "ModuleNotFoundError: No module named 'pybullet'"

```bash
pip install pybullet==3.2.5
```

### Erro: "YAML file not found"

Confirmar localização do arquivo:
```bash
ls -la config/robot_config.yaml
```

### PyBullet GUI não abre

- Em Linux: Instalar dependências gráficas
- Em WSL: Usar `use_gui=False`
- Em Windows: Executar como administrador

### Conexão Node-RED falha

```bash
# Verificar se Node-RED está rodando
curl http://localhost:1880

# Reiniciar Node-RED
node-red --port 1880
```

### Simulação muito lenta

- Reduzir `time_step` em robot_config.yaml
- Desabilitar GUI: `use_gui=False`
- Reduzir `grid_resolution` (menor precisão)

## 📚 Documentação Técnica

### Módulos Principais

#### **robot_simulation.py**
- `VacuumRobotSimulation`: Classe principal de simulação
- `UltrasonicSensor`: Simulação de sensor ultrassônico
- Métodos: `initialize()`, `run_execution()`, `run_learning_cycles()`

#### **mapping_module.py**
- `OccupancyGrid`: Grade de ocupação 2D
- `MappingModule`: Integração com sensores
- Métodos: `process_sensor_data()`, `get_coverage_percentage()`

#### **navigation_module.py**
- `NavigationModule`: Controle reativo
- `ExplorationStrategy`: Estratégias de exploração (3 tipos)
- Métodos: `compute_velocity()`, `set_exploration_strategy()`

#### **learning_module.py**
- `LearningModule`: Aprendizado multi-execução
- `RouteOptimizer`: Otimização de rotas
- Métodos: `record_execution()`, `get_next_strategy()`

#### **node_red_client.py**
- `VacuumNodeREDClient`: Cliente HTTP
- `TelemetryAggregator`: Montagem de pacotes
- Método: `send_telemetry()`, `send_metrics()`

#### **environment_setup.py**
- `EnvironmentSetup`: Gerenciador do ambiente
- `Obstacle`: Classe para obstáculos
- Métodos: `setup_world()`, `load_obstacles()`

## 🎯 Objetivos de Desempenho

Benchmarks esperados (com hardware padrão):

| Métrica | Valor | Unidade |
|---------|-------|--------|
| Cobertura (spiral) | 85-92 | % |
| Cobertura (systematic) | 78-88 | % |
| Cobertura (random) | 60-75 | % |
| Energia típica | 15-25 | J |
| Eficiência best | 4.0-5.5 | %/J |
| Tempo convergência | 3-5 | ciclos |

## 🔄 Fluxo de Aprendizado

1. **Execução 1**: Testa estratégia padrão (espiral)
2. **Execução 2**: Testa segunda estratégia (sistemática)
3. **Execução 3**: Testa terceira estratégia (aleatória)
4. **Executação 4+**: Repetem melhor estratégia detectada
5. **Convergência**: Quando melhoria < threshold

## 📝 Licença

MIT License

## 👨‍💻 Autor

Desenvolvido para fins educacionais em robótica autônoma.

## 🤝 Contribuições

Contribuições bem-vindas! Áreas de interesse:

- [ ] Suporte a GPU
- [ ] Múltiplos robôs
- [ ] Interfaces Web
- [ ] Novos sensores (LIDAR, câmera)
- [ ] Planejamento de trajetória (RRT)
- [ ] Machine learning avançado

## 📞 Suporte

Para problemas, consulte:
- Logs em `logs/`
- Configuração em `config/robot_config.yaml`
- Exemplos em `examples.py`
