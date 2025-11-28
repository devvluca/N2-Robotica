# Índice de Arquivos - Robô Aspirador

Documentação e localização de todos os componentes.

## 📂 Estrutura Completa

```
robo_aspirador/
├── 📄 README.md                    # Documentação principal
├── 📄 QUICKSTART.md                # Guia 5 minutos
├── 📄 NODE_RED_SETUP.md            # Integração visualização
├── 📄 FILE_INDEX.md                # Este arquivo
├── 🐍 validate.py                  # Validação instalação
├── 🐍 examples.py                  # 6 exemplos práticos
├── 📋 requirements.txt             # Dependências Python
│
├── 📁 config/
│   └── 🔧 robot_config.yaml        # Configuração centralizada
│
├── 📁 src/                         # Código principal
│   ├── 🤖 robot_simulation.py      # Simulador principal (600+ linhas)
│   ├── 🗺️  mapping_module.py        # Mapeamento ocupancy grid (400+ linhas)
│   ├── 🧭 navigation_module.py     # Navegação/exploração (280+ linhas)
│   ├── 🧠 learning_module.py       # Aprendizado/otimização (350+ linhas)
│   ├── 📡 node_red_client.py       # Cliente HTTP Node-RED (400+ linhas)
│   └── 🏗️  environment_setup.py     # Setup PyBullet (350+ linhas)
│
├── 📁 logs/                        # Arquivos executados
│   └── execution_YYYYMMDD_HHMMSS.csv  # Trajetória completa
│
└── 📁 maps/                        # Dados persistidos
    ├── occupancy_grid.npy          # Grade de ocupação
    └── learned_routes.json         # Rotas aprendidas
```

## 📖 Documentação

### Para Iniciantes
- **Comece aqui**: [QUICKSTART.md](QUICKSTART.md) - 5 minutos
- **Entender tudo**: [README.md](README.md) - Documentação completa
- **Visualizar dados**: [NODE_RED_SETUP.md](NODE_RED_SETUP.md) - Dashboard

### Para Desenvolvedores
- **Módulo de mapeamento**: `src/mapping_module.py` (400 linhas, bem documentado)
- **Módulo de navegação**: `src/navigation_module.py` (280 linhas)
- **Módulo de aprendizado**: `src/learning_module.py` (350 linhas)
- **Cliente Node-RED**: `src/node_red_client.py` (400 linhas)
- **Simulador**: `src/robot_simulation.py` (600 linhas)
- **Ambiente**: `src/environment_setup.py` (350 linhas)

## 🚀 Início Rápido

```bash
# 1. Validar instalação
python validate.py

# 2. Rodar exemplos
python examples.py

# 3. Editar configuração
nano config/robot_config.yaml

# 4. Criar script customizado
python -c "..."
```

## 📊 Arquivos de Saída

### CSV de Execução (`logs/execution_*.csv`)

Colunas:
- `time`: Timestamp em segundos
- `x`, `y`, `heading`: Posição e orientação
- `linear_vel`, `angular_vel`: Velocidades
- `coverage`: % de cobertura acumulada
- `energy`: Energia total consumida
- `state`: Estado do robô (IDLE, MOVING, AVOIDING, etc)
- `strategy`: Estratégia de exploração (spiral, systematic, random)

Uso:
```python
import pandas as pd
df = pd.read_csv('logs/execution_*.csv')
print(f"Cobertura final: {df['coverage'].iloc[-1]:.1f}%")
```

### Grade de Ocupação (`maps/occupancy_grid.npy`)

Formato: NumPy array float32 2D (width × height)
- Valores: 0-1 (0=livre, 1=ocupado)
- Resolução: 0.1m por célula (configurável)
- Uso:
```python
import numpy as np
grid = np.load('maps/occupancy_grid.npy')
print(f"Grid shape: {grid.shape}")
print(f"Ocupação: {grid.mean()*100:.1f}%")
```

### Rotas Aprendidas (`maps/learned_routes.json`)

```json
{
  "execution_history": [
    {
      "execution_id": 1,
      "coverage": 82.5,
      "energy": 18.3,
      "strategy": "spiral"
    }
  ],
  "best_strategy": "systematic",
  "convergence_reached": false
}
```

## 🔧 Arquivos de Configuração

### `config/robot_config.yaml`

Controla tudo:

```yaml
robot:              # Specs físicos do robô
simulation:         # Parâmetros de física
mapping:            # Grid ocupação
navigation:         # Exploração
learning:           # Otimização
environment:        # Mundo/obstáculos
node_red:           # Integração
logging:            # Logs CSV
```

Editar antes de rodar simular para customizar.

## 🐍 Scripts Python

### `validate.py` - Verificação

```bash
python validate.py
```

Valida:
- ✓ Versão Python
- ✓ Estrutura arquivos
- ✓ Arquivo configuração
- ✓ Dependências instaladas

### `examples.py` - Menu Interativo

```bash
python examples.py
```

6 exemplos:
1. Limpeza básica (GUI)
2. Headless (rápido)
3. Ciclos aprendizado
4. Comparação estratégias
5. Node-RED integração
6. Análise desempenho

## 📦 Dependências (`requirements.txt`)

```
pybullet==3.2.5      # Simulação física 3D
numpy==1.24.3        # Computação numérica
pyyaml==6.0          # Carregar YAML
requests==2.31.0     # HTTP client
scipy==1.11.1        # Processamento sinais
```

Instalar:
```bash
pip install -r requirements.txt
```

## 🎯 Casos de Uso Recomendados

### Pesquisa Acadêmica
→ Usar módulos individuais
→ Estender com novos algoritmos
→ Publicar resultados de cobertura

### Educação
→ Usar `QUICKSTART.md`
→ Rodar `examples.py`
→ Analisar dados em CSV

### Prototipar Algoritmos
→ Editar `src/navigation_module.py`
→ Manter interface compatível
→ Executar comparações de estratégia

### Integração Sistemas
→ Usar Node-RED para dashboard
→ Persistir dados em banco
→ Alertas em tempo real

## 📈 Fluxo de Trabalho Típico

```
1. Clonar projeto
   ↓
2. Rodar validate.py
   ↓
3. Editar robot_config.yaml
   ↓
4. Rodar examples.py
   ↓
5. Analisar logs/ e maps/
   ↓
6. Criar script customizado
   ↓
7. Integrar Node-RED (opcional)
```

## 🔗 Relacionamentos Entre Módulos

```
robot_simulation.py (main)
    ├→ environment_setup.py (criar mundo)
    ├→ mapping_module.py (rastrear cobertura)
    ├→ navigation_module.py (controlar movimento)
    ├→ learning_module.py (otimizar estratégia)
    └→ node_red_client.py (enviar dados)
```

### Ordem de Inicialização

1. `VacuumRobotSimulation.__init__()` - Setup
2. `VacuumRobotSimulation.initialize()` - Criar mundo
   - `EnvironmentSetup.setup_world()` - Paredes
   - `EnvironmentSetup.load_obstacles()` - Móveis
   - `MappingModule.__init__()` - Grid inicial
   - `NavigationModule.__init__()` - Estados
   - `LearningModule.__init__()` - Histórico
   - `VacuumNodeREDClient.connect()` - HTTP

3. `VacuumRobotSimulation.run_execution()` - Executar
   - Loop principal:
     - Ler sensores
     - Atualizar mapa
     - Computar velocidade
     - Aplicar força
     - Enviar telemetria

4. `VacuumRobotSimulation.shutdown()` - Limpeza
   - Salvar mapa
   - Persistir rotas
   - Desconectar Node-RED

## 💾 Métodos Principais por Módulo

### MappingModule
- `process_sensor_data()` - Atualizar com leituras
- `get_coverage_percentage()` - % coberto
- `get_uncovered_cells()` - Células não visitadas
- `save()` / `load()` - Persistência

### NavigationModule
- `compute_velocity()` - Gerar velocidades
- `set_exploration_strategy()` - Mudar tática
- `_wall_following_correction()` - Seguir parede

### LearningModule
- `record_execution()` - Registrar ciclo
- `should_continue_learning()` - Convergência?
- `get_next_strategy()` - Qual próxima?
- `save_learning_data()` - Persistir aprendizado

### VacuumRobotSimulation
- `initialize()` - Setup completo
- `run_execution()` - Uma limpeza
- `run_learning_cycles()` - Múltiplas com aprendizado
- `shutdown()` - Limpeza recursos

## 🎓 Recursos de Aprendizado

### Dentro do Projeto
1. Ler `README.md` - Entender arquitetura
2. Ler `QUICKSTART.md` - Começar rápido
3. Rodar `examples.py` - Ver na prática
4. Editar `config/robot_config.yaml` - Experimentar
5. Ler código em `src/` - Implementação

### Online
- [PyBullet Docs](https://pybullet.org/wordpress/)
- [NumPy Tutorial](https://numpy.org/learn/)
- [Node-RED Guide](https://nodered.org/docs)
- [Ocupancy Grid](https://en.wikipedia.org/wiki/Occupancy_grid_mapping)

## ⚡ Performance

### Tempos Típicos (Desktop i5)

| Operação | Tempo |
|----------|-------|
| Validação | 2s |
| Inicialização | 3s |
| Passo simulação | 0.01-0.05s |
| 60s execução (GUI) | 2-3min real |
| 60s execução (headless) | 10-15s |
| 5 ciclos aprendizado | 1-2min |

### Otimizações Possíveis
- Aumentar `time_step` em config (mais rápido, menos preciso)
- Desabilitar GUI (`use_gui=False`)
- Reduzir `grid_resolution` (menos memória)
- Usar estratégia random (mais rápida)

## 🐛 Debug e Troubleshooting

### Verificar Dados
```python
# CSV
df = pd.read_csv('logs/execution_*.csv')
print(df.describe())

# Grid
grid = np.load('maps/occupancy_grid.npy')
print(f"Mean occupancy: {grid.mean()}")

# Rotas
import json
data = json.load(open('maps/learned_routes.json'))
print(data)
```

### Logs de Execução
```python
import logging
logging.basicConfig(level=logging.DEBUG)
# Rodar simulação, muitos detalhes nos logs
```

### Inspecionar Sensores
```python
sim = VacuumRobotSimulation(config)
sim.initialize()
for name, sensor in sim.sensors.items():
    print(f"{name}: {sensor.read()}m")
```

## 📞 Suporte Rápido

| Problema | Solução |
|----------|---------|
| Não entendo | Ler QUICKSTART.md |
| Erro módulo | Rodar validate.py |
| Dados ruins | Verificar config/robot_config.yaml |
| Performance | Aumentar time_step ou desabilitar GUI |
| Node-RED | Ver NODE_RED_SETUP.md |

---

**Última atualização**: 2024-01-15

Todos os 9 arquivos principais + 3 documentação = Projeto Completo ✅
