# Simulação do Manipulador Planar 2 DOF com Controle PID e Node-RED

Sistema completo de simulação de um manipulador planar com 2 graus de liberdade (DOF) em PyBullet, com controle PID individual para cada junta, integração com Node-RED para visualização em tempo real e coleta de métricas de desempenho.

## 📋 Características

✅ **Simulação Física Realista** - Ambiente PyBullet com gravidade, inércia e dinâmica completa
✅ **Controle PID Avançado** - Controladores independentes para cada junta com anti-windup
✅ **Cinemática Direta e Inversa** - Cálculos de posição do end-effector e espaço de trabalho
✅ **Logging Detalhado** - Coleta contínua de métricas e dados de desempenho
✅ **Integração Node-RED** - Envio em tempo real de dados via HTTP para visualização
✅ **Análise de Desempenho** - Cálculo de settling time, overshoot, energia consumida
✅ **Perturbações Externas** - Teste de robustez com aplicação de forças
✅ **Visualização 3D** - Renderização 3D do manipulador e trajetória do end-effector

## 🏗️ Estrutura do Projeto

```
manipulador_planar/
├── src/
│   ├── robot_simulation.py      # Simulação principal
│   ├── pid_controller.py        # Controladores PID
│   ├── kinematics.py           # Cinemática direta/inversa
│   └── node_red_client.py      # Cliente HTTP para Node-RED
├── config/
│   └── robot_config.yaml       # Configuração centralizada
├── logs/                        # Arquivos de log e métricas
├── requirements.txt            # Dependências
└── README.md                   # Este arquivo
```

## 🔧 Instalação

### Pré-requisitos
- Python 3.8+
- pip

### Passo 1: Clonar e instalar dependências

```bash
cd manipulador_planar
pip install -r requirements.txt
```

### Passo 2: Configurar Node-RED (opcional)

Se você possui um servidor Node-RED em `localhost:1880`, a simulação se conectará automaticamente.

Edite `config/robot_config.yaml` para alterar o host/porta:

```yaml
node_red:
  host: "seu_servidor.com"
  port: 1880
  enabled: true
```

## 🚀 Uso

### Executar simulação básica

```bash
python src/robot_simulation.py
```

A simulação vai:
1. Inicializar o ambiente PyBullet com GUI
2. Criar o manipulador planar
3. Executar por 5 segundos com referência θ1=45°, θ2=-30°
4. Aplicar uma perturbação no meio da simulação
5. Salvar métricas em `logs/metrics_YYYYMMDD_HHMMSS.csv`

### Usar em seu próprio script

```python
from src.robot_simulation import RobotSimulation
import numpy as np

# Criar simulação
sim = RobotSimulation(enable_gui=True)

# Definir referência angular
sim.set_reference_angles(np.pi/4, -np.pi/6)

# Executar 10 segundos
dt = sim.config['simulation']['time_step']
for _ in range(int(10 / dt)):
    sim.step(dt)

# Salvar dados
sim.save_metrics()
sim.shutdown()
```

## ⚙️ Configuração

Todos os parâmetros estão em `config/robot_config.yaml`:

### Parâmetros do Robô

```yaml
robot:
  link1_length: 0.5    # Comprimento do link 1 (metros)
  link2_length: 0.3    # Comprimento do link 2 (metros)
  link1_mass: 1.0      # Massa do link 1 (kg)
  link2_mass: 1.0      # Massa do link 2 (kg)
  joint_limits:
    min: -3.14159265359  # -π rad
    max: 3.14159265359   # +π rad
```

### Parâmetros PID

```yaml
pid_controller:
  joint1:
    kp: 50.0      # Ganho proporcional
    ki: 10.0      # Ganho integral
    kd: 5.0       # Ganho derivativo
    integral_max: 10.0   # Limite anti-windup
  joint2:
    kp: 50.0
    ki: 10.0
    kd: 5.0
    integral_max: 10.0
  max_torque: 50.0  # Torque máximo (N.m)
```

**Dicas para ajuste PID:**
- Aumentar `kp` para resposta mais rápida
- Aumentar `kd` para reduzir overshoot
- Aumentar `ki` para corrigir erro em estado estacionário
- Usar `integral_max` para anti-windup (evita saturação)

### Parâmetros de Perturbação

```yaml
perturbation:
  enabled: true
  force_magnitude: [1.0, 1.0, 0.0]  # Força em N (x, y, z)
  force_duration: 0.5               # Duração da força (s)
  force_application_point: [0.5, 0.15, 0.0]  # Ponto de aplicação
```

## 📊 Integração Node-RED

### Flow JSON Básico

Importe este JSON no Node-RED para receber dados da simulação:

```json
[
  {
    "id": "robot_input",
    "type": "http in",
    "method": "post",
    "url": "/data/robot",
    "name": "Robot Data Input"
  },
  {
    "id": "robot_parse",
    "type": "json",
    "action": "parse"
  },
  {
    "id": "robot_debug",
    "type": "debug",
    "name": "Robot State"
  },
  {
    "id": "robot_response",
    "type": "http response",
    "statusCode": "200"
  },
  {
    "id": "robot_storage",
    "type": "mongodb out",
    "database": "robot_sim",
    "collection": "metrics",
    "name": "Store Metrics"
  }
]
```

### Conectar a banco de dados

Para persistência, configure uma conexão MongoDB:

```javascript
// Function node para preparar dados
msg.payload = {
  timestamp: msg.payload.timestamp,
  angles: msg.payload.joint_angles,
  reference: msg.payload.reference_angles,
  end_effector: msg.payload.end_effector_position,
  metrics: msg.payload.metrics
};
return msg;
```

## 📈 Métricas Coletadas

A simulação coleta automaticamente:

| Métrica | Descrição | Unidade |
|---------|-----------|---------|
| `joint_angles` | Ângulos atuais das juntas | rad |
| `reference_angles` | Ângulos de referência | rad |
| `errors` | Diferença referência - atual | rad |
| `torques` | Torques aplicados nas juntas | N.m |
| `end_effector_position` | Posição (x, y) do efetador | m |
| `end_effector_distance` | Distância da origem | m |
| `settling_time` | Tempo para atingir 95% da referência | s |
| `overshoot` | Excesso máximo sobre a referência | rad |
| `steady_state_error` | Erro final em estado estacionário | rad |
| `energy_consumed` | Energia gasta ∫(τ²)dt | J |

## 🎯 Cinemática

### Cinemática Direta

Calcula a posição do end-effector dados os ângulos das juntas:

```
x = L₁·cos(θ₁) + L₂·cos(θ₁ + θ₂)
y = L₁·sin(θ₁) + L₂·sin(θ₁ + θ₂)
```

Uso:
```python
fk_result = sim.kinematics.forward_kinematics((theta1, theta2))
print(fk_result['ee_position'])  # [x, y]
print(fk_result['ee_distance'])  # Distância da origem
```

### Cinemática Inversa

Encontra os ângulos necessários para atingir uma posição cartesiana:

```python
ik_result = sim.kinematics.inverse_kinematics((x, y))
if ik_result['valid']:
    theta1, theta2 = ik_result['theta']
else:
    print(ik_result['error'])
```

### Jacobiano

Matriz que relaciona velocidades angulares com velocidades cartesianas:

```python
J = sim.kinematics.jacobian((theta1, theta2))
velocity_cartesiana = J @ [omega1, omega2]
```

## 🔍 Análise de Resultados

Os dados são salvos em `logs/metrics_*.csv` com as seguintes colunas:

```csv
time,joint1_angle,joint2_angle,joint1_ref,joint2_ref,joint1_error,joint2_error,joint1_torque,joint2_torque,ee_x,ee_y,ee_distance
```

### Visualizar em Python

```python
import pandas as pd
import matplotlib.pyplot as plt

# Carregar dados
df = pd.read_csv('logs/metrics_20240101_120000.csv')

# Plotar ângulos
plt.figure(figsize=(12, 6))
plt.plot(df['time'], df['joint1_angle'], label='θ₁ Atual')
plt.plot(df['time'], df['joint1_ref'], label='θ₁ Referência', linestyle='--')
plt.xlabel('Tempo (s)')
plt.ylabel('Ângulo (rad)')
plt.legend()
plt.grid()
plt.show()
```

## 🐛 Troubleshooting

### Simulação muito lenta
- Reduzir `simulation_frequency` em `robot_config.yaml`
- Desabilitar GUI com `enable_gui: false`
- Reduzir tempo de simulação

### Node-RED não recebe dados
- Verificar se endpoint está correto: `http://localhost:1880/data/robot`
- Habilitar logs: `tail -f logs/simulation_*.log`
- Testar com curl: `curl -X POST http://localhost:1880/data/robot -H "Content-Type: application/json" -d "{}"`

### Controlador instável
- Reduzir `kp` (ganho proporcional)
- Aumentar `kd` (ganho derivativo)
- Verificar limites de torque com `max_torque`

## 📝 Exemplos de Uso Avançado

### Teste de seguimento de trajetória

```python
sim = RobotSimulation()
times = np.linspace(0, 5, 500)
theta1_ref = 0.5 * np.sin(2 * np.pi * 0.5 * times)
theta2_ref = 0.3 * np.cos(2 * np.pi * 0.5 * times)

for i, t in enumerate(times):
    sim.set_reference_angles(theta1_ref[i], theta2_ref[i])
    sim.step(0.01)
```

### Otimização de ganhos PID

```python
from scipy.optimize import minimize

def calculate_error(gains):
    sim = RobotSimulation()
    sim.pid_controller.update_gains(1, kp=gains[0], ki=gains[1], kd=gains[2])
    # ... executar simulação ...
    return total_error

optimal_gains = minimize(calculate_error, [50, 10, 5])
```

## 📚 Referências Técnicas

### Controlador PID com Anti-Windup

```
u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·de/dt

Integral clamped em [I_min, I_max] para evitar wind-up
```

### Modelo Dinâmico do Manipulador

O PyBullet simula:
- Cinemática direta/inversa
- Dinâmica de corpos rígidos
- Atrito nas juntas (configurável)
- Colisões com ambiente

## 📄 Licença

Este projeto é fornecido como exemplo educacional.

## 🤝 Contribuições

Melhorias bem-vindas! Envie pull requests com:
- Novos controladores
- Modelos de robôs alternativos
- Análises de desempenho
- Documentação

## ✉️ Suporte

Para dúvidas ou problemas:
1. Verifique os logs em `logs/`
2. Consulte a documentação do PyBullet
3. Revise os parâmetros em `robot_config.yaml`
