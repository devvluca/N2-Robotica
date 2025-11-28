# Guia de Integração com Node-RED

Visualize dados do robô aspirador em tempo real com Node-RED.

## 🚀 Instalação Rápida

### 1. Instalar Node-RED

```bash
# Global
npm install -g node-red

# Ou com apt (Linux)
sudo apt-get install npm
npm install -g node-red
```

### 2. Iniciar Node-RED

```bash
node-red
```

Acessar: http://localhost:1880

### 3. Configurar Endpoint

No Node-RED, criar um **HTTP IN** node:

```
Method: POST
URL: /vacuum/data
```

## 📊 Dados Recebidos

Estrutura JSON enviada por `node_red_client.py`:

### Telemetria (5Hz)

```json
{
  "timestamp": "2024-01-15T10:30:45.123456",
  "robot_id": "vacuum_001",
  "pose": {
    "x": 5.234,
    "y": 4.123,
    "heading": 1.57
  },
  "sensor_readings": {
    "sensor_0": 0.85,
    "sensor_1": 1.00,
    "sensor_2": 0.65,
    "sensor_3": 0.92,
    "sensor_4": 0.78
  },
  "velocity": {
    "linear": 0.35,
    "angular": 0.45
  },
  "coverage_percentage": 78.5,
  "energy_consumed": 12.34,
  "state": "MOVING"
}
```

### Métricas (1Hz)

```json
{
  "timestamp": "2024-01-15T10:30:45.654321",
  "execution_id": 3,
  "total_coverage": 87.2,
  "total_energy": 18.45,
  "mean_revisits": 0.234,
  "execution_time": 45.6,
  "efficiency_score": 4.72,
  "learning_iteration": 2
}
```

## 🎨 Dashboard Recomendado

### Layout

```
┌────────────────────────────────────────┐
│      Robô Aspirador - Dashboard        │
├─────────────────┬──────────────────────┤
│                 │                      │
│  Mapa de        │  Sensores Ultra-    │
│  Ocupação       │  sônicos (radial)   │
│  (heatmap)      │                      │
│                 │                      │
├─────────────────┼──────────────────────┤
│  Cobertura %    │  Energia (J)         │
│  [████████░░]   │  [████░░░░░░]        │
│  87.3%          │  18.45J              │
├─────────────────┼──────────────────────┤
│  Trajetória     │  Velocidades         │
│  (XY plot)      │  Linear: 0.35 m/s    │
│                 │  Angular: 0.45 rad/s │
└─────────────────┴──────────────────────┘
```

### Nós Necessários

1. **HTTP IN** - Receber dados
2. **JSON Parse** - Converter string → objeto
3. **ui_gauge** - Mostrar cobertura %
4. **ui_chart** - Gráficos de série temporal
5. **ui_template** - Visualizar mapa ocupação
6. **ui_text** - Status texto

## 💻 Flow Básico

### Node-RED JSON

```json
[
  {
    "id": "http_in",
    "type": "http in",
    "method": "post",
    "url": "/vacuum/data",
    "name": "Dados Robô"
  },
  {
    "id": "parse_json",
    "type": "json",
    "property": "payload",
    "action": "str"
  },
  {
    "id": "coverage_gauge",
    "type": "ui_gauge",
    "title": "Cobertura (%)",
    "min": 0,
    "max": 100,
    "property": "payload.coverage_percentage"
  },
  {
    "id": "energy_gauge",
    "type": "ui_gauge",
    "title": "Energia (J)",
    "min": 0,
    "max": 50,
    "property": "payload.energy_consumed"
  },
  {
    "id": "trajectory_chart",
    "type": "ui_chart",
    "chartType": "scatter",
    "title": "Trajetória XY",
    "xproperty": "payload.pose.x",
    "yproperty": "payload.pose.y"
  },
  {
    "id": "sensor_polar",
    "type": "ui_template",
    "title": "Sensores (Radial)",
    "template": "<polar-chart></polar-chart>"
  }
]
```

## 🔧 Configuração Passo a Passo

### 1. Criar HTTP Endpoint

```
1. Abrir Node-RED
2. Menu > Imports > Clipboard
3. Colar flow JSON acima
4. Deploy
```

### 2. Habilitar em Robot Config

```yaml
node_red:
  enabled: true
  host: localhost
  port: 1880
  endpoint: /vacuum/data
  robot_id: vacuum_001
```

### 3. Iniciar Simulação com Node-RED

```python
from src.robot_simulation import VacuumRobotSimulation
import yaml

with open('config/robot_config.yaml') as f:
    config = yaml.safe_load(f)

sim = VacuumRobotSimulation(config, use_gui=False)
sim.initialize()

# Telemetria enviada automaticamente
metrics = sim.run_execution()

sim.shutdown()
```

### 4. Visualizar em Node-RED

- Dashboard visível em: http://localhost:1880/ui
- Dados chegando em tempo real
- Atualização a 5Hz para telemetria

## 📈 Gráficos Recomendados

### Série Temporal de Cobertura

```
Nodes:
- HTTP IN
- JSON Parse
- Function: extract msg.payload.coverage_percentage
- ui_chart (Line chart, time axis)
```

### Trajetória 2D

```
Nodes:
- HTTP IN
- JSON Parse
- Function: return {x: msg.payload.pose.x, y: msg.payload.pose.y}
- ui_chart (Scatter, XY)
```

### Velocidades em Tempo Real

```
Nodes:
- HTTP IN
- JSON Parse
- ui_gauge (linear velocity, 0-1 m/s)
- ui_gauge (angular velocity, -5 to 5 rad/s)
```

### Leituras de Sensores

```
Nodes:
- HTTP IN
- JSON Parse
- Function: converter para formato radial
- ui_template com canvas para plot polar
```

## 🎯 Exemplos de Nodes Úteis

### Function: Extrair Cobertura

```javascript
return {
    payload: msg.payload.coverage_percentage
};
```

### Function: Converter para Radial (Sensores)

```javascript
let sensors = msg.payload.sensor_readings;
let data = [];
let angleStep = 2 * Math.PI / Object.keys(sensors).length;

for (let i = 0; i < Object.keys(sensors).length; i++) {
    data.push({
        angle: i * angleStep,
        radius: sensors[`sensor_${i}`]
    });
}

return { payload: data };
```

### Function: Filtrar Apenas Métricas

```javascript
if (msg.payload.execution_id) {
    return msg;  // É métrica
}
return null;    // Ignorar telemetria
```

## 📊 Template HTML para Mapa de Ocupação

```html
<div id="map-container" style="width:100%;height:400px;"></div>

<script>
    // Receber dados
    RED.comms.subscribe("vacuum_map", function(topic, payload) {
        drawOccupancyMap(JSON.parse(payload));
    });
    
    function drawOccupancyMap(data) {
        // data.grid: array 2D de ocupação
        // data.robot_pos: {x, y}
        
        let canvas = document.getElementById('map-canvas');
        let ctx = canvas.getContext('2d');
        
        // Desenhar grid
        for (let i = 0; i < data.grid.length; i++) {
            for (let j = 0; j < data.grid[i].length; j++) {
                let occupancy = data.grid[i][j];
                let color = `rgba(0, 0, 0, ${occupancy})`;
                
                ctx.fillStyle = color;
                ctx.fillRect(i * 10, j * 10, 10, 10);
            }
        }
        
        // Desenhar robô
        ctx.fillStyle = 'red';
        ctx.beginPath();
        ctx.arc(data.robot_pos.x * 10, data.robot_pos.y * 10, 5, 0, 2*Math.PI);
        ctx.fill();
    }
</script>

<canvas id="map-canvas" width="400" height="400"></canvas>
```

## 🔗 Integrar com Banco de Dados

### InfluxDB + Node-RED

```
HTTP IN → Parse JSON → 
  ├─→ InfluxDB Write (timestamp, coverage, energy, etc)
  └─→ Dashboard (histórico)
```

### Node: InfluxDB Write

```
Database: vacuum_sim
Measurement: telemetry
Tags: robot_id, state
Fields: coverage, energy, x, y
```

### Query para Grafana

```sql
SELECT "coverage", "energy" FROM "telemetry"
WHERE time > now() - 1h
GROUP BY time(10s)
```

## 🚨 Troubleshooting

### Dados não chegam

```bash
# Verificar Node-RED
curl -X POST http://localhost:1880/vacuum/data \
  -H "Content-Type: application/json" \
  -d '{"test": "data"}'

# Verificar se Python está enviando
# Adicionar print em node_red_client.py:
print(f"Enviando para {self.url}")
```

### Conexão recusada

```bash
# Node-RED não está rodando
node-red --port 1880

# Verificar porta
lsof -i :1880
```

### Dados desatualizados

```bash
# Aumentar frequência de envio em robot_config.yaml
# Ou reduzir intervalo no script
if self.sim_step % 2 == 0:  # Era % 5
    self.node_red.send_telemetry(telemetry)
```

## 📱 Dashboard Responsivo

Node-RED Dashboard funciona em:
- Desktop: http://localhost:1880/ui
- Mobile: http://<IP>:1880/ui
- Tablet: Suportado

### Otimização Mobile

```yaml
node_red:
  ui_compact: true      # Remover espaçamento
  ui_columns: 1         # Uma coluna
  ui_font_size: 14      # Maior
```

## 🔐 Segurança (Produção)

```javascript
// Validar token antes de aceitar dados
if (msg.req.headers.authorization != "Bearer TOKEN") {
    msg.statusCode = 401;
    return null;
}
```

## 📊 Exportar Dados

### Baixar Histórico

```bash
# Exportar InfluxDB
influx backup ~/backup

# Exportar como CSV
SELECT * INTO OUT FROM telemetry
```

### Gráficos Estáticos

Usar Grafana para gerar PDFs periódicos.

## 🎓 Recursos Adicionais

- [Docs Node-RED](https://nodered.org/docs)
- [UI Dashboard](https://github.com/node-red/node-red-dashboard)
- [Cookbook](https://cookbook.nodered.org)

## ✅ Checklist Setup

- [ ] Node-RED instalado e rodando
- [ ] HTTP IN node criado em `/vacuum/data`
- [ ] Python enviando dados (verificar logs)
- [ ] Dashboard visível em localhost:1880/ui
- [ ] Gráficos atualizando em tempo real
- [ ] Cobertura % aumentando
- [ ] Trajetória XY apareça no chart
- [ ] Sensores mostrando valores realistas
- [ ] Exportar dados funcionando

---

**Pronto!** Seu dashboard está funcionando. 📊✨
