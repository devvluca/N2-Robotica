# Guia Rápido - Robô Aspirador

Comece em 5 minutos!

## ⚡ 5 Minutos para Começar

### Passo 1: Instalar (1 min)

```bash
cd robo_aspirador
pip install -r requirements.txt
```

### Passo 2: Validar (30 seg)

```bash
python validate.py
```

Deve mostrar:
```
✓ Python 3.8+
✓ Project Structure
✓ Configuration File
✓ Dependencies
```

### Passo 3: Rodar Exemplo (3.5 min)

```bash
python examples.py
```

Escolha opção `1` (Limpeza Básica com GUI)

Verá:
- Robô visualizado em 3D com PyBullet
- Movimento em ambiente com obstáculos
- Gráfico de sensores em tempo real
- Estatísticas de cobertura

## 📊 Entender os Resultados

Depois de rodar uma execução, observe:

```
✓ Execução concluída!
  Cobertura: 87.3%
  Energia: 18.45J
  Tempo: 45.2s
```

### O que significa?

- **Cobertura 87.3%**: Robô visitou 87% da área
- **Energia 18.45J**: Gasto energético total
- **Tempo 45.2s**: Duração da limpeza

### Melhorar cobertura?

1. Aumentar `max_duration` em exemplos.py
2. Usar estratégia 'systematic' em vez de 'spiral'
3. Reduzir `obstacle_threshold` (mais ousado)

## 🎮 Controles da GUI

No PyBullet GUI:

| Ação | Tecla |
|------|-------|
| Sair | ESC |
| Pausar | P |
| Zoom | Scroll |
| Rotacionar | Click + Arrastar |
| Panar | Ctrl + Click + Arrastar |

## 📁 Onde estão os arquivos?

- **Logs**: `logs/execution_*.csv` (trajetória completa)
- **Mapas**: `maps/occupancy_grid.npy` (grade de ocupação)
- **Rotas**: `maps/learned_routes.json` (rotas otimizadas)

### Ver trajetória em Python

```python
import csv
with open('logs/execution_20240101_120000.csv') as f:
    reader = csv.DictReader(f)
    for row in reader:
        x = float(row['x'])
        y = float(row['y'])
        cov = float(row['coverage'])
        print(f"({x:.2f}, {y:.2f}) -> {cov:.1f}% coberto")
```

## 🚀 Próximos Passos

### Executar Outras Exemplos

No menu `python examples.py`:

1. **Limpeza Básica** - Visualizar robô em ação
2. **Headless** - Simulação rápida sem GUI
3. **Aprendizado** - Ver otimização entre ciclos
4. **Estratégias** - Comparar spiral vs systematic vs random
5. **Node-RED** - Enviar dados em tempo real
6. **Performance** - Análise detalhada de métricas

### Modificar Configuração

Editar `config/robot_config.yaml`:

**Para ambiente maior:**
```yaml
environment:
  world_width: 15.0    # Era 10.0
  world_height: 12.0   # Era 8.0
```

**Para robô mais rápido:**
```yaml
robot:
  max_linear_velocity: 1.0     # Era 0.5
  max_angular_velocity: 4.0    # Era 2.0
```

**Para mais sensores:**
```yaml
robot:
  sensors:
    count: 8            # Era 5
```

**Para mapeamento mais preciso:**
```yaml
mapping:
  grid_resolution: 0.05   # Era 0.1 (menor = mais preciso)
```

## 🔧 Troubleshooting Rápido

### GUI não abre

```bash
# Usar modo headless
# No examples.py, alterar:
# use_gui=True → use_gui=False
```

### Erro "pybullet not found"

```bash
pip install pybullet==3.2.5
```

### Simulação muito lenta

```bash
# Em robot_config.yaml, aumentar time_step:
simulation:
  time_step: 0.02   # Era 0.01
```

### Robô não se move

Verificar sensores:
```bash
# Adicionar ao script:
print(sim.sensors)  # Deve listar 5 sensores
```

## 📈 Interpretar Estratégias

### Espiral (Spiral)
- Começa no centro, expande em raio crescente
- ✓ Boa cobertura geral
- ✗ Pode revisitar muito
- Tempo: 40-50s

### Sistemática (Systematic)
- Varre de esquerda para direita, linha por linha
- ✓ Mínima revisita
- ✓ Previsível
- Tempo: 45-60s

### Aleatória (Random)
- Caminhada aleatória com evitação
- ✓ Rápida
- ✗ Cobertura baixa
- Tempo: 30-40s

## 🎯 Métricas Esperadas

Com 60 segundos de execução:

| Estratégia | Cobertura | Energia | Eficiência |
|-----------|-----------|---------|-----------|
| Spiral | 85% | 20J | 4.2 |
| Systematic | 88% | 22J | 4.0 |
| Random | 65% | 18J | 3.6 |

*Valores podem variar com hardware*

## 💡 Dicas e Truques

### Executar Múltiplas Vezes
```bash
for i in {1..5}; do python examples.py < <(echo 2); done
```

### Gerar Gráficos dos Logs
```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('logs/execution_20240101_120000.csv')
plt.plot(df['x'], df['y'], label='Trajetória')
plt.plot(df['coverage'] * 100, label='Cobertura %')
plt.legend()
plt.show()
```

### Verificar Aprendizado
```python
import json
with open('maps/learned_routes.json') as f:
    learning = json.load(f)
    print(f"Execuções: {len(learning)}")
    print(f"Melhor: {learning['best_strategy']}")
```

## 🔄 Workflow Recomendado

1. **Dia 1**: Rodar `examples.py` para entender projeto
2. **Dia 2**: Modificar `robot_config.yaml` para experimentar
3. **Dia 3**: Criar scripts customizados usando `VacuumRobotSimulation`
4. **Dia 4**: Integrar com Node-RED para visualização
5. **Dia 5**: Adicionar novos sensores/estratégias

## 📞 Ajuda Rápida

| Problema | Solução |
|----------|---------|
| Não entendo resultado | Ver `README.md` - seção "Análise de Resultados" |
| Quer modificar robô | Editar `config/robot_config.yaml` |
| Quer código customizado | Ver exemplos em `examples.py` |
| Node-RED não funciona | Rodar `node-red --port 1880` em outro terminal |
| Performance ruim | Desabilitar GUI ou aumentar `time_step` |

## ✅ Checklist de Sucesso

- [ ] Python 3.8+ instalado
- [ ] `python validate.py` passou
- [ ] `python examples.py` funciona
- [ ] Viu robô em GUI em 3D
- [ ] Cobertura > 80% em spiral
- [ ] CSV logs criados em `logs/`
- [ ] Entende configuração YAML
- [ ] Conseguiu modificar velocidade do robô
- [ ] Comparou 2+ estratégias
- [ ] Viu aprendizado de 3+ ciclos

## 🎓 Próxima Leitura

- Módulos individuais em `src/`
- Integração Node-RED em `NODE_RED_SETUP.md`
- Conceitos avançados em `README.md`
- Código completo documentado inline

---

**Pronto para começar!** 🚀

Rode: `python examples.py`
