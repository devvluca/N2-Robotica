# Simulação de Braço Robótico Planar 2/3 DOF com Controle PID

Este projeto simula um braço robótico articulado planar com 2 ou 3 graus de liberdade (DOF) no PyBullet. Ele demonstra controle por PID em cada junta, detecção de alvos simples, ajuste de trajetória para evitar colisões, e simulação simples de pegada e transporte com perturbações de massa.

## Funcionalidades

- **Cinemática Direta e Inversa**: Cálculo de posições e resolução de ângulos para movimento planar.
- **Controle PID**: Controle em malha fechada individual por junta para atingir referências angulares.
- **Detecção de Obstáculos**: Ajuste automático de trajetória para evitar colisões.
- **Simulação de Grasping**: Mudança de dinâmica do objeto para simular pegada e transporte.
- **Perturbações**: Aplicação de mudanças de massa para testar estabilidade.
- **Logs de Desempenho**: Métricas como erro médio, tempo de estabilização e overshoot máximo.
- **Supervisão Node-RED**: Dashboard em tempo real com métricas e visualização.

## Como Usar

### 1. Instale as dependências Python:
```bash
pip install pybullet requests
```

### 2. Execute a simulação (modo básico):
```bash
python main.py --cycles 6
```

### 3. Com supervisão Node-RED (opcional):

#### Pré-requisitos Node-RED:
1. Instale o Node.js: https://nodejs.org/
2. Instale o Node-RED:
   ```bash
   npm install -g node-red
   ```
3. Instale o dashboard:
   ```bash
   cd ~/.node-red
   npm install node-red-dashboard
   ```

#### Configuração:
1. Inicie o Node-RED:
   ```bash
   node-red
   ```
2. Acesse http://localhost:1880
3. Importe o flow: Menu → Import → selecione `node_red_flow.json`
4. Clique em "Deploy"
5. Acesse o dashboard: http://localhost:1880/ui

#### Execute a simulação:
```bash
python main.py --cycles 6
```

O dashboard exibirá em tempo real:
- 📊 **Erro médio de posição** (graus)
- 📈 **Overshoot máximo** (graus)
- ⚡ **Energia total consumida** (Joules)
- 🎯 **Ângulos das juntas** (1, 2, 3)
- 📍 **Posição do efetuador**
- ✅ **Status do ciclo**

## Argumentos da Linha de Comando

```bash
python main.py [--dof 2|3] [--cycles N] [--nogui]
```

- `--dof`: Número de graus de liberdade (2 ou 3, padrão: 3)
- `--cycles`: Número de ciclos de pegar/soltar (padrão: 6)
- `--nogui`: Executar sem interface gráfica do PyBullet

## Arquivos

### Código Principal
- `main.py`: Loop principal da simulação
- `src/kinematics.py`: Cinemática direta e inversa
- `src/arm.py`: Classe do braço com controle PID
- `src/control.py`: Controlador de movimento e grasping
- `src/simulation.py`: Configuração do ambiente PyBullet

### Modelos
- `models/planar_arm_3dof.urdf`: Modelo URDF do braço 3-DOF
- `models/planar_arm_2dof.urdf`: Modelo URDF do braço 2-DOF

### Node-RED
- `node_red_client.py`: Cliente HTTP para enviar métricas
- `node_red_flow.json`: Flow do Node-RED com dashboard

## Métricas Supervisionadas

O sistema envia as seguintes métricas para o Node-RED:

| Métrica | Descrição | Unidade |
|---------|-----------|---------|
| Erro médio | Diferença média entre ângulo alvo e atual | rad / graus |
| Overshoot máximo | Maior erro registrado durante movimento | rad / graus |
| Energia total | Trabalho realizado pelos motores | Joules |
| Tempo de estabilização | Tempo até erro < limiar | segundos |

## Requisitos do Trabalho Atendidos

✅ Manipulador planar 2/3 DOF  
✅ Controle PID por junta  
✅ Detecção automática de alvo  
✅ Desvio de obstáculos  
✅ Pegar e carregar objetos  
✅ Reação a perturbações  
✅ Cinemática direta/inversa  
✅ Sensores simulados (encoder, torque)  
✅ Métricas de log enviadas ao Node-RED  

## Inspiração

Baseado no enunciado: braço planar 2/3 DOF com PID, detecção de alvos, ajuste de trajetória, grasping e reação a perturbações.</content>
<parameter name="filePath">c:\Users\lucan\Documents\GitHub\ur5_grasp_object_pybullet\README.md


