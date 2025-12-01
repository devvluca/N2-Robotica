# Simulação de Braço Robótico Planar 2/3 DOF com Controle PID

Este projeto simula um braço robótico articulado planar com 2 ou 3 graus de liberdade (DOF) no PyBullet. Ele demonstra controle por PID em cada junta, detecção de alvos simples, ajuste de trajetória para evitar colisões, e simulação simples de pegada e transporte com perturbações de massa.

## Funcionalidades

- **Cinemática Direta e Inversa**: Cálculo de posições e resolução de ângulos para movimento planar.
- **Controle PID**: Controle em malha fechada individual por junta para atingir referências angulares.
- **Detecção de Obstáculos**: Ajuste automático de trajetória para evitar colisões.
- **Simulação de Grasping**: Mudança de dinâmica do objeto para simular pegada e transporte.
- **Perturbações**: Aplicação de mudanças de massa para testar estabilidade.
- **Logs de Desempenho**: Métricas como erro médio, tempo de estabilização e overshoot máximo.

## Como Usar

1. Instale as dependências:
   ```bash
   pip install pybullet
   ```

2. Execute a simulação:
   ```bash
   python main.py
   ```

3. Observe no GUI do PyBullet: O braço pega cubos aleatórios, evita obstáculos, aplica perturbações e os coloca na bandeja.

## Arquivos

- `kinematics.py`: Cinemática direta e inversa nominal (2DOF analítico + 3DOF aproximado).
- `arm.py`: Classe do braço com controle PID.
- `simulation.py`: Configuração do ambiente e utilitários.
- `control.py`: Controlador de movimento, grasping e logs.
- `main.py`: Loop principal da simulação.
- `planar_arm_3dof.urdf`: Modelo URDF do braço planar.

## Inspiração

Baseado no enunciado: braço planar 2/3 DOF com PID, detecção de alvos, ajuste de trajetória, grasping e reação a perturbações.</content>
<parameter name="filePath">c:\Users\lucan\Documents\GitHub\ur5_grasp_object_pybullet\README.md


