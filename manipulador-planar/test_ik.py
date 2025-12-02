import math
from src.kinematics import PlanarArmKinematics

kin = PlanarArmKinematics([0.5, 0.5, 0.4])

# Testar algumas posições problemáticas
test_positions = [
    (1.18, -0.09),  # Ciclo 5 - DENTRO
    (1.04, -0.15),  # Ciclo 1 - DENTRO
    (1.02, -0.09),  # Ciclo 3 - DENTRO
]

for pos in test_positions:
    angles = kin.inverse_kinematics(pos[0], pos[1])
    if angles:
        fk = kin.forward_kinematics(angles)
        error = math.sqrt((fk[0]-pos[0])**2 + (fk[1]-pos[1])**2)
        print(f"Target: ({pos[0]:.2f}, {pos[1]:.2f})")
        print(f"  Angles: [{angles[0]:.3f}, {angles[1]:.3f}, {angles[2]:.3f}]")
        print(f"  FK result: ({fk[0]:.3f}, {fk[1]:.3f})")
        print(f"  Error: {error:.4f}m")
        print()
    else:
        print(f"Target: ({pos[0]:.2f}, {pos[1]:.2f}) - IK failed (out of reach)")
