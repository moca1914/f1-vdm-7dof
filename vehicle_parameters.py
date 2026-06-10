from dataclasses import dataclass

@dataclass
class ChassisParams:
    """Sprung mass (chassis) parameters based on generic F1 regulations."""
    m_s: float = 680.0       # Sprung mass (kg)
    I_xx: float = 150.0      # Roll moment of inertia (kg*m^2)
    I_yy: float = 550.0      # Pitch moment of inertia (kg*m^2)
    a: float = 1.6           # Distance from CG to front axle (m)
    b: float = 1.8           # Distance from CG to rear axle (m)
    t_f: float = 0.85        # Front half-track width (m)
    t_r: float = 0.80        # Rear half-track width (m)

@dataclass
class SuspensionParams:
    """Unsprung mass, spring, and damping rates."""
    m_uf: float = 30.0       # Front unsprung mass per wheel (kg)
    m_ur: float = 32.0       # Rear unsprung mass per wheel (kg)
    k_sf: float = 65000.0    # Front suspension stiffness (N/m)
    k_sr: float = 80000.0    # Rear suspension stiffness (N/m)
    c_sf: float = 4500.0     # Front suspension damping (N*s/m)
    c_sr: float = 5500.0     # Rear suspension damping (N*s/m)
    k_tf: float = 220000.0   # Front tire vertical stiffness (N/m)
    k_tr: float = 220000.0   # Rear tire vertical stiffness (N/m)

class WilliamsF1Car:
    def __init__(self):
        self.chassis = ChassisParams()
        self.suspension = SuspensionParams()
