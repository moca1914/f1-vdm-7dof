import numpy as np
from vehicle_parameters import WilliamsF1Car

class RideKinematics:
    """
    Builds the 7-DOF system equations of motion: M*X_ddot + C*X_dot + K*X = F
    Degrees of freedom: [z_s, theta, phi, z_ufL, z_ufR, z_urL, z_urR]
    """
    def __init__(self, car: WilliamsF1Car):
        self.c = car.chassis
        self.s = car.suspension
        
        # 1. Mass Matrix (7x7)
        self.M = np.diag([
            self.c.m_s, self.c.I_yy, self.c.I_xx, 
            self.s.m_uf, self.s.m_uf, self.s.m_ur, self.s.m_ur
        ])
        
        self.K = np.zeros((7, 7))
        self.C = np.zeros((7, 7))
        self._build_matrices()

    def _build_matrices(self):
        a, b, tf, tr = self.c.a, self.c.b, self.c.t_f, self.c.t_r
        ksf, ksr, csf, csr = self.s.k_sf, self.s.k_sr, self.s.c_sf, self.s.c_sr
        ktf, ktr = self.s.k_tf, self.s.k_tr

        # --- HEAVE (Row 0) ---
        self.K[0, 0] = 2 * (ksf + ksr)
        self.K[0, 1] = 2 * (ksr * b - ksf * a)
        self.K[0, 3] = self.K[0, 4] = -ksf
        self.K[0, 5] = self.K[0, 6] = -ksr

        self.C[0, 0] = 2 * (csf + csr)
        self.C[0, 1] = 2 * (csr * b - csf * a)
        self.C[0, 3] = self.C[0, 4] = -csf
        self.C[0, 5] = self.C[0, 6] = -csr

        # --- PITCH (Row 1) ---
        self.K[1, 0] = 2 * (ksr * b - ksf * a)
        self.K[1, 1] = 2 * (ksf * a**2 + ksr * b**2)
        self.K[1, 3] = self.K[1, 4] = ksf * a
        self.K[1, 5] = self.K[1, 6] = -ksr * b

        self.C[1, 0] = 2 * (csr * b - csf * a)
        self.C[1, 1] = 2 * (csf * a**2 + csr * b**2)
        self.C[1, 3] = self.C[1, 4] = csf * a
        self.C[1, 5] = self.C[1, 6] = -csr * b

        # --- ROLL (Row 2) ---
        self.K[2, 2] = 2 * (ksf * tf**2 + ksr * tr**2)
        self.K[2, 3] = -ksf * tf
        self.K[2, 4] = ksf * tf
        self.K[2, 5] = -ksr * tr
        self.K[2, 6] = ksr * tr

        self.C[2, 2] = 2 * (csf * tf**2 + csr * tr**2)
        self.C[2, 3] = -csf * tf
        self.C[2, 4] = csf * tf
        self.C[2, 5] = -csr * tr
        self.C[2, 6] = csr * tr

        # --- UNSPRUNG MASSES (Rows 3 to 6) ---
        # Front Left Wheel
        self.K[3, 0], self.K[3, 1], self.K[3, 2], self.K[3, 3] = -ksf, ksf * a, -ksf * tf, ksf + ktf
        self.C[3, 0], self.C[3, 1], self.C[3, 2], self.C[3, 3] = -csf, csf * a, -csf * tf, csf
        # Front Right Wheel
        self.K[4, 0], self.K[4, 1], self.K[4, 2], self.K[4, 4] = -ksf, ksf * a, ksf * tf, ksf + ktf
        self.C[4, 0], self.C[4, 1], self.C[4, 2], self.C[4, 4] = -csf, csf * a, csf * tf, csf
        # Rear Left Wheel
        self.K[5, 0], self.K[5, 1], self.K[5, 2], self.K[5, 5] = -ksr, -ksr * b, -ksr * tr, ksr + ktr
        self.C[5, 0], self.C[5, 1], self.C[5, 2], self.C[5, 5] = -csr, -csr * b, -csr * tr, csr
        # Rear Right Wheel
        self.K[6, 0], self.K[6, 1], self.K[6, 2], self.K[6, 6] = -ksr, -ksr * b, ksr * tr, ksr + ktr
        self.C[6, 0], self.C[6, 1], self.C[6, 2], self.C[6, 6] = -csr, -csr * b, csr * tr, csr

    def get_state_space(self):
        """Converts to standard form: X_dot = A*X + B*U"""
        M_inv = np.linalg.inv(self.M)
        A = np.zeros((14, 14))
        A[0:7, 7:14] = np.eye(7)
        A[7:14, 0:7] = -M_inv @ self.K
        A[7:14, 7:14] = -M_inv @ self.C
        return A, M_inv  
