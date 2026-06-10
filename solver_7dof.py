import numpy as np
from scipy.integrate import solve_ivp
from vehicle_parameters import WilliamsF1Car
from kinematics_7dof import RideKinematics

class DynamicSimulation:
    def __init__(self, duration: float = 2.0, dt: float = 0.001):
        self.car = WilliamsF1Car()
        self.kinematics = RideKinematics(self.car)
        self.A, self.M_inv = self.kinematics.get_state_space()
        self.t_eval = np.arange(0.0, duration, dt)
        
    def road_profile_input(self, t: float) -> np.ndarray:
        """Simulates hitting a 40mm kerb with the Front-Left wheel at t=0.1s."""
        U = np.zeros(7)
        if 0.1 <= t <= 0.15:
            kerb_height = 0.04
            U[3] = self.car.suspension.k_tf * kerb_height
        return U
        
    def equations_of_motion(self, t, y):
        U_force = self.road_profile_input(t)
        B_U = np.zeros(14)
        B_U[7:14] = self.M_inv @ U_force
        return self.A @ y + B_U

    def run(self):
        print("Executing 7-DOF RK45 integration loop...")
        initial_state = np.zeros(14)
        solution = solve_ivp(
            fun=self.equations_of_motion,
            t_span=(self.t_eval[0], self.t_eval[-1]),
            y0=initial_state,
            t_eval=self.t_eval,
            method='RK45'
        )
        print("Simulation complete!")
        print(f"Max Heave Displacement: {np.max(np.abs(solution.y[0, :]))*1000:.2f} mm")
        print(f"Max Pitch Oscillation: {np.max(np.abs(solution.y[1, :]))*180/np.pi:.3f} deg")

if __name__ == "__main__":
    sim = DynamicSimulation()
    sim.run()
