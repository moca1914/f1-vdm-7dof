# F1-VDM-7DOF: Multi-Body Vehicle Dynamics & Ride Simulator

This repository contains a production-grade, object-oriented structural vehicle dynamics modelling (VDM) framework designed to simulate the heave, pitch, and roll response of a Formula 1 car traversing transient track hazards, such as aggressive kerb strikes. 

Instead of treating the vehicle as a simplified, isolated point-mass, this system uses a full multi-body state-space conversion to track the coupled transient interactions between the sprung mass (chassis) and the four individual unsprung masses (wheel stations).

## Mathematical Framework

The simulation solves the generalised equations of motion for multi-body mechanical systems:

$$M\ddot{X} + C\dot{X} + KX = F(t)$$

Where the degree-of-freedom spatial vector $X$ is explicitly defined as:

$$X = \begin{bmatrix} z_s & \theta & \phi & z_{ufL} & z_{ufR} & z_{urL} & z_{urR} \end{bmatrix}^T$$

* $z_s$: Chassis Heave (Vertical displacement at the Centre of Gravity)
* $\theta$: Chassis Pitch Angle (Rotation about the lateral axis)
* $\phi$: Chassis Roll Angle (Rotation about the longitudinal axis)
* $z_{u}$: Vertical displacements of the four individual unsprung wheel hubs

### State-Space Conversion
To resolve the secondary differential equations using standard numerical ODE integration frameworks, the system equations are reduced to a first-order continuous state-space system:

$$\dot{Z} = AZ + BU$$

The 14x14 continuous state matrix $A$ is constructed via automated matrix partitioning using the inverted structural mass matrix $M^{-1}$:

$$A = \begin{bmatrix} [0]_{7\times7} & [I]_{7\times7} \\ -M^{-1}K & -M^{-1}C \end{bmatrix}$$

This coupling maps exactly how forces injected into a single wheel station distribute across the entire vehicle kinematics web.

## Architecture Directory

* `vehicle_parameters.py`: Outlines the object-oriented data structures (`dataclasses`) storing mass distributions, moments of inertia tensors, and high-stiffness spring/damper parameters natively matching real UK F1 vehicle regulations.
* `kinematics_7dof.py`: Constructs the symmetric 7x7 stiffness, damping, and inertia matrices, handling complex multi-point cross-coupling links before translating the system into continuous state-space forms.
* `solver_7dof.py`: Sets up the temporal track simulation workspace, applying a high-frequency localised vertical load profile (kerb strike) and executing the explicit adaptive fourth-order Runge-Kutta (RK45) numerical integration loop.

## Dependencies & Execution

The engine utilises standard scientific Python libraries. To clone the workspace and execute the solver pipeline natively:

```bash
pip install numpy scipy
python solver_7dof.py
