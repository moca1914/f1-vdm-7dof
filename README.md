```markdown
# 7-Degree-of-Freedom (7-DOF) Full Vehicle F1 Ride Model

This repository contains a production-grade, object-oriented structural vehicle dynamics simulator designed to model the heave, pitch, and roll response of a Formula 1 car traversing transient track hazards (such as kerb strikes). 

Instead of treating the vehicle as an isolated point-mass, this system uses a full multi-body state-space conversion to track interactions between the sprung mass (chassis) and four individual unsprung masses (wheels).

## Mathematical Framework

The system solves the generalized equations of motion for multi-body mechanical systems:

$$M\ddot{X} + C\dot{X} + KX = F(t)$$

Where the degree-of-freedom state vector $X$ is explicitly defined as:

$$X = \begin{bmatrix} z_s & \theta & \phi & z_{ufL} & z_{ufR} & z_{urL} & z_{urR} \end{bmatrix}^T$$

*   $z_s$: Chassis Heave (Vertical displacement)
*   $\theta$: Chassis Pitch Angle
*   $\phi$: Chassis Roll Angle
*   $z_{u}$: Vertical displacement of the 4 individual unsprung wheel stations

### State-Space Conversion
To compute the step-by-step transient responses using standard numerical ODE integration frameworks, the secondary differential equations are reduced to a first-order system:

$$\dot{Z} = AZ + BU$$

The 14x14 continuous state matrix $A$ is constructed via matrix partition using the inverted mass matrix $M^{-1}$:

$$A = \begin{bmatrix} [0]_{7\times7} & [I]_{7\times7} \\ -M^{-1}K & -M^{-1}C \end{bmatrix}$$

## Architecture Directory

*   `vehicle_parameters.py`: Outlines the physical data architecture (`dataclasses`) storing mass distributions, rotational inertia tensor components, and high-stiffness spring/damper parameters natively matching F1 ride profiles.
*   `kinematics_7dof.py`: Constructs the symmetric 7x7 stiffness, damping, and inertia matrices, handling cross-coupling links before translating the system into continuous state-space forms.
*   `solver_7dof.py`: Handles transient time stepping using an explicit Runge-Kutta (RK45) adaptive integration loop passing a localized road profile input.

## Dependencies & Execution

The engine utilizes standard scientific Python dependencies. To execute the solver pipeline natively:

```bash
pip install numpy scipy
python solver_7dof.py
