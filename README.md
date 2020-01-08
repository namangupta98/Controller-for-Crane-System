# Controller-for-Crane-System

A crane that moves along an one-dimensional track behaves as a frictionless cart with mass M actuated by an external force F that constitutes the input of the system. There are two loads suspended from cables attached to the crane. The loads have mass m1 and m2, and the lengths of the cables are l1 and l2, respectively.

Please refer to the report for figure.

The objective is to design a best controller and observer for the system which can control the loads suspended from the cables of the crane. The project flows in steps, starting from deriving the equation of motion of the system. Then, linearizing the system at equilibrium points, designing a LQR controller in SIMULINK, designing a Luenberger Observer for both linear an well as non-linear system. The calculations on non-linear system is computed using ode solver Ode45. Finally, designing a LQG controller using Kalman  Filter and LQR Controller.

In conclusion, the working of LQR and LQG controller is understood and implemented using MATLAB and SIMULINK.
