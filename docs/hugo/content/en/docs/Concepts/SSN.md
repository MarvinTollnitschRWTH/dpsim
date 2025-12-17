---
title: "State Space Nodal Modeling"
linkTitle: "State Space Nodal Modeling"
date: 2025-12-16
---

The State Space Nodal approach describes electrical components or (sub)systems
using a suitable state space representation and from that develops an output equation
which can be included into a total system's MNA representation for simulation. Doing so,
more complex systems are described using their total input-output relation and internal states.
This can often reduce the amount of nodes in the circuit which in turn reduces the
size of the system matrix dimensions. Also, this allows components or systems
which may be described using a state space representation to be implemented
and simulated in a systematic way using the state space matrices as parameters.

The implementation of the State Space Nodal concept is based on the methods and
deductions in [Dufour2010](https://ieeexplore.ieee.org/document/5667072) which will be briefly
outlined below.

# Development of the output equation

A circuit, potentially consisting of multiple connected elements, is given
in state space representation:

```math
\begin{align}
\dot{x} &= \textbf{A} \cdot x + \textbf{B} \cdot u \\
y &= \textbf{C} \cdot x + \textbf{D} \cdot u
\end{align}
```

As the next step, the states are discretized. In dpsim, the trapezoidal integration
rule is used. This leads to the following representation with altered, hatted matrices:

```math
x_{t+\Delta t} = \mathbf{\hat{A}} \cdot x_{t} + \mathbf{\hat{B}} \cdot u_{t} + \mathbf{\hat{B}} \cdot u_{t+\Delta t}
```

Inserting this into the output equation and combining terms yields:

```math
y_{t+\Delta t} = y_{hist} + \mathbf{W} \cdot u_{t+\Delta t}
```

Now, $y_{hist}$ is composed of all known terms from the current and previous time step,
$u_{t+\Delta t}$ are the current unknown system quantities and $\mathbf{W}$ can be
interpreted as this circuits system matrix. Depending on what $y_{t+\Delta t}$ describes,
the represented circuit is classified as:

* a V-type group: $y_{t+\Delta t}$ describes output currents
* an I-type group: $y_{t+\Delta t}$ describes output voltages
* a mixed-type group: $y_{t+\Delta t}$ describes both output currents and voltages

In the Modified Nodal Analysis system representation $GV = I$, system rows correspond
to nodal or mesh equations of the system. For V-type outputs, the current described by the
output equation can be integrated into the system matrix as part of one existing row's nodal
equation. For I-type outputs, a mesh equation can be added as a new row in the system
matrix similar to the procedure used when adding a voltage source. For both types this means
stamping the entries of $\mathbf{W}$ into the system matrix $\mathbf{G}$ while adding
the entries of $-y_{hist}$ to the excitation vector I.

# Implemented components in dpsim

| Component Name                                                                                                    | State Space Representation                                                                                                                                                                                                                                                                                                              |
| ---                                                                                                               | ---                                                                                                                                                                                                                                                                                                                                     |
| [EMT_Ph3_SSN_Inductor](../../../../dpsim-models/include/dpsim-models/EMT/EMT_Ph3_SSN_Inductor.h)                  | $$\dot{\mathbf{x}}(t)=\begin{bmatrix}\dot{I}_a(t)\\\dot{I}_b(t)\\\dot{I}_c(t)\end{bmatrix}=\begin{bmatrix}\frac{1}{L}&0&0\\ 0&\frac{1}{L}&0\\ 0&0&\frac{1}{L}\end{bmatrix}\begin{bmatrix}V_a(t)\\ V_b(t)\\ V_c(t)\end{bmatrix},\quad \mathbf{y}(t)=\mathbf{x}(t)=\begin{bmatrix}I_a(t)\\ I_b(t)\\ I_c(t)\end{bmatrix}$$                 |
| [EMT_Ph3_SSN_Capacitor](../../../../dpsim-models/include/dpsim-models/EMT/EMT_Ph3_SSN_Capacitor.h)                | $$\dot{\mathbf{x}}(t)=\begin{bmatrix}\dot{V}_a(t)\\\dot{V}_b(t)\\\dot{V}_c(t)\end{bmatrix}=\begin{bmatrix}\frac{1}{C}&0&0\\ 0&\frac{1}{C}&0\\ 0&0&\frac{1}{C}\end{bmatrix}\begin{bmatrix}I_a(t)\\ I_b(t)\\ I_c(t)\end{bmatrix},\quad \mathbf{y}(t)=\mathbf{x}(t)=\begin{bmatrix}V_a(t)\\ V_b(t)\\ V_c(t)\end{bmatrix}$$                 |
| [EMT_Ph1_SSN_Full_Serial_RLC](../../../../dpsim-models/include/dpsim-models/EMT/EMT_Ph1_SSN_Full_Serial_RLC.h)    | $$\dot{\mathbf{x}}=\begin{bmatrix}\dot{V}_C\\\dot{I}\end{bmatrix}=\begin{bmatrix}0&\frac{1}{C}\\-\frac{1}{L}&-\frac{R}{L}\end{bmatrix}\begin{bmatrix}V_C\\ I\end{bmatrix}+V_{RLC}\begin{bmatrix}0\\\frac{1}{L}\end{bmatrix},\quad y=I=\begin{bmatrix}0 & 1\end{bmatrix}\mathbf{x}$$                                                     |
| [EMT_Ph3_SSN_Full_Serial_RLC](../../../../dpsim-models/include/dpsim-models/EMT/EMT_Ph3_SSN_Full_Serial_RLC.h)    | $$\dot{\mathbf{x}}_{a,b,c}=\begin{bmatrix}\dot{V}_{C_{a,b,c}}\\\dot{I}_{a,b,c}\end{bmatrix}=\begin{bmatrix}0&\frac{1}{C}\\-\frac{1}{L}&-\frac{R}{L}\end{bmatrix}\begin{bmatrix}V_{C_{a,b,c}}\\ I_{a,b,c}\end{bmatrix}+V_{RLC}\begin{bmatrix}0\\\frac{1}{L}\end{bmatrix},\quad y=I_{a,b,c}=\begin{bmatrix}0 & 1\end{bmatrix}\mathbf{x}$$ |
| [EMT_Ph1_SSNTypeV_2T](../../../../dpsim-models/include/dpsim-models/EMT/EMT_Ph1_SSNTypeV_2T.h)                    | $$\dot{x} = \textbf{A} \cdot x + \textbf{B} \cdot u \quad y = \textbf{C} \cdot x + \textbf{D} \cdot u$$                                                                                                                                                                                                                                 |
| [EMT_Ph1_SSN_TypeI_2T](../../../../dpsim-models/include/dpsim-models/EMT/EMT_Ph1_SSNTypeI_2T.h)                   | $$\dot{x} = \textbf{A} \cdot x + \textbf{B} \cdot u \quad y = \textbf{C} \cdot x + \textbf{D} \cdot u$$                                                                                                                                                                                                                                 |
