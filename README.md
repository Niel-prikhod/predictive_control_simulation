# Predictive Control

Learning implementation of classic and predictive control algorithms.

## Program Flow

1. `argument_parser()` selects controller via CLI (`pid` / `pole` / `mpc`)
2. `main()` constructs a continuous plant $H(s) = \frac{1.5}{5s^2 + 5s + 1}$ and instantiates the regulator
3. `simulate_plant()` discretises the plant (ZOH), iterates the difference equation, and calls `regulator.regulate()` at each step
4. `plot_responce()` shows open-loop and closed-loop step responses side-by-side

## Implemented Controllers

### PID (`pid.py`)

Velocity (incremental) form of a discrete PID. Control law:

$$
\begin{aligned}
q_0 &= -r_0 \left(1 + \frac{T}{2T_i} + \frac{T_d}{T}\right) \\[2pt]
q_1 &= r_0 \left(1 - \frac{T}{2T_i} + 2\frac{T_d}{T}\right) \\[2pt]
q_2 &= -r_0 \frac{T_d}{T} \\[4pt]
u_k &= u_{k-1} - (q_0+q_1+q_2)\, r_k + q_0\, y_k + q_1\, y_{k-1} + q_2\, y_{k-2}
\end{aligned}
$$

### Pole-Placement (RST) (`ppc.py`)

Solves the Diophantine equation to place closed-loop poles arbitrarily:

$$
A(z) \cdot P(z) + B(z) \cdot Q(z) = D(z)
$$

- $A(z)$, $B(z)$ — plant denominator and numerator
- $D(z)$ — desired closed-loop polynomial (mapped from continuous poles $s_i \to e^{s_i T}$)
- $P(z)$, $Q(z)$ — unknown controller polynomials solved via $\text{lstsq}$

Control law (RST structure):

$$
u_k = R \cdot r_k - \sum_{i=0}^{n-1} q_i \cdot y_{k-i} - \sum_{i=1}^{n-1} p_i \cdot u_{k-i}
$$

where $R = D(1) / B(1)$ ensures zero steady-state error.

## Planned

- **GPC** — Generalized Predictive Control (unconstrained).
- **GPC with constraints** — GPC with input/output constraints.

## Usage

```bash
make run-pid
make run-pole
```

Or manually:

```bash
python main.py pid
python main.py pole
```

The tested plant is a second-order system $1.5 / (5s^2 + 5s + 1)$. Each command plots the open-loop and closed-loop step response.

## Dependencies

- `numpy`, `scipy`, `matplotlib`
