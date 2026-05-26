import matplotlib.pyplot as plt
import numpy as np


def simulate_plant(num_discrete, den_discrete, regulator, reference, t):
    """Discretise plant and run closed-loop simulation."""
    out = np.zeros_like(t)
    control = np.zeros_like(t)

    for i in range(2, len(t)):
        out[i] = - den_discrete[1] * out[i-1] - den_discrete[2] * out[i-2] + \
            num_discrete[1] * control[i-1] + num_discrete[2] * control[i-2]
        control[i] = regulator.regulate(out[i], reference[i])
    return out


def plot_responce(t, t_open, ref, open_loop, close_loop):
    """Plot open-loop and closed-loop step responses side-by-side."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))

    ax1.plot(t_open, ref, 'k--', label="reference")
    ax1.plot(t_open, open_loop, label="output")
    ax1.set_title("Open-Loop Step Response")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("y(t)")
    ax1.grid(True)

    ax2.plot(t, ref, 'k--', label="reference")
    ax2.plot(t, close_loop, label="output")
    ax2.set_title("Closed-Loop Response")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("y(t)")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()
