import matplotlib.pyplot as plt
import numpy as np
from scipy import signal


def simulate_plant(plant_cont, regulator, reference, t):
    plant_discrete = signal.cont2discrete(
        (plant_cont.num, plant_cont.den), t[1] - t[0], method='zoh'
    )
    num_discrete = plant_discrete[0].flatten()
    den_discrete = plant_discrete[1].flatten()

    out = np.zeros_like(t)
    control = np.zeros_like(t)

    for i in range(2, len(t)):
        out[i] = - den_discrete[1] * out[i-1] - den_discrete[2] * out[i-2] + \
            num_discrete[1] * control[i-1] + num_discrete[2] * control[i-2]
        control[i] = regulator.regulate(out[i], reference[i])
    return out


def plot_responce(t, t_open, ref, open_loop, close_loop):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))

    ax1.plot(t_open, ref, 'k--', label="reference")
    ax1.plot(t_open, open_loop, label="output")
    ax1.set_title("Open-Loop Step Response")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("y(t)")
    ax1.grid(True)

    ax2.plot(t, ref, 'k--', label="reference")
    ax2.plot(t, close_loop, label="output")
    ax2.set_title("Closed-Loop PID Response")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("y(t)")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()
