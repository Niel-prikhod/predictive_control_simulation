import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

def simulate_plant(plant, regulator, reference, t):
    plant_state = np.zeros(len(plant.den) - 1)
    out = np.zeros_like(t)  
    control = np.zeros_like(t) 
    dt = t[1] - t[0]

    for i in range(1, len(t)):
        error = reference[i] - out[i-1]
        control[i] = regulator.regulate(error, dt)
        _, out_step, in_step = signal.lsim(
                plant,
                U=[control[i-1], control[i]],
                T=[t[i-1], t[i]],
                X0=plant_state
                )
        out[i] = out_step[-1]
        plant_state = in_step[-1]
    return out

def plot_responce(t, ref, open_loop, close_loop):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))

    ax1.plot(t, ref, 'k--', label="reference")
    ax1.plot(t, open_loop, label="output")
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

