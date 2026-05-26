import matplotlib.pyplot as plt
import numpy as np
import os


def simulate_plant(num_discrete, den_discrete, regulator, reference, t):
    """Discretise plant and run closed-loop simulation."""
    out = np.zeros_like(t)
    control = np.zeros_like(t)

    for i in range(2, len(t)):
        out[i] = - den_discrete[1] * out[i-1] - den_discrete[2] * out[i-2] + \
            num_discrete[1] * control[i-1] + num_discrete[2] * control[i-2]
        control[i] = regulator.regulate(out[i], reference[i])
    return out


def plot_responce(t, out, ref, save, controller, out_folder):
    """Plot open-loop and closed-loop step responses side-by-side."""
    filename = f"{controller}_response.png"
    filepath = os.path.join(out_folder, filename)
    if controller != "null":
        title = f"{controller.upper()} Regulation Response"
    else:
        title = "Open-loop Responce"
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(t, ref, 'k--', label="reference")
    ax.plot(t, out, label="output")
    ax.set_title(title)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("y(t)")
    ax.legend()
    ax.grid(True)

    if save == 1:
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        plt.close()
    else:
        plt.show()
