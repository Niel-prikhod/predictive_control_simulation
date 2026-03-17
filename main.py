from scipy import signal
import numpy as np
import matplotlib.pyplot as plt
from pid import PID

def main():
    Kp = 50
    t = np.linspace(0, 40, 1000)
    dt = t[1] - t[0]

    ref = np.ones_like(t)    

    out = np.zeros_like(t)  
    control = np.zeros_like(t) 

    regulator = PID(Kp)
    plant = signal.TransferFunction([1.5], [5, 5, 1])   
    plant_state = np.zeros(len(plant.den) - 1)

    t_out, y_out = signal.step(plant, T=t)

    for i in range(1, len(t)):
        error = ref[i] - out[i-1]
        control[i] = regulator.regulate(error, dt)
        _, out_step, in_step = signal.lsim(
                plant,
                U=[control[i-1], control[i]],
                T=[t[i-1], t[i]],
                X0=plant_state
                )
        out[i] = out_step[-1]
        plant_state = in_step[-1]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))
    # --- Left plot: open-loop plant response ---
    ax1.plot(t_out, ref, 'k--', label="reference")
    ax1.plot(t_out, y_out, label="output")
    ax1.set_title("Open-Loop Step Response")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("y(t)")
    ax1.grid(True)

    # --- Right plot: closed-loop PID response ---
    ax2.plot(t, ref, 'k--', label="reference")
    ax2.plot(t, out, label="output")
    ax2.set_title("Closed-Loop PID Response")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("y(t)")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
