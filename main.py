from scipy import signal
import numpy as np
from pid import PID
import sim

def main():
    t = np.linspace(0, 40, 1000)
    ref = np.ones_like(t)    

    Kp, Ki, Kd = 10.5, 0, 0
    regulator = PID(Kp, Ki, Kd)

    plant = signal.TransferFunction([1.5], [5, 5, 1])   

    t, y_out = signal.step(plant, T=t)

    out = sim.simulate_plant(plant, regulator, ref, t)
    sim.plot_responce(t, ref, y_out, out)

if __name__ == "__main__":
    main()
