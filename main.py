from scipy import signal
import numpy as np
from pid import PID
import sim


def main():
    T = 1
    t = np.linspace(0, 100, 101)
    ref = np.ones_like(t)
    Ys = [1.5]
    Us = [5, 5, 1]
    plant = signal.TransferFunction(Ys, Us)
    t_open, step_resp = signal.step(plant, T=t)

    r0 = 2.2414
    Ti = 11.5
    Td = 2.875
    regulator = PID(r0, T, Ti, Td)

    out = sim.simulate_plant(plant, regulator, ref, t)
    sim.plot_responce(t, t_open, ref, step_resp, out)


if __name__ == "__main__":
    main()
