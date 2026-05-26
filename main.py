import argparse
from scipy import signal
import numpy as np
from pid import PID
import sim
from ppc import PolePlacementRegulator
from gpc import GeneralPredictiveController


def argument_parser():
    """Parse CLI controller choice."""
    parser = argparse.ArgumentParser()
    parser.add_argument("controller", choices=["pid", "pole", "gpc"])
    args = parser.parse_args()
    return args


def main():
    """Build plant, run simulation, plot results."""
    args = argument_parser()
    dt = 1
    t = np.linspace(0, 100, 101)
    ref = np.ones_like(t)
    Ys = [1.5]
    Us = [5, 5, 1]
    plant = signal.TransferFunction(Ys, Us)
    t_open, step_resp = signal.step(plant, T=t)
    regulator = 0
    plant_dis = signal.cont2discrete(
        (plant.num, plant.den), dt, method='zoh')
    num_d = plant_dis[0].flatten()
    den_d = plant_dis[1].flatten()
    if args.controller == "pid":
        r0 = 2.2414
        Ti = 11.5
        Td = 2.875
        regulator = PID(r0, dt, Ti, Td)
    elif args.controller == "pole":
        pole_const = - 0.5
        third_pole = - 3.0
        controller_p = [pole_const + pole_const *
                        1j, pole_const - pole_const * 1j]
        controller_p.append(third_pole)
        regulator = PolePlacementRegulator(den_d, num_d, controller_p, dt)
    elif args.controller == "gpc":
        regulator = GeneralPredictiveController(num_d, den_d, 10, 0.1)

    out = sim.simulate_plant(num_d, den_d, regulator, ref, t)
    sim.plot_responce(t, t_open, ref, step_resp, out)


if __name__ == "__main__":
    main()
