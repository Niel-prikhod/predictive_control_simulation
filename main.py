import argparse
from scipy import signal
import numpy as np
from pid import PID
import sim
from ppc import PolePlacementRegulator


def argument_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("controller", choices=["pid", "pole", "mpc"])
    args = parser.parse_args()
    return args


def pole_constructor(controller, observer, dt):
    s_controller = np.array(
        [controller - controller * 1j, controller + controller * 1j])
    contr_poles = np.exp(s_controller * dt)
    s_observer = np.array(
        [observer - observer * 1j, observer + observer * 1j])
    obs_poles = np.exp(s_observer * dt)
    return contr_poles, obs_poles


def main():
    args = argument_parser()
    dt = 1
    t = np.linspace(0, 100, 101)
    ref = np.ones_like(t)
    Ys = [1.5]
    Us = [5, 5, 1]
    plant = signal.TransferFunction(Ys, Us)
    t_open, step_resp = signal.step(plant, T=t)
    regulator = 0
    if args.controller == "pid":
        r0 = 2.2414
        Ti = 11.5
        Td = 2.875
        regulator = PID(r0, dt, Ti, Td)
    elif args.controller == "pole":
        A_c, B_c, C_c, D_c = signal.tf2ss(plant.num, plant.den)
        plant_cont_ss = (A_c, B_c, C_c, D_c)
        A_d, B_d, C_d, D_d, _ = signal.cont2discrete(
            plant_cont_ss, dt, method='zoh')
        B_d = B_d.reshape(-1, 1)
        # print("C shape:", C_d.shape)
        # C_d = C_d.reshape(-1, 1)
        # print("C after reshape:", C_d.shape)
        pole_const = - 0.5
        obs_pole_const = 3 * pole_const
        controller_p, observer_p = pole_constructor(
            pole_const, obs_pole_const, dt)
        regulator = PolePlacementRegulator(
            A_d, B_d, C_d, controller_p, observer_p)
    out = sim.simulate_plant(plant, regulator, ref, t)
    sim.plot_responce(t, t_open, ref, step_resp, out)


if __name__ == "__main__":
    main()
