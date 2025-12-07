
from argparse import ArgumentParser
from csbenchlab.backend.python_backend import PythonBackend
from csbenchlab.scenario_templates.control_environment import ControlEnvironment
from m_scripts.eval_metrics import eval_metrics
from bdsim import BDSim
from matplotlib import pyplot as plt
import numpy as np



class LiveMetricPoleCart:
    def __init__(self):
        # init matplotlib figure. I want to plot cart in 1d space with pole on top
        self.fig, self.ax = plt.subplots(figsize=(10, 4))
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-0.5, 2.5)
        self.ax.set_aspect('equal')
        self.ax.axhline(y=0, color='k', linewidth=0.5)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('Cart Position (m)')
        self.ax.set_ylabel('Height (m)')
        self.ax.set_title('Cart Inverted Pendulum Live Visualization')

        # Initialize cart and pole line objects
        self.cart, = self.ax.plot([], [], 'bs', markersize=10, label='Cart')
        self.pole, = self.ax.plot([], [], 'r-', linewidth=2, label='Pole')
        self.ax.legend()

        plt.ion()  # Turn on interactive mode

    def __call__(self, time, value):

        if not hasattr(self, 'last_t'):
            self.last_t = time
            return
        # this is called at each simulation step
        if time - self.last_t == 0:
            return
        dt = (time - self.last_t)
        self.last_t = time
        x = value[2]
        theta = np.pi + value[3]
        self.cart.set_data([x], [0])
        pole_x = x + np.sin(theta)
        pole_y = np.cos(theta)
        self.pole.set_data([x, pole_x], [0, pole_y])
        self.fig.canvas.draw_idle()
        plt.pause(dt)


def eval_control_environment(env_path, system_instance:str=None, controller_ids:str=None):


    backend = PythonBackend()
    env_params, data = backend.load_control_environment_params_and_data(env_path, system_instance, controller_ids)

    env = ControlEnvironment(env_path, data.metadata, backend=backend)

    live_metrics = [LiveMetricPoleCart()]
    env.generate({
        "system": data.systems[0],
        "controllers": data.controllers
    },
    env_params=env_params,
    generate_scopes=False,
    live_metrics=live_metrics)

    scenario = env.select_scenario(0)
    env.compile()
    out = env.run(T=scenario["SimulationTime"])


    r = eval_metrics(data, out)
    plt.show()
    print(r)


def main():
    env_path = "/home/luka/fer/csbenchlab_gym/envs/cart_inverted_pendulum"
    system_instance = "49ce58b6-532e-4c7f-90b9-9fa38707fae5"
    controller_ids = ["0ac647ec-1937-455b-80a1-fcf0c4db240c", "43a8b674-a981-4b52-9e74-b37728839260", "6d1af3df-25b6-4eaa-b235-da23b8d2fd2d", "a2d09404-1425-4551-b056-e0738041c105"]
    eval_control_environment(env_path, system_instance, controller_ids)

if __name__ == "__main__":
    main()
