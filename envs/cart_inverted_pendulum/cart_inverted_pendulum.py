
from argparse import ArgumentParser
from csbenchlab.backend.python_backend import PythonBackend
from csbenchlab.scenario_templates.control_environment import ControlEnvironment
from m_scripts.eval_metrics import load_metrics, eval_metrics
from matplotlib import pyplot as plt


def eval_control_environment(env_path, system_instance:str=None, controller_ids:str=None):


    backend = PythonBackend()
    env_params, data = backend.load_control_environment_params_and_data(env_path, system_instance, controller_ids)

    env = ControlEnvironment(env_path, data.metadata, backend=backend)

    metrics = load_metrics(env_path)
    env.generate({
        "system": data.systems[0],
        "controllers": data.controllers
    },
    env_params=env_params,
    generate_scopes=False,
    live_metrics=[m.metric for m in metrics.live_metrics]
    )

    scenario = env.select_scenario(1)
    env.compile()
    out = env.run(T=scenario["SimulationTime"])


    r = eval_metrics(metrics.post_metrics, out)
    plt.show()
    print(r)


def main():
    env_path = "/home/luka/fer/csbenchlab_gym/envs/cart_inverted_pendulum"
    system_instance = "49ce58b6-532e-4c7f-90b9-9fa38707fae5"
    controller_ids = ["0ac647ec-1937-455b-80a1-fcf0c4db240c", "43a8b674-a981-4b52-9e74-b37728839260", "6d1af3df-25b6-4eaa-b235-da23b8d2fd2d", "a2d09404-1425-4551-b056-e0738041c105"]
    eval_control_environment(env_path, system_instance, controller_ids)

if __name__ == "__main__":
    main()
