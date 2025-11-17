import numpy as np

from csbenchlab.descriptor import ParamDescriptor
from csbenchlab.plugin import Controller
from csbenchlab.descriptor import LogEntry



class Controller2(Controller):

    param_description = [
        ParamDescriptor(
            name="p1",
            default_value = 0
        ),
        ParamDescriptor(
            name="p2",
            default_value = np.array([1, 2, 3])
        ),
        ParamDescriptor(
            name="p12",
            default_value = lambda params: params.p1 + params.p2[0]  # Example of a derived parameter
        )
    ]

    log_description = [
        LogEntry("d3")
    ]

    @staticmethod
    def create_data_model(params, mux):
        from types import SimpleNamespace
        return SimpleNamespace(
            d1=np.zeros(3),
            d2=np.zeros(3),
            d3=np.zeros(3),
        )

    def on_step(self, y_ref, y, dt):
        u = np.array([0.0, 0.0, 0.0]).T  # Control input initialized to zero
        dx = y_ref - y
        u[0] = self.params.p1 * dx[0]
        return u

    def on_reset(self):
        pass

    def on_configure(self):
        pass
