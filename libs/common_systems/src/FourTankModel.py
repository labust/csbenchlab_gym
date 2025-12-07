from csbenchlab.plugin import CasadiDynSystem
from csbenchlab.descriptor import ParamDescriptor
import casadi as ca

class FourTankModel(CasadiDynSystem):


    param_description = [
        ParamDescriptor(name='gamma1', default_value=0.4, type='float', description='Valve coefficient for tank 1'),
        ParamDescriptor(name='gamma2', default_value=0.4, type='float', description='Valve coefficient for tank 2'),
        ParamDescriptor(name='A1', default_value=50.27, type='float', description='Cross-sectional area of tank 1 (cm^2)'),
        ParamDescriptor(name='A2', default_value=50.27, type='float', description='Cross-sectional area of tank 2 (cm^2)'),
        ParamDescriptor(name='A3', default_value=28.27, type='float', description='Cross-sectional area of tank 3 (cm^2)'),
        ParamDescriptor(name='A4', default_value=28.27, type='float', description='Cross-sectional area of tank 4 (cm^2)'),
        ParamDescriptor(name='a1', default_value=0.233, type='float', description='Outlet hole area of tank 1 (cm^2)'),
        ParamDescriptor(name='a2', default_value=0.242, type='float', description='Outlet hole area of tank 2 (cm^2)'),
        ParamDescriptor(name='a3', default_value=0.127, type='float', description='Outlet hole area of tank 3 (cm^2)'),
        ParamDescriptor(name='a4', default_value=0.127, type='float', description='Outlet hole area of tank 4 (cm^2)'),
        ParamDescriptor(name='g', default_value=981, type='float', description='Acceleration due to gravity (cm/s^2)'),
    ]

    @classmethod
    def get_dims_from_params(cls, params):
        return {
            "Inputs": 2,
            "Outputs": 4
        }

    def casadi_configure(self):
        # Constants
        self.x = ca.MX(4, 1)
        self.dx = ca.MX(4, 1)


    def casadi_step_fn(self):
        u = ca.MX(2, 1)
        g = self.params.g
        A1 = self.params.A1
        A2 = self.params.A2
        A3 = self.params.A3
        A4 = self.params.A4
        a1 = self.params.a1
        a2 = self.params.a2
        a3 = self.params.a3
        a4 = self.params.a4
        gamma1 = self.params.gamma1
        gamma2 = self.params.gamma2


        self.dx[0] = -a1/A1 * ca.sqrt(2*g*self.x[0]) \
            + a3/A1 * ca.sqrt(2*g*self.x[2]) \
            + gamma1/A1 * u[0]
        self.dx[1] = -a2/A2 * ca.sqrt(2*g*self.x[1]) \
            + a4/A2 * ca.sqrt(2*g*self.x[3]) \
            + gamma2/A2 * u[1]
        self.dx[2] = -a3/A3 * ca.sqrt(2*g*self.x[2]) \
            + (1 - gamma2)/A3 * u[1]
        self.dx[3] = -a4/A4 * ca.sqrt(2*g*self.x[3]) \
            + (1 - gamma1)/A4 * u[0]

        return [ca.Function('four_tank',[self.x, u], [self.dx], ["x", "u"], ["dx"])]

