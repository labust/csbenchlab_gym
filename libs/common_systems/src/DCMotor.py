from csbenchlab.plugin import CasadiDynSystem
from csbenchlab.descriptor import ParamDescriptor
import casadi as ca

class DCMotor(CasadiDynSystem):


    param_description = [
        ParamDescriptor(name='J', default_value=0.01, type='float', description='Moment of inertia (kg*m^2)'),
        ParamDescriptor(name='b', default_value=0.1, type='float', description='Viscous friction coefficient (N*m*s/rad)'),
        ParamDescriptor(name='Kt', default_value=0.01, type='float', description='Torque constant (N*m/A)'),
        ParamDescriptor(name='Ke', default_value=0.01, type='float', description='Back-emf constant (V*s/rad)'),
        ParamDescriptor(name='R', default_value=1.0, type='float', description='Armature resistance (Ohm)'),
        ParamDescriptor(name='L', default_value=0.5, type='float', description='Armature inductance (H)'),
    ]


    @classmethod
    def get_dims_from_params(cls, params):
        return {
            "Inputs": 2,
            "Outputs": 3
        }

    def casadi_configure(self):
        # state and derivative placeholders (theta, omega, i)
        self.x = ca.MX(3, 1)
        self.dx = ca.MX(3, 1)

    def casadi_step_fn(self):
        # inputs: u[0] = applied voltage V (V), u[1] = load torque tau_load (N*m)
        u = ca.MX(2, 1)

        theta = self.x[0]
        omega = self.x[1]
        i = self.x[2]

        V = u[0]
        tau_load = u[1]

        J = self.params.J
        b = self.params.b
        Kt = self.params.Kt
        Ke = self.params.Ke
        R = self.params.R
        L = self.params.L

        # state derivatives
        theta_dot = omega
        omega_dot = (Kt * i - b * omega - tau_load) / J
        i_dot = (1.0 / L) * (V - R * i - Ke * omega)

        self.dx[0] = theta_dot
        self.dx[1] = omega_dot
        self.dx[2] = i_dot

        return [ca.Function('dc_motor', [self.x, u], [self.dx], ["x", "u"], ["dx"])]

