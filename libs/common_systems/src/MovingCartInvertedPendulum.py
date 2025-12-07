
from csbenchlab.plugin import CasadiContinuousDynSystem
from csbenchlab.descriptor import ParamDescriptor
import casadi as ca

class MovingCartInvertedPendulum(CasadiContinuousDynSystem):
    """
    Moving Cart with Inverted Pendulum System
    The system consists of a cart that can move horizontally and an inverted pendulum
    attached to the cart. The goal is to control the cart's movement to keep the
    pendulum balanced in the upright position.

    States:
        x: horizontal position of the cart (m)
        x_dot: horizontal velocity of the cart (m/s)
        theta: angle of the pendulum from vertical (pi is upright) (rad)
        theta_dot: angular velocity of the pendulum (rad/s)
    """

    param_description = [
        ParamDescriptor(name='M', default_value=5.0, type='float', description='Mass of the cart (kg)'),
        ParamDescriptor(name='m', default_value=1.0, type='float', description='Mass of the pendulum (kg)'),
        ParamDescriptor(name='l', default_value=2.0, type='float', description='Length to center of mass of the pendulum (m)'),
        ParamDescriptor(name='g', default_value=-9.81, type='float', description='Acceleration due to gravity (m/s^2)'),
        ParamDescriptor(name='delta', default_value=1, type='float', description='Damping coefficient at the pendulum pivot (N m s / rad)'),
    ]

    @classmethod
    def get_dims_from_params(cls, params):
        return {
            "Inputs": 1,
            "Outputs": 4
        }

    def casadi_configure(self):

        # state and derivative placeholders (4 states: x_dot, theta_dot, x, theta)
        self.x = ca.MX.sym('x', 4, 1)
        self.dx = ca.MX(4, 1)

    def casadi_step_fn(self):
        # inputs: u[0] = horizontal force on cart (N)
        u = ca.MX.sym('u', 1, 1)
        x_dot = self.x[0]
        theta_dot = self.x[1]
        x = self.x[2]
        theta = self.x[3]

        M = self.params.M
        m = self.params.m
        l = self.params.l
        g = self.params.g
        delta = self.params.delta

        F = u[0]

        s = ca.sin(theta)
        c = ca.cos(theta)

        den = m * l * l * (M + m * (1 - c**2))  # small term to avoid div by zero

        # equations of motion
        theta_ddot = ((m + M) * m * g * l * s \
            -  m * l * c * (m * l * theta_dot**2 * s - delta * x_dot) \
            + m * l * c * F) / den
        x_ddot = (- m * m * l * l * g * c * s\
            + m * l * l * (m * l * theta_dot**2 * s - delta * x_dot) \
            + m * l * l * F) / den

        # assemble derivative vector
        self.dx[0] = x_ddot
        self.dx[1] = theta_ddot
        self.dx[2] = x_dot
        self.dx[3] = theta_dot

        return [ca.Function('moving_cart_inverted_pendulum', [self.x, u], [self.dx], ["x", "u"], ["dx"])]

