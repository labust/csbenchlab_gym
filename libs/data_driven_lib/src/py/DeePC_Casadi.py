from csbenchlab.plugin import CasadiController
from csbenchlab.descriptor import ParamDescriptor
import casadi as ca
import numpy as np
from data_driven_lib import deepc_utils as dutils
from types import SimpleNamespace

class DeePC_Casadi(CasadiController):

    param_description = [
        ParamDescriptor(name="L", default_value=10),
        ParamDescriptor(name="Tini", default_value=1),
        ParamDescriptor(name="Ts", default_value=1),
        ParamDescriptor(name="D_u", default_value=0),
        ParamDescriptor(name="D_y", default_value=0),
        ParamDescriptor(name="end_point", default_value=0),
        ParamDescriptor(name="T", default_value=lambda params: len(params.D_u)),
        ParamDescriptor(name="R", default_value=1),
        ParamDescriptor(name="Q", default_value=1),
        ParamDescriptor(name="lambda_g", default_value=0),
        ParamDescriptor(name="lambda_s", default_value=0),
        ParamDescriptor(name="lambda_s_ini", default_value=0),
        ParamDescriptor(name="lambda_term_u", default_value=0),
        ParamDescriptor(name="lambda_term_y", default_value=0),
        ParamDescriptor(name="Lc", default_value=lambda params: params.L),
        ParamDescriptor(name="terminal_constraint_size", default_value=1),
        ParamDescriptor(name="affine_constraint", default_value=1),
        ParamDescriptor(name="use_overshoot_constraints", default_value=1),
        ParamDescriptor(name="use_input_terminal_constraints", default_value=1),
        ParamDescriptor(name="use_input_delta_constraints", default_value=0),
        ParamDescriptor(name="use_projected_regularization", default_value=0),
        ParamDescriptor(name="is_strict_terminal_constraint", default_value=1),
        ParamDescriptor(name="use_ref_integral", default_value=0),
        ParamDescriptor(name="Ki", default_value=1),
        ParamDescriptor(name="decay", default_value=1),
        ParamDescriptor(name="allowed_offset_terminal", default_value=1),
        ParamDescriptor(name="allowed_offset_slack", default_value=1),
        ParamDescriptor(name="u_min", default_value=-1e6),
        ParamDescriptor(name="u_max", default_value=1e6),
        ParamDescriptor(name="y_min", default_value=-1e6),
        ParamDescriptor(name="y_max", default_value=1e6),
        ParamDescriptor(name="input_delta", default_value=1e6)
    ]

    @classmethod
    def create_data_model(cls, options):
        return dutils.create_basic_data_model(options)

    def casadi_configure(self):
        """Build the QP controller using deepc_utils."""
        params = self.params

        # Get data dimensions
        m = self.data.m
        p = self.data.p
        idx = dutils.set_param_indices_and_dims(params, self.data.T, m, p)

        # Prepare data trajectories - transpose to match expected format (T, m) and (T, p)
        U = np.array(params.D_u).reshape(-1, m)  # Shape: (T, m)
        Y = np.array(params.D_y).reshape(-1, p)  # Shape: (T, p)

        self.data.A = dutils.update_data_matrix(idx, self.data.A, \
            U, Y, [], self.data.T, m, p, params)

        [self.data.lb, self.data.ub] = dutils.configure_bounds( \
            self.data.lb, self.data.ub, idx, params)



    def prestep(self):

        m = self.data.m
        p = self.data.p

        uini = ca.SX.sym("uini", self.data.Tini * m)
        yini = ca.SX.sym("yini", self.data.Tini * p)
        y = ca.SX.sym("y", p)
        y_ref = ca.SX.sym("y_ref", p)


        yini = dutils.update_ini(self.data.y, yini, self.data.Tini * p)

        self.data.b = dutils.update_b_vector(self.data.b, uini, yini, self.data.Tini, m, p)

        self.data.A = dutils.update_data_matrix(self.data.idx, self.data.A, \
            self.data.U, self.data.Y, self.data.H, self.data.T, m, p, self.params)

        self.data = dutils.update_matrices(self.data, self.params)

        self.data.optimT = dutils.set_optim_params(self.data, self.params)

        if self.data.has_lt:
            self.solver = ca.qpsol("solver", "qpoases", self.data.optimT)
        else:
            self.solver = ca.qpsol("solver", "qrqp", self.data.optimT)


    def casadi_step_fn(self):

        # return [self.prestep(), solve_qp, extract_control]
        return [self.prestep()]

