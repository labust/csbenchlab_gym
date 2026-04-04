from csbenchlab.plugin import CasadiController
from csbenchlab.descriptor import ParamDescriptor, DataModel
import casadi as ca
import numpy as np
from data_driven_lib import deepc_utils as dutils


class DeePC_position_differential(CasadiController):
    """
    DeePC controller with differential kinematics for position tracking.

    Input: Control inputs (e.g., accelerations)
    Output: Velocities (e.g., vx, vy, vyaw)

    The controller uses DeePC for velocity prediction and integrates with
    differential kinematics to predict positions. The optimization minimizes
    errors in both predicted velocities and positions relative to a reference
    trajectory (which includes both velocity and position references).

    State representation: 6D = [vx, vy, vyaw, x, y, yaw]
    """

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
        ParamDescriptor(name="y_max", default_value=1e6)
    ]

    @classmethod
    def create_data_model(cls, options):
        """
        Create data model for position-differential controller.

        uini: Past control inputs
        yini: Past velocity outputs
        yini_pos: Past position states
        u: Future control inputs
        y: Future velocity outputs (combined with position in y_ref during optimization)
        """
        num_velocity_outputs = mux.get("Outputs", 3)  # Default to 3D (vx, vy, vyaw)
        return DataModel(
            uini=np.zeros(mux["Inputs"] * params.Tini),
            yini=np.zeros(num_velocity_outputs * params.Tini),
            yini_pos=np.zeros(num_velocity_outputs * params.Tini),
            u=np.zeros(mux["Inputs"] * params.L),
            y=np.zeros(num_velocity_outputs * params.L),
        )

    def casadi_configure(self):
        D_u = getattr(self.params, "D_u", None)
        D_y = getattr(self.params, "D_y", None)
        if D_u is None or D_y is None:
            return super().casadi_configure()

        U = np.asarray(D_u)
        Y = np.asarray(D_y)

        result = dutils.build_position_differential_controller(U, Y, self.params)

        # Store solver and functions
        self.solver = result['solver']
        self.prepare_data = result['prepare_data']
        self.update_data = result['update_data']

        # Store matrices for inspection if needed
        for key, val in result['matrices'].items():
            setattr(self, key, val)
        for key, val in result['dimensions'].items():
            setattr(self, key, val)
        self.Np = self.Tini

        return super().casadi_configure()

    def casadi_step_fn(self):
        if not hasattr(self, "solver"):
            raise RuntimeError("Call casadi_configure before casadi_step_fn")

        return [self.prepare_data, self.solver, self.update_data]
