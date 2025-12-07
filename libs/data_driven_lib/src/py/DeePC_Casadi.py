from csbenchlab.plugin import CasadiController
from csbenchlab.descriptor import ParamDescriptor, DataModel
import casadi as ca
import numpy as np

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
        ParamDescriptor(name="y_max", default_value=1e6)
    ]

    @classmethod
    def create_data_model(cls, params, mux):
        return DataModel(
            uini=np.zeros(mux["Inputs"] * params.Tini),
            yini=np.zeros(mux["Outputs"] * params.Tini),
            u=np.zeros(mux["Inputs"] * params.L),
            y=np.zeros(mux["Outputs"] * params.L),
        )


    # build block Hankel matrices
    def _hankel(self, data, Lb):
        n, T = data.shape
        cols = T - Lb + 1
        if cols <= 0:
            raise RuntimeError("Not enough data samples for Hankel")
        H = np.zeros((n * Lb, cols))
        for i in range(Lb):
            H[i * n:(i + 1) * n, :] = data[:, i:i + cols]
        return H

    def casadi_configure(self):
        # Build DeePC data matrices from params.D_u / params.D_y and configure solver
        # Expect self.params to contain D_u and D_y (offline trajectories)
        D_u = getattr(self.params, "D_u", None)
        D_y = getattr(self.params, "D_y", None)
        if D_u is None or D_y is None:
            # nothing to configure yet; rely on user calling set_data or providing params
            return super().casadi_configure()

        U = np.asarray(D_u)
        Y = np.asarray(D_y)
        if U.ndim == 1:
            U = U.reshape(1, -1)
        if Y.ndim == 1:
            Y = Y.reshape(1, -1)

        m, T1 = U.shape
        p, T2 = Y.shape
        if T1 != T2:
            raise RuntimeError("D_u and D_y must have same length")

        L = int(self.params.L)
        Tini = int(self.params.Tini)

        Hu = self._hankel(U, L+Tini)
        Hy = self._hankel(Y, L+Tini)

        s = Hu.shape[1]

        mNp = m * Tini
        mNc = m * L
        pNp = p * Tini
        pNc = p * L

        Up = Hu[:mNp, :]
        Uf = Hu[mNp:mNp + mNc, :]
        Yp = Hy[:pNp, :]
        Yf = Hy[pNp:pNp + pNc, :]

        # weights
        Qp = self.params.Q
        Rp = self.params.R
        if np.isscalar(Qp):
            Q = np.eye(pNc) * float(Qp)
        else:
            Q = np.asarray(Qp)
        if np.isscalar(Rp):
            R = np.eye(mNc) * float(Rp)
        else:
            R = np.asarray(Rp)

        # regularization
        lambda_g = float(getattr(self.params, "lambda_g", 0.0))

        # terminal / slack params
        tc_size = int(getattr(self.params, "terminal_constraint_size", 0))
        use_u_tc = bool(getattr(self.params, "use_input_terminal_constraints", False))
        is_strict_tc = bool(getattr(self.params, "is_strict_terminal_constraint", True))
        lambda_term_y = float(getattr(self.params, "lambda_term_y", 0.0))
        lambda_term_u = float(getattr(self.params, "lambda_term_u", 0.0))

        # store matrices for step function
        self.m = m
        self.p = p
        self.L = L
        self.Np = Tini
        self.s = s
        self.Up = Up
        self.Uf = Uf
        self.Yp = Yp
        self.Yf = Yf
        self.Q = Q
        self.R = R
        self.lambda_g = lambda_g

        # build CasADi DM versions
        self.Uf_DM = ca.DM(Uf)
        self.Yf_DM = ca.DM(Yf)
        self.G_DM = ca.DM(np.vstack([Up, Yp]))

        # --- Build NLP here so casadi_step_fn can be minimal ---
        # Symbols for parameters (only uini, yini, y_ref as requested)
        mNp = m * Tini
        pNp = p * Tini
        pL = p * L

        uini = ca.MX.sym("u_ini", mNp)
        yini = ca.MX.sym("y_ini", pNp)
        y_ref = ca.MX.sym("y_ref", pL)

        # decision variable (we may append slack variables later)
        # start with g of length s
        g = ca.MX.sym("g", s)
        x_vars = [g]
        x_sizes = [s]

        # DM matrices for use in expressions
        Uf_DM = ca.DM(Uf)
        Yf_DM = ca.DM(Yf)
        G_DM = ca.DM(np.vstack([Up, Yp]))

        # predicted sequences
        y_pred = ca.mtimes(Yf_DM, g)  # (p*L, 1)
        u_pred = ca.mtimes(Uf_DM, g)  # (m*L, 1)

        Q_DM = ca.DM(Q)
        R_DM = ca.DM(R)

        e = y_pred - y_ref
        obj = ca.mtimes([e.T, Q_DM, e]) + ca.mtimes([u_pred.T, R_DM, u_pred]) + lambda_g * ca.mtimes(g.T, g)
        obj = ca.reshape(obj, (), 1)

        # equality constraints from past
        b = ca.vertcat(uini, yini)
        cons = ca.mtimes(G_DM, g) - b

        # terminal constraints: derive terminal refs from y_ref and uini (no extra params)
        tc = int(tc_size)
        if tc > 0:
            # y_ref is (p, L) flattened; reshape to pick last tc columns
            y_ref_mat = ca.reshape(y_ref, (p, L))
            y_term_ref_mat = y_ref_mat[:, L - tc:L]
            y_term_ref_vec = ca.reshape(y_term_ref_mat, (p * tc, 1))

            # predicted terminal outputs
            y_pred_mat = ca.reshape(y_pred, (p, L))
            y_term_pred = y_pred_mat[:, L - tc:L]
            y_term_pred_vec = ca.reshape(y_term_pred, (p * tc, 1))

            if is_strict_tc:
                # hard equality
                cons = ca.vertcat(cons, (y_term_pred_vec - y_term_ref_vec))
            else:
                # soft constraint: introduce slack sy (p*tc)
                sy = ca.MX.sym("s_term_y", p * tc)
                x_vars.append(sy)
                x_sizes.append(p * tc)
                cons = ca.vertcat(cons, (y_term_pred_vec - y_term_ref_vec - sy))
                # add quadratic penalty on slack
                obj = obj + lambda_term_y * ca.mtimes(sy.T, sy)

            if use_u_tc:
                # derive terminal input reference from last column of uini
                uini_mat = ca.reshape(uini, (m, Tini))
                last_u = uini_mat[:, -1]
                u_term_ref_mat = ca.repmat(last_u, 1, tc)
                u_term_ref_vec = ca.reshape(u_term_ref_mat, (m * tc, 1))

                u_pred_mat = ca.reshape(u_pred, (m, L))
                u_term_pred = u_pred_mat[:, L - tc:L]
                u_term_pred_vec = ca.reshape(u_term_pred, (m * tc, 1))

                if is_strict_tc:
                    cons = ca.vertcat(cons, (u_term_pred_vec - u_term_ref_vec))
                else:
                    su = ca.MX.sym("s_term_u", m * tc)
                    x_vars.append(su)
                    x_sizes.append(m * tc)
                    cons = ca.vertcat(cons, (u_term_pred_vec - u_term_ref_vec - su))
                    obj = obj + lambda_term_u * ca.mtimes(su.T, su)

        # assemble decision variable from parts
        if len(x_vars) == 1:
            x_all = x_vars[0]
        else:
            x_all = ca.vertcat(*x_vars)

        # assemble solver parameter (only uini,yini,y_ref)
        p_param = ca.vertcat(uini, yini, y_ref)

        nlp = {"x": x_all, "f": obj, "g": cons, "p": p_param}
        opts = {"ipopt.print_level": 0, "print_time": 0}
        solver = ca.nlpsol("solver", "ipopt", nlp, opts)

        # prepare_data packs uini,yini,y_ref into p (trivial) and update_data maps g->u0
        prepare_data = ca.Function("prepare_data", [uini, yini, y_ref], [p_param])
        # extract g portion from x_all to compute u0
        if len(x_sizes) == 1:
            g_from_x = x_all
        else:
            # g is the first block
            g_from_x = x_all[0:s]

        u_pred_full = ca.mtimes(Uf_DM, g_from_x)
        u_pred_full = ca.reshape(u_pred_full, (m, L))
        u0 = u_pred_full[:, 0]
        update_data = ca.Function("update_data", [x_all], [u0])

        # store for step function
        self.prepare_data = prepare_data
        self.update_data = update_data
        self.solver = solver

        return super().casadi_configure()

    def casadi_step_fn(self):
        # casadi_step_fn should be minimal: return prebuilt functions
        if not hasattr(self, "solver"):
            raise RuntimeError("Call casadi_configure before casadi_step_fn")

        return [self.prepare_data, self.solver, self.update_data]

