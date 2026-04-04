import numpy as np
from csbenchlab.descriptor import DataModel
from types import SimpleNamespace


class Indexer:
    def __init__(self, b, e):
        self.b = b
        self.e = e
        self.sz = e - b
        self.r = slice(b, e)  # Python uses 0-based indexing


def get_traj_info(D_u, D_y):
    m = D_u.shape[1]
    p = D_y.shape[1]
    T = D_u.shape[0]
    return T, m, p



def configure_bounds(lb, ub, idx, params):
    term = params.terminal_constraint_size
    term_off = params.allowed_offset_terminal

    lb[idx.u.r] = np.full((params.L, 1), params.u_min)
    ub[idx.u.r] = np.full((params.L, 1), params.u_max)
    lb[idx.y.r] = np.full((params.L, 1), params.y_min)
    ub[idx.y.r] = np.full((params.L, 1), params.y_max)
    slack_off = params.allowed_offset_slack
    if slack_off == 0:
        slack_off = 1
    lb[idx.s.b:idx.s.e] = np.full((params.L + params.Tini, 1), params.y_min * slack_off)
    ub[idx.s.b:idx.s.e] = np.full((params.L + params.Tini, 1), params.y_max * slack_off)
    if term > 0 and params.is_strict_terminal_constraint == 0:
        lb[idx.yterm.r] = np.full((params.terminal_constraint_size, 1), params.y_min * term_off)
        ub[idx.yterm.r] = np.full((params.terminal_constraint_size, 1), params.y_max * term_off)
    return lb, ub


def create_basic_data_model(params, mux):
    dm = DataModel()
    dm.Ts = params.Ts
    T, m, p = get_traj_info(np.array(params.D_u).reshape(-1, mux["Inputs"]),
                            np.array(params.D_y).reshape(-1, mux["Outputs"]))
    dm.T = T
    dm.m = m
    dm.p = p
    dm.L = params.L
    dm.Tini = params.Tini

    idx = set_param_indices_and_dims(params, T, m, p)
    dm.x_op_u = np.zeros((dm.m * params.L, 1))
    dm.x_op_y = np.zeros((dm.p * params.L, 1))
    dm.x_op_g = np.zeros((dm.T - params.L - params.Tini + 1, 1))

    data = create_optim_matrices(dm, idx, params, T, m, p)
    data.x_op = np.zeros((idx.state.e, 1))
    data.lb = np.full((idx.state.e, 1), -np.inf)
    data.ub = np.full((idx.state.e, 1), np.inf)
    data.fval = 0.0
    data.old_y_ref = 0.0
    data.eta = np.zeros((data.p, 1))
    data.has_lt = 1
    if data.b_lt.shape[0] == 1 and data.b_lt[0] == 0:
        data.has_lt = 0

    # Additional initialization and checks can be added here

    dm.uini = np.zeros(mux["Inputs"] * params.Tini)
    dm.yini = np.zeros(mux["Outputs"] * params.Tini)
    # Other fields can be initialized as needed

    return data


def set_param_indices_and_dims(params, T, m, p):

    idx = SimpleNamespace()
    Tini = params.Tini
    L = params.L
    Lc = params.Lc
    affine_constraint = params.affine_constraint
    terminal_constraint_size = params.terminal_constraint_size
    use_input_terminal_constraints = params.use_input_terminal_constraints
    is_strict_terminal_constraint = params.is_strict_terminal_constraint
    use_input_delta_constraints = params.use_input_delta_constraints
    use_overshoot_constraints = params.use_overshoot_constraints
    idx.m = m
    idx.p = p

    idx.uini_v = Indexer(0, m*Tini)
    idx.u_v = Indexer(m*Tini, m*(Tini + L))

    idx.yini_v = Indexer(m*(Tini + L), m*(Tini + L) + p*Tini)
    idx.y_v = Indexer(m*(Tini + L) + p * Tini, (m + p) * (L + Tini))

    # COLS DIMENSIONS
    idx.u = Indexer(0, m*L)
    idx.y = Indexer(m*L, (m + p)*L)

    dim_a = T-L-Tini+1
    curr_state_dim = (m + p) * L
    curr_v_dim = (m + p) * (Tini + L)

    idx.s = Indexer(curr_state_dim, curr_state_dim + p*(L + Tini))
    curr_state_dim = curr_state_dim + p * (L + Tini)

    idx.A_v = Indexer(0, curr_v_dim)
    term_v_b = curr_v_dim

    idx.yterm = Indexer(-1, -1)
    idx.uterm = Indexer(-1, -1)
    idx.uterm_v = Indexer(-1, -1)
    idx.yterm_v = Indexer(-1, -1)
    idx.affine_v = Indexer(-1, -1)

    if affine_constraint > 0:
        idx.affine_v = Indexer(curr_v_dim, curr_v_dim + 1)
        curr_v_dim = curr_v_dim + 1
    total_constraints = affine_constraint
    if terminal_constraint_size > 0:
        idx.yterm_v = Indexer(curr_v_dim, curr_v_dim + p * terminal_constraint_size - 1)
        curr_v_dim = curr_v_dim + p * terminal_constraint_size
        if not (use_input_terminal_constraints == 0):
            idx.uterm_v = Indexer(curr_v_dim, curr_v_dim + m * terminal_constraint_size - 1)
            curr_v_dim = curr_v_dim + m * terminal_constraint_size
        else:
            idx.uterm_v = Indexer(-1, -1)
        terminal_constraints = p * terminal_constraint_size + use_input_terminal_constraints * m * terminal_constraint_size
        if is_strict_terminal_constraint == 0:
            idx.yterm = Indexer(curr_state_dim, curr_state_dim + p * terminal_constraint_size - 1)
            if not (use_input_terminal_constraints == 0):
                idx.uterm = Indexer(curr_state_dim + p * terminal_constraint_size,
                                    curr_state_dim + (m + p) * terminal_constraint_size - 1)
                curr_state_dim = curr_state_dim + terminal_constraints
        total_constraints = total_constraints + terminal_constraints
    idx.total_constraints = Indexer(0, total_constraints)
    idx.term_v = Indexer(term_v_b, curr_v_dim)
    idx.a = Indexer(curr_state_dim, curr_state_dim + dim_a)
    curr_state_dim = curr_state_dim + dim_a

    idx.state = Indexer(0, curr_state_dim)

    idx.u_lt = Indexer(-1, -1)
    idx.y_lt = Indexer(-1, -1)
    if use_input_delta_constraints != 0:
        idx.u_lt = Indexer(0, 2*m*Lc)
        end_lt = idx.u_lt.e

    if use_overshoot_constraints != 0:
        idx.y_lt = Indexer(end_lt+1, end_lt + p*L)
    return idx

def hankel_matrix(data, order):
    rows = order
    cols = len(data) - order + 1
    H = np.zeros((rows, cols))
    for i in range(rows):
        H[i, :] = np.squeeze(data[i:i + cols])
    return H

def update_ini(s, sini, d=1):
    sini[:-d] = sini[d:]
    sini[-d:] = s
    return sini

def update_data_matrix(idx, A, D_u, D_y, H, T, m, p, params):
    H_size = H.shape if isinstance(H, np.ndarray) else (1, 1)
    if H_size != (1, 1):
        A[idx.A_v.r, idx.a.r] = H
        return A

    L = params.L
    Tini = params.Tini
    H = np.zeros(((m + p) * (L + Tini), (T - L - Tini + 1)))
    H = combined_hankel_matrix(
        D_u, D_y, L + Tini, H)
    A[idx.A_v.r, idx.a.r] = H
    return A

def combined_hankel_matrix(D_u, D_y, L, H=None):
    if H is None:
        H = np.vstack((
            hankel_matrix(D_u, L),
            hankel_matrix(D_y, L)
        ))
    else:
        H[0:D_u.shape[1]*L, :] = hankel_matrix(D_u, L)
        H[D_u.shape[1]*L:, :] = hankel_matrix(D_y, L)
    return H


def create_optim_matrices(data, idx, params, T, m, p):

    L = params.L
    Tini = params.Tini
    Lc = params.Lc
    terminal_constraint_size = params.terminal_constraint_size
    affine_constraint = params.affine_constraint
    use_input_delta_constraints = params.use_input_delta_constraints
    use_input_terminal_constraints = params.use_input_terminal_constraints
    is_strict_terminal_constraint = params.is_strict_terminal_constraint
    use_overshoot_constraints = params.use_overshoot_constraints
    input_delta = params.input_delta

    data.uini = np.zeros((m * Tini, 1))
    data.yini = np.zeros((p * Tini, 1))
    data.A0 = np.zeros((idx.A_v.sz + idx.total_constraints.sz, idx.state.sz))
    data.b0 = np.zeros((idx.A_v.sz + idx.total_constraints.sz, 1))
    data.A0 = update_data_matrix(idx, data.A0, np.array(params.D_u).reshape(-1, m),
                                 np.array(params.D_y).reshape(-1, p), 0, T, m, p, params)

    data.A0[idx.u_v.r, idx.u.r] = -np.eye(m * L)
    data.A0[idx.y_v.r, idx.y.r] = -np.eye(p * L)
    data.A0[idx.yini_v.b:idx.y_v.e, idx.s.r] = -np.eye(p * (L + Tini))
    if terminal_constraint_size > 0:
        data.A0[idx.yterm_v.r, idx.y.e - p * terminal_constraint_size + 1:idx.y.e] = \
            np.eye(p * terminal_constraint_size)
        if use_input_terminal_constraints:
            data.A0[idx.uterm_v.r, idx.u.e - m * terminal_constraint_size + 1:idx.u.e] = \
                np.eye(m * terminal_constraint_size)
        if is_strict_terminal_constraint == 0:
            data.A0[idx.yterm_v.r, idx.yterm.r] = \
                np.eye(p * terminal_constraint_size)
            if use_input_terminal_constraints:
                data.A0[idx.uterm_v.r, idx.uterm.r] = \
                    np.eye(m * terminal_constraint_size)
    if affine_constraint > 0:
        data.A0[idx.affine_v.r, idx.a.r] = np.ones((1, idx.a.sz))
        data.b0[idx.affine_v.r] = 1.0
    data.A_lt = np.zeros((1, data.A0.shape[1]))
    data.b_lt = np.zeros((1, 1))
    if not (use_input_delta_constraints == 0):
        data.A_lt = np.zeros((idx.u_lt.sz, data.A0.shape[1]))
        data.b_lt = np.zeros((idx.u_lt.sz, 1))
        last_u = data.uini[-m:, :]
        data.A_lt[0:m, idx.u.b:idx.u.b + m - 1] = np.eye(m)
        data.A_lt[m:2 * m, idx.u.b:idx.u.b + m - 1] = -np.eye(m)
        data.b_lt[0:m, :] = last_u + input_delta
        data.b_lt[m:2 * m, :] = -last_u + input_delta
        for i in range(1, Lc):  # L-1 constraints
            s = 2 * m * i
            si = idx.u.b + m * i
            data.A_lt[s:s + m, si:si + m - 1] = np.eye(m)
            data.A_lt[s:s + m, si - m:si - 1] = -np.eye(m)
            data.A_lt[s + m:s + 2 * m, si:si + m - 1] = -np.eye(m)
            data.A_lt[s + m:s + 2 * m, si - m:si - 1] = np.eye(m)
            data.b_lt[s:s + m, :] = input_delta
            data.b_lt[s + m:s + 2 * m, :] = input_delta
    if not (use_overshoot_constraints == 0):
        data.A_lt = np.vstack((data.A_lt, np.zeros((idx.y_lt.sz, data.A0.shape[1]))))
        data.b_lt = np.vstack((data.b_lt, np.zeros((idx.y_lt.sz, 1))))
    data.optim_T = np.zeros((idx.state.sz, idx.state.sz))
    data.optim_f = np.zeros((idx.state.sz, 1))
    data.optim_T = set_optim_params(data.optim_T, data.A0, idx, params)
    data.A = data.A0
    return data

def set_optim_params(optim_T, A, idx, params):
    Tini = params.Tini
    p = idx.p
    is_strict_terminal_constraint = params.is_strict_terminal_constraint
    terminal_constraint_size = params.terminal_constraint_size
    use_input_terminal_constraints = params.use_input_terminal_constraints
    u_max = params.u_max
    y_max = params.y_max
    lambda_g = params.lambda_g
    lambda_s = np.array(params.lambda_s).reshape(-1, 1)
    lambda_s_ini = np.array(params.lambda_s_ini).reshape(-1, 1)
    lambda_term_y = np.array(params.lambda_term_y).reshape(-1, 1)
    lambda_term_u = np.array(params.lambda_term_u).reshape(-1, 1)
    use_projected_regularization = params.use_projected_regularization

    optim_T[idx.y.r, idx.y.r] = normalize_Q(params)
    optim_T[idx.u.r, idx.u.r] = normalize_R(params)

    if use_projected_regularization == 0:
        optim_T[idx.a.r, idx.a.r] = lambda_g * np.eye(idx.a.sz) / idx.a.sz
    else:
        a = A[idx.uini_v.b:idx.yini_v.e, idx.a.r]
        PI = np.linalg.pinv(a) @ a
        pp = (np.eye(idx.a.sz) - PI).T @ (np.eye(idx.a.sz) - PI)
        optim_T[idx.a.r, idx.a.r] = lambda_g / idx.a.sz * (pp.T + pp) / 2  # ensure symetric

    optim_T[idx.s.b:idx.s.b + p * Tini, idx.s.b:idx.s.b + p * Tini] = np.kron(np.eye(Tini), np.diag(lambda_s_ini / Tini))
    optim_T[idx.s.b + p * Tini:idx.s.e, idx.s.b + p * Tini:idx.s.e] = np.kron(np.eye(params.L), np.diag(lambda_s / idx.s.sz))

    if terminal_constraint_size > 0 and \
            is_strict_terminal_constraint == 0:
        optim_T[idx.yterm.r, idx.yterm.r] = \
            np.kron(np.eye(terminal_constraint_size), np.diag(lambda_term_y / (y_max ** 2)))
        if use_input_terminal_constraints:
            optim_T[idx.uterm.r, idx.uterm.r] = \
                np.kron(np.eye(terminal_constraint_size), np.diag(lambda_term_u / (u_max ** 2)))
    return optim_T

def update_matrices(y_ref, y, vel_stop_idx, data, idx, params):

    term = params.terminal_constraint_size
    use_input_delta_constraints = params.use_input_delta_constraints
    use_overshoot_constraints = params.use_overshoot_constraints
    is_strict_terminal_constraint = params.is_strict_terminal_constraint
    L = params.L
    m = idx.m
    p = idx.p
    end_point = params.end_point
    saturate = lambda val, vmin, vmax: np.minimum(np.maximum(val, vmin), vmax)

    yd = y[0:vel_stop_idx]

    if y_ref.ndim == 1:
        yrefd = np.tile(y_ref[0:vel_stop_idx], (L, 1)).T
    else:
        yrefd = y_ref[:, 0:vel_stop_idx]

    ic = yd
    term_v = yrefd[-1, :]
    rt = yrefd.T
    data.optim_f[idx.y.r] = 2 * normalize_Q(params) * -rt
    ref_u = saturate(yrefd / end_point.T, params.u_min, params.u_max)
    rt[:, :] = ref_u.T
    data.optim_f[idx.u.r] = 2 * normalize_R(params) * -rt

    last_u = data.uini[-m:, :]
    data.x0[idx.u.b:idx.u.e-m] = data.x0[idx.u.b+m:idx.u.e]
    data.x0[idx.y.b:idx.y.e-m] = data.x0[idx.y.b+m:idx.y.e]
    data.x0[idx.a.b:idx.a.e-m] = data.x0[idx.a.b+m:idx.a.e]
    data.x0[idx.s.r] = 0

    if term > 0:
        data.b[idx.yterm_v.r] = \
            np.full((term * p, 1), saturate(term_v, params.y_min, params.y_max))
        if not (params.use_input_terminal_constraints == 0):
            data.b[idx.uterm_v.r] = \
                np.full((term * m, 1), saturate(term_v / end_point, params.u_min, params.u_max))
        if is_strict_terminal_constraint == 0:
            data.x0[idx.yterm.r] = 0
            if not (params.use_input_terminal_constraints == 0):
                data.x0[idx.uterm.r] = 0

        # update input delta constraints
        if use_input_delta_constraints > 0:
            data.b_lt[0:m, :] = last_u + params.input_delta
            data.b_lt[m:2 * m, :] = -last_u + params.input_delta

        # update overshoot constraints
        if use_overshoot_constraints > 0:
            sgn = 1 if term_v - ic >= 0 else -1

            for i in range(idx.y_lt.b, idx.y_lt.e - 1):  # L-1 constraints
                yidx = i - idx.y_lt.b

                if params.get('pos_control', 0) == 1:
                    data.A_lt[i, idx.yp.b + yidx] = sgn
                    data.A_lt[i, idx.yp.b + yidx + 1] = -sgn
                else:
                    data.A_lt[i, idx.y.b + yidx] = sgn
                    data.A_lt[i, idx.y.b + yidx + 1] = -sgn

                data.b_lt[i, :] = 0
            data.A_lt[idx.y_lt.e, idx.y.e - p:idx.y.e] = sgn
            data.b_lt[idx.y_lt.e, :] = sgn * term_v


def normalize_R(params):
    r = np.array(params.R).reshape(-1, 1)
    if r.ndim > 1 and r.shape[1] > 1:
        R = np.kron(np.diag(params.decay ** (np.arange(1, params.L + 1)) / params.L),
                    r / (params.u_max * params.u_max))
    else:
        R = np.kron(np.diag(params.decay ** (np.arange(1, params.L + 1)) / params.L),
                    np.diag(r) / (params.u_max * params.u_max))
    R[params.Lc:, params.Lc:] = 0
    return R


def normalize_Q(params):
    q = np.array(params.Q).reshape(-1, 1)
    if q.ndim > 1 and q.shape[1] > 1:
        Q = np.kron(np.diag(params.decay ** (np.arange(1, params.L + 1)) / params.L),
                    q / (params.y_max * params.y_max))
    else:
        Q = np.kron(np.diag(params.decay ** (np.arange(1, params.L + 1)) / params.L),
                    np.diag(q) / (params.y_max * params.y_max))
    return Q


def check_param_dims(m, p, params):
    error_msg = []
    if params.Q.ndim == 1 and params.Q.shape[0] != p:
        error_msg.append("""Matrix Q should either be a {}-dimensional column
vector of diagonal elements of Q or a {}x{} matrix.""".format(p, p, p))
    elif params.Q.ndim == 2 and params.Q.shape != (p, p):
        error_msg.append("""Matrix Q should either be a {}-dimensional column
vector of diagonal elements of Q or a {}x{} matrix.""".format(p, p, p))

    if params.R.ndim == 1 and params.R.shape[0] != m:
        error_msg.append("""Matrix R should either be a {}-dimensional column
vector of diagonal elements of R or a {}x{} matrix.""".format(m, m, m))
    elif params.R.ndim == 2 and params.R.shape != (m, m):
        error_msg.append("""Matrix R should either be a {}-dimensional column
vector of diagonal elements of R or a {}x{} matrix.""".format(m, m, m))

    if params.u_max.ndim == 1 and params.u_max.shape[0] != m:
        error_msg.append("""Parameter u_max must have dimension {}.""".format(m))
    elif params.u_max.ndim == 2 and params.u_max.shape != (m, 1):
        error_msg.append("""Parameter u_max must have dimension {}.""".format(m))

    if params.u_min.ndim == 1 and params.u_min.shape[0] != m:
        error_msg.append("""Parameter u_min must have dimension {}.""".format(m))
    elif params.u_min.ndim == 2 and params.u_min.shape != (m, 1):
        error_msg.append("""Parameter u_min must have dimension {}.""".format(m))

    if params.y_max.ndim == 1 and params.y_max.shape[0] != p:
        error_msg.append("""Parameter y_max must have dimension {}.""".format(p))
    elif params.y_max.ndim == 2 and params.y_max.shape != (p, 1):
        error_msg.append("""Parameter y_max must have dimension {}.""".format(p))

    if params.y_min.ndim == 1 and params.y_min.shape[0] != p:
        error_msg.append("""Parameter y_min must have dimension {}.""".format(p))
    elif params.y_min.ndim == 2 and params.y_min.shape != (p, 1):
        error_msg.append("""Parameter y_min must have dimension {}.""".format(p))

    if params.lambda_s.ndim == 1 and params.lambda_s.shape[0] != p:
        error_msg.append("""Parameter lambda_s must have dimension {}.""".format(p))
    elif params.lambda_s.ndim == 2 and params.lambda_s.shape != (p, 1):
        error_msg.append("""Parameter lambda_s must have dimension {}.""".format(p))

    if params.lambda_s_ini.ndim == 1 and params.lambda_s_ini.shape[0] != p:
        error_msg.append("""Parameter lambda_s_ini must have dimension {}.""".format(p))
    elif params.lambda_s_ini.ndim == 2 and params.lambda_s_ini.shape != (p, 1):
        error_msg.append("""Parameter lambda_s_ini must have dimension {}.""".format(p))

    if params.use_input_delta_constraints != 0:
        if params.input_delta.ndim == 1 and params.input_delta.shape[0] != m:
            error_msg.append("""Parameter input_delta must have dimension {}.""".format(m))
        elif params.input_delta.ndim == 2 and params.input_delta.shape != (m, 1):
            error_msg.append("""Parameter input_delta must have dimension {}.""".format(m))

    if params.terminal_constraint_size > 0 and \
            params.is_strict_terminal_constraint == 0:
        if params.lambda_term_y.ndim == 1 and params.lambda_term_y.shape[0] != p:
            error_msg.append("""Parameter lambda_term_y must have dimension {}.""".format(p))
        elif params.lambda_term_y.ndim == 2 and params.lambda_term_y.shape != (p, 1):
            error_msg.append("""Parameter lambda_term_y must have dimension {}.""".format(p))
        if params.use_input_delta_constraints:
            if params.lambda_term_u.ndim == 1 and params.lambda_term_u.shape[0] != m:
                error_msg.append("""Parameter lambda_term_u must have dimension {}.""".format(m))
            elif params.lambda_term_u.ndim == 2 and params.lambda_term_u.shape != (m, 1):
                error_msg.append("""Parameter lambda_term_u must have dimension {}.""".format(m))

    if error_msg:
        raise ValueError("\n".join(error_msg))
