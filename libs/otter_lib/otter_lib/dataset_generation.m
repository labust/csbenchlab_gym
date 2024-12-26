function [changed, out_idx] = dataset_generation(t, y, params)

    coder.extrinsic('trajectory_dataset'); % arx function not supported
    idx = params.idx;
    u_traj = circshift(u_traj, -1);
    y_traj = circshift(y_traj, -1);
    u = x_op_old(idx.u.b);
    u_traj(end) = u;
    y_traj(end) = y(2);

    traj_len_s = 0.8*(params.traj_len * params.Ts);
    changed = 0;
    out_idx = active_dataset_idx;
    if ~params.enable_online_datasets || (t - last_changed_t) < traj_len_s  || t > 15 % temp
        return
    end
    [Db_u, Db_y, num_datasets_in_bank, changed] = trajectory_dataset(u_traj, y_traj, Db_u, Db_y, num_datasets_in_bank, params);
    if changed
        last_changed_t = t;  
    end

    if ~ (params.dataset_mode == 2)
        if ~changed
            return
        end
    end

    [u_t, y_t] = compose_trajectory_with_prediction(u_traj, y_traj, x_op_old, params);
    [D_u, D_y, active_dataset_idx] = select_current_datasets(Db_u, Db_y, D_u, D_y, u_t, y_t, num_datasets_in_bank, active_dataset_idx, params);
    out_idx = active_dataset_idx;


y.D_u = zeros(1);
y.D_y = zeros(1);

end