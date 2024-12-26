function [t, last_changed_t, db_u_p, db_y_p, db_endpoint_p, db_num_datasets_p, ...
          db_u, db_y, db_endpoint, db_num_datasets] = update_dataset_bank( ...
            t, last_changed_t, db_u_p, db_y_p, db_endpoint_p, db_num_datasets_p, ...
            dt, u_traj, y_traj, db_params, data)
    
    coder.extrinsic('trajectory_dataset'); % arx function not supported
    t = t + dt;

    traj_len = data.T;
    traj_len_s = (traj_len + 1) * data.Ts;

    if ~db_params.enable_online_datasets || (t - last_changed_t) < traj_len_s
        db_u = db_u_p;
        db_y = db_y_p;
        db_endpoint = db_endpoint_p;
        db_num_datasets = db_num_datasets_p;
        return
    end

    changed = 0;
    [db_u_p, db_y_p, db_endpoint_p, db_num_datasets_p, changed] = ...
        trajectory_dataset(u_traj, y_traj, db_u_p, db_y_p, db_endpoint_p, db_num_datasets_p, data.n + data.L + data.Tini, data.Ts, db_params);

    db_u = db_u_p;
    db_y = db_y_p;
    db_endpoint = db_endpoint_p;
    db_num_datasets = db_num_datasets_p;   

    if changed
        last_changed_t = t;  
    end
end