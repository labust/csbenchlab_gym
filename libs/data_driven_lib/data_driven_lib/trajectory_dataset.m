function [db_u, db_y, db_static_gain, db_num_datasets, changed] = trajectory_dataset(u_traj, y_traj, ...
    db_u, db_y, db_static_gain, db_num_datasets, n, L, Tini, Ts, db_params)

    changed = 0;
    m = size(u_traj, 2);
    p = size(y_traj, 2);
    hankel_order = m*(L+Tini);
    % required_order = m*(L+Tini);
    required_order = L+Tini;
    if is_traj_ok(u_traj, y_traj, hankel_order, required_order, db_params.sigma_min, db_params.dataset_delta_y_min)
        
        if db_num_datasets == db_params.dataset_bank_size
            error('dataset overflow')
        end

        [~, u, y_m] = linearize_hankel(u_traj, y_traj, L, Tini, hankel_order, db_params.opts);
        

        [static_gain, cond] = calculate_static_gain_for_trajectory(u, y_m);
        
        if cond > 1e-3
            return
        end

        db_u(end-db_num_datasets, :, :) = u;
        db_y(end-db_num_datasets, :, :) = y_m;
        db_static_gain(end-db_num_datasets, :) = static_gain';  
        changed = 1;
        db_num_datasets = db_num_datasets + 1;
    end
end