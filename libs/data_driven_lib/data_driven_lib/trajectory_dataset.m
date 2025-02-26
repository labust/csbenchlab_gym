function [db_u, db_y, db_endpoint, db_num_datasets, changed] = trajectory_dataset(u_traj, y_traj, db_u, db_y, db_endpoint, db_num_datasets, hankel_order, Ts, params)

    changed = 0;
    if is_traj_ok(u_traj, y_traj, hankel_order, params.sigma_min, params.dataset_delta_y_min)
        [m, ic] = arx(u_traj, y_traj, [params.fit.na params.fit.nb params.fit.nk], 'Ts', Ts);
        t = (0:1:length(u_traj)-1) * Ts;
        as_ss = idss(m);
        y_m = lsim(as_ss, u_traj, t, ic.X0);
        db_u(end-db_num_datasets, :, :) = u_traj;
        db_y(end-db_num_datasets, :, :) = y_m;
        db_endpoint(end-db_num_datasets, :) = sum(m.B) / sum(m.A);
        changed = 1;

        db_num_datasets = db_num_datasets + 1;
        if db_num_datasets == params.dataset_bank_size + 1
            error('dataset overflow')
        end
    end
end