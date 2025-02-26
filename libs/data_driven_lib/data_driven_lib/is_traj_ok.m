function ok = is_traj_ok(u_traj, y_traj, hankel_order, sigma_min, delta_y_min)
    ok = 0;

    m = size(u_traj, 2);
    % H = [DDHelpers.hankel_matrix(u_traj, hankel_order); DDHelpers.hankel_matrix(y_traj, hankel_order)];
    H = DDHelpers.hankel_matrix(u_traj, hankel_order);
    min_y = min(y_traj);
    max_y = max(y_traj);

    if ~(max_y - min_y > delta_y_min)
        return
    end
    
    if DDHelpers.sigma_rank(H, sigma_min) < hankel_order
        return
    end

    ok = 1;
end