function [u_t, y_t] = compose_trajectory_with_prediction(u_traj, y_traj, x_op_u, x_op_y, traj_mode)
%COMPOSE_TRAJ Summary of this function goes here
%   Detailed explanation goes here
    m = size(u_traj, 2);
    L = size(x_op_u, 1) / m;
    if traj_mode == 1
        u_t = circshift(u_traj, -L);
        y_t = circshift(y_traj, -L);
        u_t(end-L+1:end, :) = reshape(x_op_u, L, m);
        y_t(end-L+1:end, :) = reshape(x_op_y, L, m);
        return
    end

    u_t = u_traj;
    y_t = y_traj;
     

end

