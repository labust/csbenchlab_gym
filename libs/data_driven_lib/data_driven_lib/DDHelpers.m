classdef DDHelpers

    
    methods (Static)
        
        function H = hankel_matrix(data, L)
            T = size(data, 1);
            m = size(data, 2);
            
            H = zeros(L*m, T-L+1);
            for i = 0:L-1
                for j = 1:T-L+1
                    H(i*m+1:i*m+m, j) = data(i+j, :)';
                end
            end
        end

        function H = combined_hankel_matrix(D_u, D_y, H, L)
            hankel_matrix = @DDHelpers.hankel_matrix;
            m = size(D_u, 2);
            H(1:m*L, :) = hankel_matrix(D_u, L);
            H(m*L+1:end, :) = hankel_matrix(D_y, L);
        end

        function [u_t, y_t] = compose_trajectory_with_prediction(u_traj, y_traj, x_op, params)

            if params.traj_mode == 0
                u_t = u_traj;
                y_t = y_traj;
                return
            end
            
            N = params.N;
            idx = params.idx;
            u_p = x_op(idx.u.b:idx.u.e);
            y_p = x_op(idx.y.b:idx.y.e);
            if params.traj_mode == 1
                u_t = circshift(u_traj, -N);
                y_t = circshift(y_traj, -N);
                u_t(end-N+1:end) = u_p;
                y_t(end-N+1:end) = y_p;
                
            end
        end

        function r = sigma_rank(H, sigma_min)
            [x, y] = size(H);
            [~, S, ~] = svd(H);
            s_values = S(1:x+1:x*y);
            r = sum(s_values > sigma_min);
        end

        function r = sigma_min(H)
            [x, y] = size(H);
            [~, S, ~] = svd(H);
            s_values = S(1:x+1:x*y);
            r = min(s_values(1:x));
        end

    end
end

