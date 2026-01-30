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

        function H = combined_hankel_matrix(D_u, D_y, L, H)
            
            hankel_matrix = @DDHelpers.hankel_matrix;
            m = size(D_u, 2);
            H_u = hankel_matrix(D_u, L);
            H_y = hankel_matrix(D_y, L);

            if ~exist('H', 'var')
                H = zeros(height(H_u) + height(H_y), width(H_u));
            end
            H(1:m*L, :) = H_u;
            H(m*L+1:end, :) = H_y;
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

