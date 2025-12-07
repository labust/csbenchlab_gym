classdef DeePCHelpers
    %DEEPC_HELPERS Summary of this class goes here
    %   Detailed explanation goes here

    methods (Static)

        function [T, m, p] = get_traj_info(D_u, D_y)
            m = size(D_u, 2);
            p = size(D_y, 2);
            T = size(D_u, 1);
        end

        function sini = update_ini(s, sini, d)
            sini = circshift(sini, -d);
            sini(end-d+1:end) = s;
        end

        function [lb, ub] = configure_bounds(lb, ub, idx, params)
            term = params.terminal_constraint_size;
            term_off = params.allowed_offset_terminal;

            lb(idx.u.r) = repmat(params.u_min, params.L, 1);
            ub(idx.u.r) = repmat(params.u_max, params.L, 1);
            lb(idx.y.r) = repmat(params.y_min, params.L, 1);
            ub(idx.y.r) = repmat(params.y_max, params.L, 1);
            slack_off = params.allowed_offset_slack;
            if slack_off == 0
                slack_off = 1;
            end

            lb(idx.s.b:idx.s.e) = repmat(params.y_min * slack_off, params.L + params.Tini, 1);
            ub(idx.s.b:idx.s.e) = repmat(params.y_max * slack_off, params.L + params.Tini, 1);

            if term > 0 && params.is_strict_terminal_constraint == 0
                lb(idx.yterm.r) = repmat(params.y_min * term_off, params.terminal_constraint_size, 1);
                ub(idx.yterm.r) = repmat(params.y_max * term_off, params.terminal_constraint_size, 1);
                % since model is nonlinear, this is bad assumption
                % if ~ (params.use_input_terminal_constraints == 0)
                %     lb(idx.uterm.r) = params.u_min * term_off;
                %     ub(idx.uterm.r) = params.u_max * term_off;
                % end
            end
        end


        function data = create_basic_data_model(params, mux)
            data.Ts = params.Ts;
            [data.T, data.m, data.p] = ...
                DeePCHelpers.get_traj_info(params.D_u, params.D_y);
            data.L = params.L;
            data.n = params.n;
            data.Tini = params.Tini;

            DeePCHelpers.check_param_dims(data.m, data.p, params)


            if params.pos_control == 1
                data.vel_stop_idx = length(mux.Inputs) / 2;
            else
                data.vel_stop_idx = length(mux.Inputs);
            end

            data.idx = DeePCHelpers.set_param_indices_and_dims(params, data.T, data.m, data.p);
            data.x_op_u = zeros(data.m * params.L, 1);
            data.x_op_y = zeros(data.p * params.L, 1);
            data.x_op_g = zeros(data.T - params.L - params.Tini + 1, 1);

            [data.uini, data.yini, data.A, data.b, data.A_lt, data.b_lt, data.optim_T, data.optim_f] = ...
                DeePCHelpers.create_optim_matrices(data.idx, params, data.T, data.m, data.p);

            data.x_op = zeros(data.idx.state.sz, 1);
            data.lb = -ones(height(data.x_op), 1) * inf;
            data.ub = ones(height(data.x_op), 1) * inf;
            data.fval = 0;

            data.old_y_ref = 0;
            data.eta = zeros(data.p);
            data.has_lt = 1;
            if size(data.b_lt, 1) == 1 && data.b_lt == 0
                data.has_lt = 0;
            end
        end

        function idx = set_param_indices_and_dims(params, T, m, p)
            Tini = params.Tini;
            L = params.L; Lc = params.Lc;
            pos_control = params.pos_control;

            affine_constraint = params.affine_constraint;
            terminal_constraint_size = params.terminal_constraint_size;
            use_input_terminal_constraints = params.use_input_terminal_constraints;
            is_strict_terminal_constraint = params.is_strict_terminal_constraint;
            use_input_delta_constraints = params.use_input_delta_constraints;
            use_overshoot_constraints = params.use_overshoot_constraints;
            idx.m  = m;
            idx.p = p;
            
            % ROWS DIMENSIONS
            idx.uini_v = Indexer(1, m*Tini);
            idx.u_v = Indexer(m*Tini+1, m*(Tini + L));
            
            idx.yini_v = Indexer(m*(Tini + L) + 1, m*(Tini + L) + p*Tini);
            idx.y_v = Indexer(m*(Tini + L) + p * Tini + 1, (m + p) * (L + Tini));
            
            % COLS DIMENSIONS
            idx.u = Indexer(1, m*L);
            idx.y = Indexer(m*L+1, (m + p)*L);
            
            if pos_control == 1
                idx.yp = Indexer((m + p)*L + 1, (m + 2*p) * L);
                pos_add = L;
            else
                % not used
                idx.yp = Indexer(-1, -1);
                pos_add = 0;
            end
            
            dim_a = T-L-Tini+1;
            curr_state_dim = (m + p) * L + pos_add;
            curr_v_dim = (m + p) * (Tini + L);
                        
            idx.s = Indexer(curr_state_dim+1, curr_state_dim + p*(L+Tini));
            curr_state_dim = curr_state_dim + p * (L + Tini);
       
            idx.A_v = Indexer(1, curr_v_dim);
            
            term_v_b = curr_v_dim + 1;


            idx.yterm = Indexer(-1, -1);
            idx.uterm = Indexer(-1, -1);
            idx.uterm_v = Indexer(-1, -1);
            idx.yterm_v = Indexer(-1, -1);
            idx.affine_v = Indexer(-1, -1);
            

            affine_constraint = affine_constraint > 0;

            if affine_constraint > 0
                idx.affine_v = Indexer(curr_v_dim + 1, curr_v_dim + 1);
                curr_v_dim = curr_v_dim + 1;
            end
            
            total_constraints = affine_constraint;
            if terminal_constraint_size > 0

                idx.yterm_v = Indexer(curr_v_dim + 1, curr_v_dim + p * terminal_constraint_size);
                curr_v_dim = curr_v_dim + p * terminal_constraint_size;
                if ~ (use_input_terminal_constraints == 0)
                    idx.uterm_v = Indexer(curr_v_dim + 1, curr_v_dim + m *terminal_constraint_size);
                    curr_v_dim = curr_v_dim + m *terminal_constraint_size;
                else
                    idx.uterm_v = Indexer(-1, 1);
                end
            
                terminal_constraints = ...
                    p * terminal_constraint_size + ...
                    use_input_terminal_constraints * m * terminal_constraint_size;
            
                if is_strict_terminal_constraint == 0
                    idx.yterm = Indexer(curr_state_dim+1, curr_state_dim + p *terminal_constraint_size);
                    if ~ (use_input_terminal_constraints == 0)
                        idx.uterm = Indexer(curr_state_dim + p * terminal_constraint_size + 1, ...
                            curr_state_dim + (m + p) * terminal_constraint_size);
                        curr_state_dim = curr_state_dim + terminal_constraints;
                    end                    
                end
                total_constraints = total_constraints + terminal_constraints;
            end

            idx.total_constraints = total_constraints;
            idx.term_v = Indexer(term_v_b, curr_v_dim);
            

            idx.a = Indexer(curr_state_dim + 1, curr_state_dim + dim_a);
            curr_state_dim = curr_state_dim + dim_a;
            
            if pos_control == 1
                idx.yp_v = Indexer(curr_v_dim + 1, curr_v_dim + pos_add);
            else
                idx.yp_v = Indexer(-1, -1);
            end

            idx.state = Indexer(1, curr_state_dim);

            % lt matrices
            idx.u_lt = Indexer(-1, -1);
            idx.y_lt = Indexer(-1, -1);
            end_lt = 0;
            if ~(use_input_delta_constraints == 0)
                idx.u_lt = Indexer(1, 2*m*Lc);
                end_lt = idx.u_lt.e;
            end

            if ~(use_overshoot_constraints == 0)
                idx.y_lt = Indexer(end_lt+1, end_lt + p*L);
                % end_lt = idx.y_lt.e;
            end

        end

        function A = update_data_matrix(idx, A, D_u, D_y, T, m, p, params)
            L = params.L; 
            Tini = params.Tini;
            H = zeros((m+p)*(L+Tini), (T-L-Tini+1));
            H = DDHelpers.combined_hankel_matrix(...
                D_u, D_y, H, L+Tini);
            A(idx.A_v.r, idx.a.r) = H;
        end
        
       
        function [uini, yini, A0, b0, A_lt, b_lt, optim_T, optim_f] ...
                = create_optim_matrices(idx, params, T, m, p)
            
            L = params.L; Tini = params.Tini; Lc = params.Lc;
            terminal_constraint_size = params.terminal_constraint_size;
            affine_constraint = params.affine_constraint;
            use_input_delta_constraints = params.use_input_delta_constraints;
            use_input_terminal_constraints = params.use_input_terminal_constraints;
            is_strict_terminal_constraint = params.is_strict_terminal_constraint;
            use_overshoot_constraints = params.use_overshoot_constraints;
            input_delta = params.input_delta;

            
            
            uini = zeros(m * Tini, 1);
            yini = zeros(p * Tini, 1);

            A0 = zeros([idx.A_v.sz + idx.total_constraints, idx.state.sz]);
            b0 = zeros(idx.A_v.sz + idx.total_constraints, 1);
            
            A0 = DeePCHelpers.update_data_matrix(idx, A0, params.D_u, params.D_y, ...
                T, m, p, params);
            
            A0(idx.u_v.r, idx.u.r) = -eye(m * L);
            A0(idx.y_v.r, idx.y.r) = -eye(p * L);
            
            A0(idx.yini_v.b : idx.y_v.e, idx.s.r) = -eye(p * (L+Tini));
            
            if terminal_constraint_size > 0 
                A0(idx.yterm_v.r, idx.y.e - p * terminal_constraint_size + 1:idx.y.e) ...
                    = eye(p*terminal_constraint_size);
                if use_input_terminal_constraints
                    A0(idx.uterm_v.r, idx.u.e - m * terminal_constraint_size + 1:idx.u.e) ... 
                        = eye(m*terminal_constraint_size);
                end
            
                if is_strict_terminal_constraint == 0
                    A0(idx.yterm_v.r, idx.yterm.r) ...
                        = eye(p * terminal_constraint_size);
                        if use_input_terminal_constraints
                            A0(idx.uterm_v.r, idx.uterm.r) ... 
                                = eye(m * terminal_constraint_size);
                        end
                end
            end

            if affine_constraint > 0
                A0(idx.affine_v.r, idx.a.r) = ones(1, idx.a.sz);
                b0(idx.affine_v.r) = ones(idx.affine_v.sz, 1);
            end    
            
            A_lt = zeros(1, width(A0)); b_lt = 0;
            if ~(use_input_delta_constraints == 0)
                A_lt = zeros(idx.u_lt.sz, width(A0));
                b_lt = zeros(idx.u_lt.sz, width(b0));
                last_u = uini(end-m+1:end);  

                A_lt(1:m, idx.u.b:idx.u.b+m-1) = eye(m);
                A_lt(m+1:2*m, idx.u.b:idx.u.b+m-1) = -eye(m);
                b_lt(1:m, :) = last_u + input_delta;
                b_lt(m+1:2*m, :) = -last_u + input_delta;
                

                
                for i=1:Lc-1 % L-1 constraints
                    s = 2*m*i+1;
                    si = idx.u.b + m*i;
                    A_lt(s:s+m-1, si:si+m-1) = eye(m);
                    A_lt(s:s+m-1, si-m:si-1) = -eye(m);
                    A_lt(s+m:s+2*m-1, si:si+m-1) = -eye(m);
                    A_lt(s+m:s+2*m-1, si-m:si-1) = eye(m);
                    b_lt(s:s+m-1, :) = input_delta;
                    b_lt(s+m:s+2*m-1, :) = input_delta;
               end
            end

            if ~(use_overshoot_constraints == 0)
                A_lt = [A_lt; zeros(idx.y_lt.sz, width(A0))];
                b_lt = [b_lt; zeros(idx.y_lt.sz, width(b0))];
            end

            if params.pos_control == 1
                Ap = -eye(idx.yp.sz);
                Ay = ones(idx.yp.sz);
                Ay = tril(Ay) * params.Ts;
                
                A0_prev = A0;
                b0_prev = b0;
                A0 = zeros(height(A0) + idx.yp.sz, width(A0));
                b0 = zeros(height(b0) + idx.yp.sz, 1);
            
                b0(1:idx.yp_v.b-1, 1) = b0_prev;
                A0(1:idx.yp_v.b-1, :) = A0_prev;
                A0(idx.yp_v.b:idx.yp_v.e, idx.y.b:idx.y.e) = Ay;
                A0(idx.yp_v.b:idx.yp_v.e, idx.yp.b:idx.yp.e) = Ap;
            end
         
            optim_T = zeros(idx.state.sz);
            optim_f = zeros(idx.state.sz, 1);
            optim_T = DeePCHelpers.set_optim_params(optim_T, A0, idx, params);
        end


        function optim_T = set_optim_params(optim_T, A, idx, params)
            
            Tini = params.Tini;
            p = idx.p;
            is_strict_terminal_constraint = params.is_strict_terminal_constraint;
            terminal_constraint_size = params.terminal_constraint_size;
            use_input_terminal_constraints = params.use_input_terminal_constraints;
            u_max = params.u_max;
            y_max = params.y_max;
            lambda_a = params.lambda_a;
            lambda_s = params.lambda_s;
            lambda_s_ini = params.lambda_s_ini;
            lambda_term_y = params.lambda_term_y;
            lambda_term_u = params.lambda_term_u;
            use_projected_regularization = params.use_projected_regularization;

            if params.pos_control == 1
                optim_T(idx.yp.r, idx.yp.r) = DeePCHelpers.normalize_Q(params);
            else 
                optim_T(idx.y.r, idx.y.r) = DeePCHelpers.normalize_Q(params);
            end
            optim_T(idx.u.r, idx.u.r) = DeePCHelpers.normalize_R(params);
            if ~use_projected_regularization
                optim_T(idx.a.r, idx.a.r) = lambda_a * eye(idx.a.sz) / idx.a.sz;
            else
                a = A(idx.uini_v.b:idx.yini_v.e, idx.a.r);
                PI = pinv(a) * a;
                pp = (eye(idx.a.sz) - PI)' * (eye(idx.a.sz) - PI);
                optim_T(idx.a.r, idx.a.r) = lambda_a / idx.a.sz * (pp' + pp) / 2; % ensure symetric
            end
            optim_T(idx.s.b:idx.s.b+p*Tini-1, idx.s.b:idx.s.b+p*Tini-1) = kron(eye(Tini), diag(lambda_s_ini ./ Tini));
            optim_T(idx.s.b+p*Tini:idx.s.e, idx.s.b+p*Tini:idx.s.e) = kron(eye(params.L), diag(lambda_s ./ idx.s.sz));
  
            if terminal_constraint_size > 0 && ...
                    is_strict_terminal_constraint == 0
                optim_T(idx.yterm.r, idx.yterm.r) = ...
                    kron(eye(terminal_constraint_size), diag(lambda_term_y ./ (y_max.^2)));
                if use_input_terminal_constraints
                    optim_T(idx.uterm.r, idx.uterm.r) = ...
                        kron(eye(terminal_constraint_size), diag(lambda_term_u ./ (u_max.^2)));
                end
            end
        end

        function [b, A_lt, b_lt, optim_f, x0] = update_matrices(y_ref, y, vel_stop_idx, end_point, b, A_lt, b_lt, optim_f, x0, idx, params)

            term = params.terminal_constraint_size;
            use_input_delta_constraints = params.use_input_delta_constraints;
            use_overshoot_constraints = params.use_overshoot_constraints;
            L = params.L;
            m = idx.m;

            saturate = @Utils.saturate; 

            yd = y(1:vel_stop_idx);

            if size(y_ref, 1) == 1
                yrefd = repmat(y_ref(1, 1:vel_stop_idx), 1, L)';
            else
                yrefd = y_ref(:, 1:vel_stop_idx);
            end

            if params.pos_control == 1
                yp = y(vel_stop_idx+1:end);
                if size(y_ref, 1) == 1
                    yrefp = repmat(y_ref(1, vel_stop_idx+1:end), L, 1);
                else
                    yrefp = y_ref(:, vel_stop_idx+1:end);
                end
                ref = yrefp;
                ref_v = saturate(params.k' .* (yrefp - yp), params.y_min, params.y_max);       
                term_v = ref_v(end, :);

                ic = yp;
                b(idx.yp_v.r, :) = -repmat(yp, idx.yp_v.sz, 1);

                rt = ref';
                optim_f(idx.yp.r) = DeePCHelpers.normalize_Q(params) * (- rt(:));
                ref_u = saturate(ref_v ./ end_point', params.u_min, params.u_max);
                rt(:, :) = ref_u';
                optim_f(idx.u.r) = DeePCHelpers.normalize_R(params) * (- rt(:));
            else
                ref = yrefd;
                ic = yd;
                term_v = ref(end, :);                
                rt = ref';
                optim_f(idx.y.r) = 2 * DeePCHelpers.normalize_Q(params) * (- rt(:));
                ref_u = saturate(ref ./ end_point', params.u_min, params.u_max);
                rt(:, :) = ref_u';
                optim_f(idx.u.r) = 2 * DeePCHelpers.normalize_R(params) * (- rt(:));
            end

            last_u = x0(idx.u.b:idx.u.b+idx.m-1);
          
            x0(idx.u.b:idx.u.e-m) = x0(idx.u.b+m:idx.u.e);
            x0(idx.y.b:idx.y.e-m) = x0(idx.y.b+m:idx.y.e);
            x0(idx.a.b:idx.a.e-m) = x0(idx.a.b+m:idx.a.e);
            x0(idx.s.r) = 0;
            
            % x0(idx.a.r) = rand(idx.a.sz, 1);
            % x0 = rand(size(x0, 1), 1);
            

            
            % update teminal constraint
            if term > 0
                b(idx.yterm_v.r) = ...
                    repmat(saturate(term_v, params.y_min, params.y_max), 1, term)';
                if ~ (params.use_input_terminal_constraints == 0)
                    b(idx.uterm_v.r) = ...
                        saturate(b(idx.yterm_v.r) ./ repmat(end_point, term, 1), params.u_min, params.u_max);
                end

                if params.is_strict_terminal_constraint == 0
                    x0(idx.yterm.r) = 0;
                    if ~ (params.use_input_terminal_constraints == 0)
                        x0(idx.uterm.r) = 0;
                    end
                end
            end

            % update input delta constraints
            if use_input_delta_constraints > 0
                b_lt(1:m, :) = last_u + params.input_delta;
                b_lt(m+1:2*m, :) = -last_u + params.input_delta;
            end

            % update overshoot constraints
            if use_overshoot_constraints > 0
                if ref(end, :) - ic >= 0
                    sgn = 1;
                else
                    sgn = -1;
                end

                for i = idx.y_lt.b : idx.y_lt.e -1 % L-1 constraints
                    yidx = i - idx.y_lt.b;
                    
                    if params.pos_control == 1
                        A_lt(i, idx.yp.b + yidx + 1) = -sgn;
                        A_lt(i, idx.yp.b + yidx) = sgn;
                    else
                        A_lt(i, idx.y.b + yidx + 1) = -sgn;
                        A_lt(i, idx.y.b + yidx) = sgn;
                    end


                    b_lt(i, :) = 0;
                end
                A_lt(idx.y_lt.e, idx.y.e) = sgn;
                b_lt(idx.y_lt.e, :) = sgn * ref(end, :);

            end

        end

        function R = normalize_R(params)
            if size(params.R, 2) > 1
                R = kron(diag(params.decay.^(1:params.L) / params.L), ...
                    params.R ./ (params.u_max * params.u_max'));
            else
                R = kron(diag(params.decay.^(1:params.L) / params.L), ...
                    diag(params.R) ./ (params.u_max * params.u_max'));
            end
            R(params.Lc+1:end, params.Lc+1:end) = 0;
        end

        function Q = normalize_Q(params)
            if size(params.Q, 2) > 1
                Q = kron(diag(params.decay.^(1:params.L) / params.L), ...
                    params.Q ./ (params.y_max * params.y_max'));
            else
                Q = kron(diag(params.decay.^(1:params.L) / params.L), ...
                    diag(params.Q) ./ (params.y_max * params.y_max'));
            end
        end

        function [x_op, fval, exit_flag] = optim(optim_T, optim_f, A, b, lb, ub, x0)
            % o = optimoptions('quadprog','Algorithm','interior-point-convex', 'Display','off');            
            o = optimoptions('quadprog','Algorithm','active-set', 'Display','off');  
            [x_op, fval, exit_flag] = quadprog(optim_T, optim_f, [], [], A, b, lb, ub, x0, o);
        end


        function [x_op, fval, exit_flag] = optim_lt(optim_T, optim_f, A, b, A_lt, b_lt, lb, ub, x0)
            o = optimoptions('quadprog','Algorithm','active-set', 'Display','off', 'ConstraintTolerance', 1e-6);      
            [x_op, fval, exit_flag] = quadprog(optim_T, optim_f, A_lt, b_lt, A, b, lb, ub, x0, o);
        end


        function check_param_dims(m, p, params)

            error_msg = {};

            if size(params.Q, 1) ~= p ...
                    && ~isequal(size(params.Q), [p, p])
                error_msg{end+1} = ['Matrix Q should either be a ' ...
                    num2str(p), '-dimensional column vector of diagonal' ...
                    ' elements of Q or a ', ...
                    num2str(p) 'x' num2str(p),  ' matrix.', newline];
            end

            if size(params.R, 1) ~= m ...
                    && ~isequal(size(params.Q), [m, m])
                error_msg{end+1} = ['Matrix Q should either be a ' ...
                    num2str(m), '-dimensional column vector of diagonal' ...
                    ' elements of Q or a ', ...
                    num2str(m) 'x' num2str(m),  ' matrix.', newline];
            end

            if size(params.u_max, 1) ~= m
                error_msg{end+1} = ['Parameter u_max must have dimension ', num2str(m), '.', newline];
            end

            if size(params.u_min, 1) ~= m
                error_msg{end+1} = ['Parameter u_min must have dimension ', num2str(m), '.', newline];
            end

            if size(params.y_max, 1) ~= p
                error_msg{end+1} = ['Parameter y_max must have dimension ', num2str(p), '.', newline];
            end

            if size(params.y_min, 1) ~= p
                error_msg{end+1} = ['Parameter y_min must have dimension ', num2str(p), '.', newline];
            end

            if size(params.lambda_s, 1) ~= p
                error_msg{end+1} = ['Parameter lambda_s must have dimension ', num2str(p), '.', newline];
            end

            if size(params.lambda_s_ini, 1) ~= p
                error_msg{end+1} = ['Parameter lambda_s_ini must have dimension ', num2str(p), '.', newline];
            end
            
            if params.use_input_delta_constraints
                if size(params.input_delta, 1) ~= m
                    error_msg{end+1} = ['Parameter input_delta must have dimension ', num2str(m), '.', newline];
                end
            end
            
            if params.terminal_constraint_size > 0 && ...
                    ~params.is_strict_terminal_constraint
                if size(params.lambda_term_y, 1) ~= p
                    error_msg{end+1} = ['Parameter lambda_term_y must have dimension ', num2str(p), '.', newline];
                end
                if params.use_input_delta_constraints
                    if size(params.lambda_term_u, 1) ~= m
                        error_msg{end+1} = ['Parameter lambda_term_u must have dimension ', num2str(m), '.', newline];
                    end
                end
            end

            if ~isempty(error_msg)
                error(strcat(error_msg{:}));
            end
        end
        
        function p = get_deepc_param_set(override_params)
            p = {
                ParamDescriptor("L", 1), ...
                ParamDescriptor("Tini", 1), ...
                ParamDescriptor("Ts", 1), ...
                ParamDescriptor("pos_control", 0), ...
                ParamDescriptor("n", 1), ...
                ParamDescriptor("D_u", 0), ...
                ParamDescriptor("D_y", 0), ...
                ParamDescriptor("end_point", 0), ...
                ParamDescriptor("T", @(params) length(params.D_u)), ...
                ParamDescriptor("R", 1), ...
                ParamDescriptor("Q", 1), ...
                ParamDescriptor("lambda_a", 0), ...
                ParamDescriptor("lambda_s", 0), ...
                ParamDescriptor("lambda_s_ini", 0), ...
                ParamDescriptor("lambda_term_u", 0), ...
                ParamDescriptor("lambda_term_y", 0), ...
                ParamDescriptor("Lc", @(params) params.L), ...
                ParamDescriptor("terminal_constraint_size", 1), ...
                ParamDescriptor("affine_constraint", 1), ...
                ParamDescriptor("use_overshoot_constraints", 1), ...
                ParamDescriptor("use_input_terminal_constraints", 1), ...
                ParamDescriptor("use_input_delta_constraints", 1), ...
                ParamDescriptor("use_projected_regularization", 0), ...
                ParamDescriptor("is_strict_terminal_constraint", 1), ...
                ParamDescriptor("use_ref_integral", 0), ...
                ParamDescriptor("Ki", 1), ...            
                ParamDescriptor("input_delta", inf), ...
                ParamDescriptor("decay", 1), ...
                ParamDescriptor("delta_sat", 1), ...
                ParamDescriptor("allowed_offset_terminal", 0), ...
                ParamDescriptor("allowed_offset_slack", 0), ...
                ParamDescriptor("u_min", -inf), ...
                ParamDescriptor("u_max", inf), ...
                ParamDescriptor("y_min", -inf), ...
                ParamDescriptor("y_max", inf) ...
            };

            if exist('override_params', 'var')
                p = ParamHelpers.override_params(p, override_params);
            end
        end


        
    end
end

