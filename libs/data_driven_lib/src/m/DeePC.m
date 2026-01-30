classdef DeePC < Controller
    %DEEPC Implementation of the deepc controller
    
    properties (Constant)
        param_description = DeePCHelpers.get_deepc_param_set();
       
        log_description = { 
            LogEntry('x_op_u'), ... 
            LogEntry('x_op_y'), ...
            LogEntry('x_op_g')
        };
    end

    methods (Static)
        function data = create_data_model(params, mux)
            data = DeePCHelpers.create_basic_data_model(params, mux);
        end
    end
    
    methods

        function this = DeePC(varargin)
            this@Controller(varargin);  
        end

        function this = on_configure(this)
            idx = this.data.idx;
                
            this.data.A = DeePCHelpers.update_data_matrix(idx, this.data.A, ...
                this.params.D_u, this.params.D_y, this.params.H, ...
                this.data.T, ...
                this.data.m, this.data.p, this.params);

            [this.data.lb, this.data.ub] = DeePCHelpers.configure_bounds( ...
                this.data.lb, this.data.ub, idx, this.params);
        end

        function [this, u] = on_step(this, y_ref, y, dt)
         
            idx = this.data.idx;
            this.data.yini = ...
                DeePCHelpers.update_ini(y(1), this.data.yini, size(y, 1));

            this.data.b(idx.uini_v.r) = this.data.uini;
            this.data.b(idx.yini_v.r) = this.data.yini;


            if this.params.use_ref_integral
                y_ref_v = y_ref(end);
                ref_change = abs(y_ref_v - this.data.old_y_ref) > 0.2;
                dx = 0.3;
                trend =  ((y - this.data.yini(1)) > dt * dx) ...
                 -1* ((y - this.data.yini(1)) < -dt * dx);
                is_far_from_ref = abs(y_ref_v - y) > 0.4;
                if ~is_far_from_ref || (is_far_from_ref && trend == 0)
                    if y_ref_v - y <= 0 && ~(trend < 0) || y_ref_v - y > 0 && ~(trend > 0) || trend == 0
                        this.data.eta = this.data.eta + this.params.Ki * (y_ref_v - y) * dt;
                    end
                end
                if ref_change || (is_far_from_ref && trend ~= 0)
                    this.data.eta = zeros(this.data.p);
                end

                this.data.eta = max(min(this.data.eta, this.params.y_max), this.params.y_min); 
                y_ref = y_ref_v + this.data.eta;
                this.data.old_y_ref = 0.85*(this.data.old_y_ref) + 0.15 * y_ref;
            end


            this.data.A = DeePCHelpers.update_data_matrix(idx, this.data.A, ...
                this.params.D_u, this.params.D_y, this.params.H, ...
                this.data.T, ...
                this.data.m, this.data.p, this.params);

            % x_op = this.data.x_op;

            [this.data.b, this.data.A_lt, ...
             this.data.b_lt, this.data.optim_f, this.data.x_op] = ...
                DeePCHelpers.update_matrices(y_ref, y, ...
                    this.data.vel_stop_idx, this.params.end_point, ...
                    this.data.b, ...
                    this.data.A_lt, this.data.b_lt, ...
                    this.data.optim_f, this.data.x_op, ...
                    idx, this.params);

            this.data.optim_T = DeePCHelpers.set_optim_params(this.data.optim_T, ...
                this.data.A, idx, this.params);

            if this.data.has_lt == 0
                [x_op_new, fval_new, optim_exit_flag] =  DeePCHelpers.optim(...
                    this.data.optim_T, this.data.optim_f, ...
                    this.data.A, this.data.b, this.data.lb, this.data.ub, ...
                    this.data.x_op);
            else
                [x_op_new, fval_new, optim_exit_flag] =  DeePCHelpers.optim_lt(...
                    this.data.optim_T, this.data.optim_f, ...
                    this.data.A, this.data.b, this.data.A_lt, this.data.b_lt, ...
                    this.data.lb, this.data.ub, ...
                    this.data.x_op);
            end

            this.data.fval = fval_new;
            
            % u = [this.data.fval; x_op_new(1); optim_exit_flag];
            u = 0;
            if optim_exit_flag >= 0 
                this.data.x_op = x_op_new; 
            else
                u(:) = optim_exit_flag;
            end
            u(:) = this.data.x_op(idx.u.b);

            this.data.uini = ...
                DeePCHelpers.update_ini(u, this.data.uini, size(u, 1));

            this.data.x_op_u = x_op_new(idx.u.r);
            this.data.x_op_y = x_op_new(idx.y.r);
            this.data.x_op_g = x_op_new(idx.a.r);
        end

        function this = on_reset(this)
            this.data.uini = zeros(size(this.data.uini));
            this.data.yini = zeros(size(this.data.yini));
        end
    end

    methods(Static)
         function value = init_p2(params)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            value = params;
         end
    end
end

