classdef DeePCPI < Controller
    %DEEPC Implementation of the deepc controller
    
    properties (Constant)
        param_description = DeePCHelpers.get_deepc_param_set( ...
        { 
           
        });
       
        log_description = { 
            LogEntry('x_op_u'), ... 
            LogEntry('x_op_y'), ...
            LogEntry('x_op_g')
        };

    end

    methods (Static)
        function data = create_data_model(params, dims)
            data = DeePCHelpers.create_basic_data_model(params, dims);

            data.deepc_step = 1;
            data.n_steps = 0;
        end
    end
    
    methods

        function this = DeePCPI(varargin)
            this@Controller(varargin);  
        end

        function this = on_configure(this)

            idx = this.data.idx;

            this.data.A = DeePCHelpers.update_data_matrix(idx, this.data.A, ...
                this.params.D_u, this.params.D_y, ...
                this.data.T, ...
                idx.m, idx.p, this.params);

            [this.data.lb, this.data.ub] = DeePCHelpers.configure_bounds( ...
                this.data.lb, this.data.ub, idx, this.params);
            this.data.deepc_step = 1;
            this.data.n_steps = 0;

        end

        function [this, u] = on_step(this, y_ref, y, dt)
            idx = this.data.idx;


            this.data.b(idx.uini_v.r) = this.data.uini;
            this.data.b(idx.yini_v.r) = this.data.yini;
         
            this.data.A = DeePCHelpers.update_data_matrix(idx, this.data.A, ...
                this.params.D_u, this.params.D_y, this.data.T, ...
                idx.m, idx.p, this.params);

            large_reference_change = any(abs(y_ref - y) > 0.1*this.params.y_max);

            if this.data.deepc_step == 0 || large_reference_change == 0
                this.data.x_op_u(1:end-idx.m) = this.data.x_op_u(idx.m+1:end);
                this.data.x_op_y(1:end-idx.p) = this.data.x_op_y(idx.p+1:end);

                [up, yp] = MPPIHelpers.mppi_step(y_ref, this.data, this.params, idx, 2000, 0.3, 0.015);
                this.data.x_op_u = up;
                this.data.x_op_y = yp;
                u = zeros(1, 1);
                u(:) = up(1:idx.m);
                this.data.n_steps = this.data.n_steps + 1;

                [this.data.uini, this.data.yini] = ...
                    DeePCHelpers.update_ini(u, y(1:this.data.vel_stop_idx), ...
                    this.data.uini, this.data.yini, idx.m, idx.p);

                if this.data.n_steps > 50
                    this.data.n_steps = 0;
                    this.data.deepc_step = 1;
                end
                return
            end

            this.data.deepc_step = 0;

            end_point = this.params.end_point;
            [this.data.b, this.data.A_lt, ...
                this.data.b_lt, this.data.optim_f, this.data.x_op] = ...
                    DeePCHelpers.update_matrices(y_ref, y, ...
                        this.data.vel_stop_idx, end_point, ...
                        this.data.b, ...
                        this.data.A_lt, this.data.b_lt, ...
                        this.data.optim_f, this.data.x_op, ...
                        idx, this.params);

            this.data.optim_T = DeePCHelpers.set_optim_params(this.data.optim_T, idx, this.params);

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
            u = zeros(1, 1);
            if optim_exit_flag >= 0 
                this.data.x_op = x_op_new; 
                u(:) = this.data.x_op(idx.u.b:idx.u.b+idx.m-1);
            else
                % u = this.data.x_op(idx.u.b:idx.u.b+idx.m-1);
                a = 5;
            end
            % u(1x  ) = 0;
            this.data.fval = fval_new;

            [this.data.uini, this.data.yini] = ...
                DeePCHelpers.update_ini(u, y(1:this.data.vel_stop_idx), ...
                this.data.uini, this.data.yini, idx.m, idx.p);

            this.data.x_op_u = this.data.x_op(idx.u.r);
            this.data.x_op_y = this.data.x_op(idx.y.r);
            this.data.x_op_g = this.data.x_op(idx.a.r);
        end

        function on_reset(this)
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

