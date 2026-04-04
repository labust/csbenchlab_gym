classdef BankedDeePC < Controller
    %DEEPC Implementation of the deepc controller
    
    properties (Constant)
        param_description = ...
            DeePCHelpers.get_deepc_param_set( ...
            { ...
                ParamDescriptor("params_path", 1), ...
            }, ...
            { ...
                "H", "D_u", "D_y", "T", "static_gain"
            });
        
       
        log_description = { 
            LogEntry('x_op_u'), ... 
            LogEntry('x_op_y'), ...
            LogEntry('x_op_g')
        };
    end

    properties
        bank
    end

    methods (Static)
        function data = create_data_model(options)
            params = options.params;
            mux = options.mux;
            BankedDeePC.load_solution_to_workspace(char(params.params_path))
            base_variable_name = char('bank');
            bank = get_workspace_variable(base_variable_name);
            params.D_u = squeeze(bank.u_trajs(1, :, :));
            params.D_y = squeeze(bank.y_trajs(1, :, :));
            data = DeePCHelpers.create_basic_data_model(params, mux);
        end
    end
    
    methods

        function this = BankedDeePC(varargin)
            this@Controller(varargin);  
        end

        function this = on_configure(this)
            idx = this.data.idx;
            BankedDeePC.load_solution_to_workspace(char(this.params.params_path))
            base_variable_name = char('bank');
            this.bank = get_workspace_variable(base_variable_name);

            utraj1 = squeeze(this.bank.u_trajs(1, :, :));
            ytraj1 = squeeze(this.bank.y_trajs(1, :, :));                
            this.data.A = DeePCHelpers.update_data_matrix(idx, this.data.A, ...
                utraj1, ytraj1, [], ...
                this.data.T, ...
                this.data.m, this.data.p, this.params);

            [this.data.lb, this.data.ub] = DeePCHelpers.configure_bounds( ...
                this.data.lb, this.data.ub, idx, this.params);
        end

        function [this, u] = on_step(this, y_ref, y, dt)
         
            idx = this.data.idx;
            p = this.data.p;
            m = this.data.m;
            this.data.yini = ...
                DeePCHelpers.update_ini(y, this.data.yini, size(y, 1));

            this.data.b(idx.uini_v.r) = this.data.uini;
            this.data.b(idx.yini_v.r) = this.data.yini;


            if this.params.use_ref_integral
                [y_ref, this.data] = DeePCHelpers.handle_ref_integral(y_ref, y, dt, this.data, this.params);
            end

            [H, hankel_idx] = this.bank.get_hankel_at(reshape(this.data.uini, [], m), reshape(this.data.yini, [], p));
            


            this.data.A = DeePCHelpers.update_data_matrix(idx, this.data.A, ...
                [], [], H, ...
                this.data.T, ...
                this.data.m, this.data.p, this.params);

            % x_op = this.data.x_op;

            static_gain = this.bank.get_static_gain(hankel_idx);
            % static_gain = this.params.static_gain;

            [this.data.b, this.data.A_lt, ...
             this.data.b_lt, this.data.optim_f, this.data.x_op] = ...
                DeePCHelpers.update_matrices(y_ref, y, ...
                this.data.vel_stop_idx, static_gain, ...
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
            u = zeros(m, 1);
            if optim_exit_flag >= 0 
                this.data.x_op = x_op_new; 
            else
                u(:) = optim_exit_flag;
            end
            u(:) = this.data.x_op(idx.u.b:idx.u.b+m-1);


            this.data.uini = ...
                DeePCHelpers.update_ini(u, this.data.uini, size(u, 1));

            this.data.x_op_u = x_op_new(idx.u.r);
            this.data.x_op_y = x_op_new(idx.y.r);
            this.data.x_op_g = x_op_new(idx.a.r);
            this.bank = this.bank.update_pred(reshape(this.data.x_op_u , this.params.L, size(u, 1)), ...
                reshape(this.data.x_op_y, this.params.L, size(y, 1)));

        end

        function this = on_reset(this)
            this.data.uini = zeros(size(this.data.uini));
            this.data.yini = zeros(size(this.data.yini));
        end

        

    end

    methods(Static)
         function load_solution_to_workspace(solution_path)
            evalin('base', strcat('load("', solution_path, '");'));
         end

    end
end

