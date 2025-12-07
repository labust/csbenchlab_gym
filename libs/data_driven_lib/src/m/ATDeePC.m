classdef ATDeePC < Controller
    %DEEPC Implementation of the deepc controller
    
    properties (Constant)
        param_description = DeePCHelpers.get_deepc_param_set( ...
        { 
            ParamDescriptor("dataset_bank_size", 1)...,
        });
       
        log_description = { 
            LogEntry('x_op_u'), ... 
            LogEntry('x_op_y'), ...
            LogEntry('x_op_g')
        };

        input_description = {
            IOArgument("trajectory", @ATDeePC.trajectory_arg_struct)
        };
    end

    methods (Static)
        function data = create_data_model(params, dims)
            data = DeePCHelpers.create_basic_data_model(params, dims);

            data.db_x_op = zeros(params.dataset_bank_size, data.idx.state.sz);
            data.db_fval_op = zeros(params.dataset_bank_size, 1);
            data.db_active_idx = 1;
        end
    end
    
    methods

        function this = ATDeePC(varargin)
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
        end

        function [this, u] = on_step(this, y_ref, y, dt, trajectory)
            idx = this.data.idx;
            this.data.yini = ...
                DeePCHelpers.update_ini(y(1:this.data.vel_stop_idx), ...
                this.data.yini, idx.p);
         
            this.data.b(idx.uini_v.r) = this.data.uini;
            this.data.b(idx.yini_v.r) = this.data.yini;
         
            this.data.A = DeePCHelpers.update_data_matrix(idx, this.data.A, ...
                trajectory.D_u, trajectory.D_y, this.data.T, ...
                idx.m, idx.p, this.params);

            end_point = trajectory.end_point;
            [this.data.b, this.data.A_lt, ...
                this.data.b_lt, this.data.optim_f, this.data.x_op] = ...
                    DeePCHelpers.update_matrices(y_ref, y, ...
                        this.data.vel_stop_idx, end_point, ...
                        this.data.b, ...
                        this.data.A_lt, this.data.b_lt, ...
                        this.data.optim_f, this.data.x_op, ...
                        idx, this.params);

            this.data.optim_T = DeePCHelpers.set_optim_params(this.data.optim_T, this.data.A, idx, this.params);

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
            end
            this.data.fval = fval_new;

            this.data.uini = ...
                DeePCHelpers.update_ini(u, ...
                this.data.uini, idx.m);

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

         function value = trajectory_arg_struct(params, mux)

            if isa(params.T, 'function_handle')
                T = params.T(params);
            else
                T = params.T;
            end
                
            input_sz = length(mux.Inputs);
            output_sz = length(mux.Outputs);
            if params.pos_control == 1
                assert(mod(input_sz, 2) == 0, ...
                    'Pos control must have even number of states');
                db_input_sz = input_sz / 2;
            else
                db_input_sz = input_sz;
            end

            value.D_u = [T, db_input_sz];
            value.D_y = [T, output_sz];
            % 
            % if db_input_sz > 1
            % else
            %     value.D_u = T;
            % end
            % 
            % if output_sz > 1
            %     value.D_y = [output_sz, T];
            % else
            %     value.D_y = T;
            % end
            value.end_point = output_sz;
        end
    end
end

