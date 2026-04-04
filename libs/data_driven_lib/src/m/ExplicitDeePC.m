classdef ExplicitDeePC < Controller
    %ExplicitDeePC Implementation 

    properties (Constant)
        param_description = {
                ParamDescriptor("A", 1), ...
                ParamDescriptor("B", 1), ...
                ParamDescriptor("C", 1), ...
                ParamDescriptor("D", 1), ...
                ParamDescriptor("Tini", 1), ...
                ParamDescriptor("solution_path", 0), ...
                ParamDescriptor("is_incremental", 0), ...
                ParamDescriptor("use_ref_integral", 0), ...
                ParamDescriptor("Ki", 0), ...
                ParamDescriptor("u_min", -inf), ...
                ParamDescriptor("u_max", inf), ...
                ParamDescriptor("y_min", -inf), ...
                ParamDescriptor("y_max", inf), ...
                ParamDescriptor("base_variable_name", 0) ...
                ParamDescriptor("out_gain", 1) ...
            };
        log_description = { 

        };
    end

    properties
        sol
        uu
    end

    methods (Static)
        function data = create_data_model(options)
            params = options.params;
            mux = options.mux;
            data.m = length(mux.Inputs);
            data.p = length(mux.Outputs);
            data.uini = zeros(params.Tini * data.m, 1);
            data.yini = zeros(params.Tini * data.p, 1);
            data.eta = zeros(data.p, 1);
            data.old_y_ref = zeros(data.p);
            [data.model.O, data.model.Cl] = ExplicitDeePC.construct_oc_matrices(params.A, params.B, params.C, params.D, params.Tini);
            data.model.O_pinv = pinv(data.model.O);
        end
    end
    
    methods

        function this = ExplicitDeePC(varargin)
            this@Controller(varargin);  
        end

        function this = on_configure(this)
            this.load_solution_to_workspace(char(this.params.solution_path))
            base_variable_name = char(this.params.base_variable_name);
            this.sol = get_workspace_variable(base_variable_name);
            this.uu = 0;
        end

        function [this, u] = on_step2(this, y_ref, y, dt, s)
            this.sol = getArrayFromByteStream(s);
            [this, u] = on_step(this, y_ref, y, dt);
        end


        function [this, u] = on_step(this, y_ref, y, dt)
            
            if this.params.use_ref_integral
                if this.params.use_ref_integral
                    [y_ref, this.data] = DeePCHelpers.handle_ref_integral(y_ref, y, dt, this.data, this.params);
                end
            end

            this.data.yini = ...
                DeePCHelpers.update_ini(y, this.data.yini, size(y, 1));

            
            optim_u = evaluate_explicit(this.sol, y_ref', this.uu, this.data, this.params);
        
            optim_u = optim_u(1:this.data.m);
            if isnan(optim_u)
                if this.params.is_incremental
                    optim_u = 0;
                else
                    optim_u = this.data.uini(end-this.data.m+1:end);
                end
            end
            
            if this.params.is_incremental
                u = this.uu + optim_u;
            else
                u = optim_u;
            end
            u = u * this.params.out_gain;
            u = Utils.saturate(u, this.params.u_min, this.params.u_max);
            
            if this.params.is_incremental
                optim_u = u - this.uu;
                this.uu = u;
            end
          
            this.data.uini = ...
                DeePCHelpers.update_ini(optim_u, this.data.uini, size(u, 1));
            
        end

        function this = on_reset(this)
            this.data.uini = zeros(size(this.data.uini));
            this.data.yini = zeros(size(this.data.yini));
        end

        function load_solution_to_workspace(this, solution_path)
            path = get_component_context_path_from_iid(this.iid);
            evalin('base', strcat( ...
                'load("', fullfile(path, solution_path), '");'));
        end

    end

    methods (Static)

        function [O, Cl] = construct_oc_matrices(A, B, C, D, L)

            O = C;
            Cl = D;
        
            a = A;
            m = size(B, 2);
            p = size(C, 1);
            h = C * B;
            for i=2:L
                O = [O; C * a];
                Cl = [Cl zeros(height(Cl), m); zeros(p, width(Cl)), D];
                Cl(end-p+1:end, m+1:(i*m)) = Cl(end-2*p+1:end-p, 1:((i-1)*m));
                Cl(end-p+1:end, 1:m) = h;
                h = C * a * B;
                a = A * a;
            end
        end

    end
end

