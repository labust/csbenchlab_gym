classdef ExplicitDeePC < Controller
    %ExplicitDeePC Implementation 

    properties (Constant)
        param_description = {
                ParamDescriptor("A", 1), ...
                ParamDescriptor("B", 1), ...
                ParamDescriptor("C", 1), ...
                ParamDescriptor("D", 1), ...
                ParamDescriptor("Tini", 1), ...
                ParamDescriptor("is_incremental", 0), ...
                ParamDescriptor("u_min", -inf), ...
                ParamDescriptor("u_max", inf), ...
                ParamDescriptor("base_variable_name", 0) ...
            };
        log_description = { 

        };
    end

    properties
        sol
        uu
    end

    methods (Static)
        function data = create_data_model(params, mux)
            data.m = size(mux.Inputs, 1);
            data.p = size(mux.Outputs, 1);
            data.uini = zeros(params.Tini, data.m);
            data.yini = zeros(params.Tini, data.p);
            [data.O, data.Cl] = ExplicitDeePC.construct_oc_matrices(params.A, params.B, params.C, params.D, params.Tini);
            data.O_pinv = pinv(data.O);
        end
    end
    
    methods

        function this = ExplicitDeePC(varargin)
            this@Controller(varargin);  
        end

        function this = on_configure(this)
            base_variable_name = char(this.params.base_variable_name);
            this.sol = get_workspace_variable(base_variable_name);
            this.uu = 0;
        end

        function [this, u] = on_step2(this, y_ref, y, dt, s)
            this.sol = getArrayFromByteStream(s);
            [this, u] = on_step(this, y_ref, y, dt);
        end


        function [this, u] = on_step(this, y_ref, y, dt)
            
            % y = Utils.saturate(y, -2.3, 2.3);
            this.data.yini = ...
                DeePCHelpers.update_ini(y(1), this.data.yini, size(y, 1));
            

            optim_u = evaluate_explicit(this.sol, y_ref, this.uu, this.data, this.params);


            if isnan(optim_u)
                if this.params.is_incremental
                    optim_u = 0;
                else
                    optim_u = this.data.uini(end);
                end
            end
            
            if this.params.is_incremental
                u = this.uu + optim_u;
            else
                u = optim_u;
            end
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
                a = a*a;
            end
        end
    end
end

