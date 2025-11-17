classdef ATExplicitDeePC < Controller
    %ExplicitDeePC Implementation 

    properties (Constant)
        param_description = {
                ParamDescriptor("solution_path", 1), ...
                ParamDescriptor("Tini", 1), ...
                ParamDescriptor("is_incremental", 0), ...
                ParamDescriptor("use_ref_integral", 0), ...
                ParamDescriptor("Ki", 0), ...
                ParamDescriptor("u_min", -inf), ...
                ParamDescriptor("u_max", inf), ...
                ParamDescriptor("y_max", inf), ...
                ParamDescriptor("y_min", inf), ...
            };
        log_description = { 

        };
    end

    properties
        pwl_model
        uu
    end

    methods (Static)
        function data = create_data_model(params, mux)
            data.m = size(mux.Inputs, 1);
            data.p = size(mux.Outputs, 1);
            data.uini = zeros(params.Tini, data.m);
            data.yini = zeros(params.Tini, data.p);
            data.eta = zeros(data.p);
            data.old_y_ref = zeros(data.p);
        end
    end
    
    methods

        function this = ATExplicitDeePC(varargin)
            this@Controller(varargin);  
        end

        function this = on_configure(this)

            pwl_path = fullfile( ...
                get_component_context_path_from_iid(this.iid), ...
                this.params.solution_path);
            this.pwl_model = load(pwl_path).pwl;
            for i=1:length(this.pwl_model)
                model = this.pwl_model{i};
                [this.data.model(i).O, this.data.model(i).Cl] = ...
                    ExplicitDeePC.construct_oc_matrices(model.A, model.B, model.C, model.D, this.params.Tini);
                this.data.model(i).O_pinv = pinv(this.data.model(i).O);
            end
            this.uu = 0;
        end


        function [this, u] = on_step(this, y_ref, y, dt)
            
            if this.params.use_ref_integral
                
                ref_change = abs(y_ref - this.data.old_y_ref) > 0.2;
                dx = 0.3;
                trend =  ((y - this.data.yini(1)) > dt * dx) ...
                 -1* ((y - this.data.yini(1)) < -dt * dx);
                is_far_from_ref = abs(y_ref - y) > 0.4;
                if ~is_far_from_ref || (is_far_from_ref && trend == 0)
                    if y_ref - y <= 0 && ~(trend < 0) || y_ref - y > 0 && ~(trend > 0) || trend == 0
                        this.data.eta = this.data.eta + this.params.Ki * (y_ref - y) * dt;
                    end
                end
                if ref_change || (is_far_from_ref && trend ~= 0)
                    this.data.eta = zeros(this.data.p);
                end

                this.data.eta = max(min(this.data.eta, 1.9), -1.9); 
                this.data.old_y_ref = 0.85*(this.data.old_y_ref) + 0.15 * y_ref;
            end

            this.data.yini = ...
                DeePCHelpers.update_ini(y(1), this.data.yini, size(y, 1));
            optim_u = evaluate_explicit(this.pwl_model, y_ref, this.uu, this.data, this.params);
        
            optim_u = optim_u(1);
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
            optim_u = u;
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

