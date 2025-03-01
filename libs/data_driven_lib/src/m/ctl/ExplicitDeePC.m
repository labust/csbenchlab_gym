classdef ExplicitDeePC < Controller
    %ExplicitDeePC Implementation 

    properties (Constant)
        param_description = ParamSet( ...
                ParamDescriptor("A", 1), ...
                ParamDescriptor("B", 1), ...
                ParamDescriptor("C", 1), ...
                ParamDescriptor("D", 1), ...
                ParamDescriptor("Tini", 1) ...
            );
       
        log_description = { 

        };
    end

    properties
        Polih
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
            this.Polih = PolyUnion;
            this.uu = 0;
        end

        function [this, u] = on_step(this, y_ref, y, dt)

            this.data.yini = ...
                DeePCHelpers.update_ini(y(1), this.data.yini, size(y, 1));
            xini = this.data.O_pinv * (this.data.yini - this.data.Cl*this.data.uini);
            % xini = this.data.yini(end);
            % xini_end = this.data.O * xini + this.data.Cl * uini;

            poly = evalin('base', 'sss');
            du = poly.feval([xini;  this.uu; y_ref], 'primal', 'tiebreak', 'obj');
            if isnan(du)
                du = this.data.uini(end);
            end
            
            u = this.uu + du;
            if u > 500
                u = 500;
            elseif u < -300
                u = -300;
            end

            du = u - this.uu;

            this.data.uini = ...
                DeePCHelpers.update_ini(du, this.data.uini, size(u, 1));

            this.uu = u;
            
        end

        function this = on_reset(this)
            this.data.uini = zeros(size(this.data.uini));
            this.data.yini = zeros(size(this.data.yini));
        end


        function eval(this, xini, y_ref)
            
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

