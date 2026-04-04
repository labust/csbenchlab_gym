classdef Otter12dof < DynSystem
    % VesselSurgeNonlinear implements nonlinear surge motion dynamics
    
    properties (Constant)
        param_description = {
            ParamDescriptor("mp", 0), ...         % mass including added mass
            ParamDescriptor("rp", [0 0 0]), ...         % nonlinear drag coefficient
            ParamDescriptor("V_c", 0), ... % output saturation min
            ParamDescriptor("beta_c", 0) ...   % output saturation max
        };
    end
    
    properties
        xk_1 % previous state [position; velocity]
    end
    
    methods
        function this = Otter12dof(varargin)
            this@DynSystem(varargin);
        end


        function xdot = exfunc(this, t, x)
            x_k = x(1:12);
            u_k = x(13:end);
            xdot = zeros(size(x));
            xdot(1:12) = otter(x_k, u_k, this.params.mp, this.params.rp, this.params.V_c, this.params.beta_c);
            
        end
        
        function this = on_configure(this)
            this.xk_1 = zeros(12, 1);
            if ~is_valid_field(this.params, 'mp')
                this.params.mp = 0;
            end
            if ~is_valid_field(this.params, 'rp')
                this.params.rp = [0;0;0];
            end
            if ~is_valid_field(this.params, 'V_c')
                this.params.V_c = 0;
            end
            if ~is_valid_field(this.params, 'beta_c')
                this.params.beta_c = 0;
            end
        end
        
        function [this, yk] = on_step(this, u, t, dt)
            if isscalar(u)
                u = [u; 0];
            end

            alloc = [1 1; 1 -1];
            n = alloc * (u);

            k_pos = 0.02216/2;                      
            k_neg = 0.01289/2;  
            
            gain_pos = 119.6820;
            gain_neg = 66.7080;
            
            for i = 1:1:2
                if n(i) > 0              % saturation, physical limits
                   n(i) = sqrt(n(i) / k_pos * gain_pos); 
                else
                   n(i) = -sqrt(-n(i) / k_neg * gain_neg); 
                end
            end
            x0 = [this.xk_1; n];
            options = odeset('Refine', 1);
            [tout, yout] = ode45(@this.exfunc, [t, t+dt/2, t+dt], x0, options);
            this.xk_1 = yout(end, 1:12)';
            yk = this.xk_1;
        end
        
        function this = on_reset(this)
            this.xk_1 = zeros(12, 1);
        end
    end
    
    methods (Static)
        function dims = get_dims_from_params(params)
            dims.Inputs = 1;  % single surge force input
            dims.Outputs = 1; % position and velocity outputs
        end
        
        function data = create_data_model(options)
            % No matrices for nonlinear system, just pass empty data
            data = struct();
        end
    end
end
