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
        end
        
        function [this, yk] = on_step(this, u, t, dt)
            if isscalar(u)
                u = [u; 0];
            end

            alloc = inv([1 1; 0.395 -0.395]);
            u = alloc * (200 * u);

            x0 = [this.xk_1; u];
            [tout, yout] = ode23(@this.exfunc, [0, dt], x0);
            this.xk_1 = yout(end, 1:12)';
            yk = this.xk_1(1);
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
        
        function data = create_data_model(params)
            % No matrices for nonlinear system, just pass empty data
            data = struct();
        end
    end
end
