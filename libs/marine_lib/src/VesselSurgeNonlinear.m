classdef VesselSurgeNonlinear < DynSystem
    % VesselSurgeNonlinear implements nonlinear surge motion dynamics
    
    properties (Constant)
        param_description = {
            ParamDescriptor("m", 1), ...         % mass including added mass
            ParamDescriptor("d", 1), ...         % nonlinear drag coefficient
            ParamDescriptor("sat_min", @(params) -inf * ones(2, 1)), ... % output saturation min
            ParamDescriptor("sat_max", @(params) inf * ones(2, 1)) ...   % output saturation max
        };
        registry_info = RegistryInfo("VesselSurgeNonlinear", true);
    end
    
    properties
        xk_1 % previous state [position; velocity]
    end
    
    methods
        function this = VesselSurgeNonlinear(varargin)
            this@DynSystem(varargin);
        end
        
        function this = on_configure(this)
            this.xk_1 = zeros(2, 1);
        end
        
        function [this, yk] = on_step(this, u, t, dt)
            % u is the surge force input (scalar)
            % States: xk_1(1): position, xk_1(2): velocity
            
            % Retrieve parameters
            m = this.params.m;
            d = this.params.d;
            
            xk = this.xk_1;
            % Unpack state
            pos = xk(1);
            vel = xk(2);
            
            % Nonlinear dynamics derivatives (Euler integration)
            dx1 = vel;
            dx2 = (u - d * vel * abs(vel)) / m;
            
            % Euler integration to update state
            new_pos = pos + dx1 * dt;
            new_vel = vel + dx2 * dt;
            
            % Updated state vector
            xk_new = [new_pos; new_vel];
            
            % Output is full state here
            yk = xk_new(2);
            
            % Saturate output if needed
            saturate = @Utils.saturate;
            yk = saturate(yk, this.params.sat_min, this.params.sat_max);
            
            % Update internal state
            this.xk_1 = xk_new;
        end
        
        function this = on_reset(this)
            this.xk_1 = zeros(2, 1);
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
