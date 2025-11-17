classdef Controller1 < Controller
    properties (Constant)
        param_description = {
                ParamDescriptor("p1", 2), ...
                ParamDescriptor("p2", 3), ...
                ParamDescriptor("p12", @(params) params.p1 + params.p2) ...
        };

        log_description = {
            LogEntry('log1'), ...
        };
    end

    methods (Static)
        function data = create_data_model(params, dims)
            data.d1 = zeros(params.p1, params.p1);
            data.d12 = params.p12;
            data.log1 = zeros(1);
        end
    end

    methods

        function this = Controller1(varargin)
            this@Controller(varargin);
        end

        function this = on_configure(this)

        end

        function [this, u] = on_step(this, y_ref, y, dt)
            % This is where the controller logic goes
            % y_ref: reference signal
            % y: current measurement
            % dt: time step

            % Example logic (to be replaced with actual control logic)
            u = zeros(3, 1);
            u(1) = this.params.p2 * (y_ref(2) - y(2));  % Simple proportional control for first element
            
            this.data.log1 = u(1);  % Log the control action for third element
        end

        function on_reset(this)
        end

    end
end

