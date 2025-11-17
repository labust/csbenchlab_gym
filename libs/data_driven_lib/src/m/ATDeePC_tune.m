classdef ATDeePC_tune < Controller
    %DEEPC Implementation of the deepc controller
    
    properties (Constant)
        param_description = ATDeePC.param_description;
       
        log_description = ATDeePC.log_description;

        input_description = [
            ATDeePC.input_description(:)', ...
            {IOArgument('input', "tune_params", @ATDeePC_tune.tune_params_arg_struct)}
        ];
    end

    properties
        controller_
    end

    methods (Static)
        function data = create_data_model(params, dims)
            data = ATDeePC.create_data_model(params, dims);
        end

         
        function value = tune_params_arg_struct(params, mux)
            value = struct;
            value.lambda_a = 1;
            value.lambda_s = 1;
            value.Q = 1;
            value.R = 1;
        end
    end
    
    methods

        function this = ATDeePC_tune(varargin)
            this@Controller(varargin);  
            this.controller_ = ATDeePC(varargin{:});
        end

        function this = on_configure(this)
            this.controller_ = this.controller_.configure();
        end

        function [this, u] = on_step(this, y_ref, y, dt, trajectory, tune_params)
            
            this = this.update_params(tune_params);

            [this.controller_, u, ~] = this.controller_.step(y_ref, y, dt, trajectory);
            this.data = this.controller_.data;
        end

        function this = update_params(this, tune_params)
            this.controller_.params.lambda_a = tune_params.lambda_a;
        end

        function on_reset(this)
            this.controller_.on_reset();
        end
       
    end

    
end

