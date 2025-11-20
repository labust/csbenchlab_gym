classdef TestController < Controller
    %TestController 
    properties (Constant)
        param_description = {};
    end

    methods

        function this = TestController(varargin)
            this@Controller(varargin);  
        end

        function this = on_configure(this)
           
        end

        function [this, u] = on_step(this, y_ref, y, dt)
         	u = 1;
        end

        function this = on_reset(this)
        end
    end

end


