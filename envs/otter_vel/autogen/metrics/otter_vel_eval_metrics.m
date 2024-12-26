function otter_vel_eval_metrics(name, varargin)
    eval(strcat(name, '(varargin{:})'));
end