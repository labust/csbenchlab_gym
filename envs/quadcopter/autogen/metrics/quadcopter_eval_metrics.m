function quadcopter_eval_metrics(name, varargin)
    eval(strcat(name, '(varargin{:})'));
end