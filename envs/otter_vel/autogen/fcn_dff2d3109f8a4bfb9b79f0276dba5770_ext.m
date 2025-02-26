
function y_hat = fcn_dff2d3109f8a4bfb9b79f0276dba5770_ext(y, dt, ic, params, data)
  persistent obj
  if isempty(obj)
    obj = PropagateState(params, 'Data', data);
    obj = obj.configure(ic);
  end
  [obj, y_hat] = obj.step(y, dt);
end