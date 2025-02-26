
function y_hat = fcn_e6d3b2c485ea4585b6e3420892c04824_ext(y, dt, ic, params, data)
  persistent obj
  if isempty(obj)
    obj = PropagateState(params, 'Data', data);
    obj = obj.configure(ic);
  end
  [obj, y_hat] = obj.step(y, dt);
end