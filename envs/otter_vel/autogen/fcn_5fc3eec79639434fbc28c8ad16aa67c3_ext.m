
function y_hat = fcn_5fc3eec79639434fbc28c8ad16aa67c3_ext(y, dt, ic, params, data)
  persistent obj
  if isempty(obj)
    obj = PropagateState(params, 'Data', data);
    obj = obj.configure(ic);
  end
  [obj, y_hat] = obj.step(y, dt);
end