
function y_hat = fcn_457e084df29b4fbc82e5903b7c40848d_ext(y, dt, ic, params, data)
  persistent obj
  if isempty(obj)
    obj = PropagateState(params, 'Data', data);
    obj = obj.configure(ic);
  end
  [obj, y_hat] = obj.step(y, dt);
end