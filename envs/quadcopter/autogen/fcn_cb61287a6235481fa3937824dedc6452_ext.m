
function y_hat = fcn_cb61287a6235481fa3937824dedc6452_ext(y, dt, ic, params, data)
  persistent obj
  if isempty(obj)
    obj = PropagateState(params, 'Data', data);
    obj = obj.configure(ic);
  end
  [obj, y_hat] = obj.step(y, dt);
end