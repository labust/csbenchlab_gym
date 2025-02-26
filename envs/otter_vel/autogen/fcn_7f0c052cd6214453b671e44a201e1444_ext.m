
function [u, log] = fcn_7f0c052cd6214453b671e44a201e1444_ext(y_ref, y, dt, u_ic, params, data)
  persistent obj
  if isempty(obj)
    obj = DeePC(params, 'Data', data);
    obj = obj.configure(1, size(y), size(y_ref));
  end
  [obj, u, log] = obj.step(y_ref, y, dt);
end