
function [u, log] = fcn_1943935ad6ad49a5a0f8d86c7dbd92c2_ext(y_ref, y, dt, trajectory, u_ic, params, data)
  persistent obj
  if isempty(obj)
    obj = ATDeePC(params, 'Data', data);
    obj = obj.configure(1, size(y), size(y_ref));
  end
  [obj, u, log] = obj.step(y_ref, y, dt, trajectory);
end