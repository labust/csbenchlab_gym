
function [u, log] = fcn_bc71a3e98f25495384c8a4c9077e7030_ext(y_ref, y, dt, trajectory, u_ic, params, data)
  persistent obj
  if isempty(obj)
    obj = ATDeePC(params, 'Data', data);
    obj = obj.configure(1, size(y), size(y_ref));
  end
  [obj, u, log] = obj.step(y_ref, y, dt, trajectory);
end