
function [u, log] = fcn_7288160711694d0db4ecd0320b575afd_ext(y_ref, y, dt, trajectory, u_ic, params, data)
  persistent obj
  if isempty(obj)
    obj = ATDeePC(params, 'Data', data);
    obj = obj.configure(1, size(y), size(y_ref));
  end
  [obj, u, log] = obj.step(y_ref, y, dt, trajectory);
end