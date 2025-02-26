
function [u, log] = fcn_acdc5143463d46bb8a855abdadd5f3e4_ext(y_ref, y, dt, trajectory, u_ic, params, data)
  persistent obj
  if isempty(obj)
    obj = ATDeePC(params, 'Data', data);
    obj = obj.configure(1, size(y), size(y_ref));
  end
  [obj, u, log] = obj.step(y_ref, y, dt, trajectory);
end