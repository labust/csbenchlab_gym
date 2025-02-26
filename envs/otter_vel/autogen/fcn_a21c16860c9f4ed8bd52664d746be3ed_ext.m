
function [u, log] = fcn_a21c16860c9f4ed8bd52664d746be3ed_ext(y_ref, y, dt, trajectory, params, data)
  persistent obj
  if isempty(obj)
    obj = ATDeePC(params, 'Data', data);
    obj = obj.configure(1, size(y), size(y_ref));
  end
  [obj, u, log] = obj.step(y_ref, y, dt, trajectory);
end