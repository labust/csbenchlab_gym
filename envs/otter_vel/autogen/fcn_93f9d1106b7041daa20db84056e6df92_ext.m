
function [u, log] = fcn_93f9d1106b7041daa20db84056e6df92_ext(y_ref, y, dt, u_ic, params, data)
  persistent obj
  if isempty(obj)
    obj = DeePC(params, 'Data', data);
    obj = obj.configure(1, size(y), size(y_ref));
  end
  [obj, u, log] = obj.step(y_ref, y, dt);
end