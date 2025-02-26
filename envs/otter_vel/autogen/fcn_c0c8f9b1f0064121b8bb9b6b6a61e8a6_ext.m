
function y_n = fcn_c0c8f9b1f0064121b8bb9b6b6a61e8a6_ext(y, dt, params, data)
  persistent obj
  if isempty(obj)
    obj = Gauss(params, 'Data', data);
    obj = obj.configure();
  end
  [obj, y_n] = obj.step(y, dt);
end