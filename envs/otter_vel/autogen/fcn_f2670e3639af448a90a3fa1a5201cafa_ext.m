
function y_n = fcn_f2670e3639af448a90a3fa1a5201cafa_ext(y, dt, params, data)
  persistent obj
  if isempty(obj)
    obj = Gauss(params, 'Data', data);
    obj = obj.configure();
  end
  [obj, y_n] = obj.step(y, dt);
end