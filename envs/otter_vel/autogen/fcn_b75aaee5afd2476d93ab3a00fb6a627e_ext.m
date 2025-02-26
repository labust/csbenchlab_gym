
function y_n = fcn_b75aaee5afd2476d93ab3a00fb6a627e_ext(y, dt, params, data)
  persistent obj
  if isempty(obj)
    obj = Gauss(params, 'Data', data);
    obj = obj.configure();
  end
  [obj, y_n] = obj.step(y, dt);
end