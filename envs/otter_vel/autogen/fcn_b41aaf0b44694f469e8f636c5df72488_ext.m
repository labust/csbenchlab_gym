
function y_n = fcn_b41aaf0b44694f469e8f636c5df72488_ext(y, dt, params, data)
  persistent obj
  if isempty(obj)
    obj = Gauss(params, 'Data', data);
    obj = obj.configure();
  end
  [obj, y_n] = obj.step(y, dt);
end