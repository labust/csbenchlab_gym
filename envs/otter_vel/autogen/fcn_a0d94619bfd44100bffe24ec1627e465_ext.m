
function y_n = fcn_a0d94619bfd44100bffe24ec1627e465_ext(y, dt, params, data)
  persistent obj
  if isempty(obj)
    obj = Gauss(params, 'Data', data);
    obj = obj.configure();
  end
  [obj, y_n] = obj.step(y, dt);
end