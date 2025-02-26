
function y_n = fcn_aaffb15128b74978876f50f91776a804_ext(y, dt, params, data)
  persistent obj
  if isempty(obj)
    obj = ZeroNoise(params, 'Data', data);
    obj = obj.configure();
  end
  [obj, y_n] = obj.step(y, dt);
end