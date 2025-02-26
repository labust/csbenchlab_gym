function y_n = fcn_ec3fa11c3aca44c6bb4f8b76b72872bc_ext(y, dt, params, data)
  persistent obj
  if isempty(obj)
    obj = ZeroNoise(params, 'Data', data);
    obj = obj.configure();
  end
  [obj, y_n] = obj.step(y, dt);
end