function y = fcn_7677e8e8f2064203b5a9c4fcc66cdfbb_ext(u, t, dt, ic, params_merged, data)
  persistent obj
  if isempty(obj)
    obj = LinearSystem(params_merged, 'Data', data);
    obj = obj.configure(ic);
  end
  [obj, y] = obj.step(u, t, dt);
end