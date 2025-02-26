
function y = fcn_a71f8bd152194cb1959f0fcc89ee4fca_ext(u, t, dt, ic, params_merged, data)
  persistent obj
  if isempty(obj)
    obj = LinearSystem(params_merged, 'Data', data);
    obj = obj.configure(ic);
  end
  [obj, y] = obj.step(u, t, dt);
end