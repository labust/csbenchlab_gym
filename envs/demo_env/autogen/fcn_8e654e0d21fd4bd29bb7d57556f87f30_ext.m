function [u, log] = fcn_8e654e0d21fd4bd29bb7d57556f87f30_ext(y_ref, y, dt, u_ic, data, pid__, iid__)

    persistent comp_dict
    if isempty(comp_dict)
        comp_dict = dictionary;
    end

    iid = char(iid__);
    if comp_dict.numEntries == 0 || ...
        ~comp_dict.isKey(iid)
        params = get_component_params_from_iid('demo_env', iid__);
        mux = get_controller_mux_struct('demo_env/Controller2/Controller2');
        o = PyComponentManager.instantiate_component('Controller2', 'demo_lib', 'Params', params, 'Mux', mux, 'pid', pid__, 'iid', iid__);
        o.configure(size(y), size(y_ref));
        comp_dict(iid) = o;
    else
        o = comp_dict(iid);
    end
    
    result = o.step(y_ref, y, dt);
    u = double(result)';
    comp_dict(iid) = o;
    log = eval_log(o.data);
end


function log = eval_log(data)
        log.d3 = double(data.d3);

end


