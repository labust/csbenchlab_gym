function [log,  u] = fcn_c55fdcc11c9c42e88157476ec69ed224_ext(y_ref, y, dt, u_ic, data, pid__, iid__)

    persistent comp_dict
    if isempty(comp_dict)
        comp_dict = dictionary;
    end

    iid = char(iid__);
    if comp_dict.numEntries == 0 || ...
        ~comp_dict.isKey(iid)
        params = get_component_params_from_iid('marine_otter_velocity', iid__);
        mux = get_controller_mux_struct('marine_otter_velocity/DeePC_acados/DeePC_acados');
        o = PyComponentManager.instantiate_component('DeePC', 'data_driven_lib_new', 'Params', params, 'Mux', mux, 'pid', pid__, 'iid', iid__);
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
        log.u_pred = double(data.u_pred);
    log.y_pred = double(data.y_pred);

end


