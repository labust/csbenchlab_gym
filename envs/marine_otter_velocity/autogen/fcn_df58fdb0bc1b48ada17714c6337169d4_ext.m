function [u, log] = fcn_df58fdb0bc1b48ada17714c6337169d4_ext(y_ref, y, dt, u_ic, cfg_path, pid__, iid__)

    persistent comp_dict
    if isempty(comp_dict)
        comp_dict = dictionary;
    end

    iid = char(iid__);
    if comp_dict.numEntries == 0 || ...
        ~comp_dict.isKey(iid)
        params = get_component_params_from_iid('marine_otter_velocity', iid__);
        mux = get_controller_mux_struct('marine_otter_velocity/DeePCtest/DeePCtest');
        o = PyComponentManager.instantiate_component('DeePCtest', 'data_driven_lib_new',  cfg_path, 'pid', pid__, 'iid', iid__);
        o.configure(u_ic, size(y), size(y_ref));
        comp_dict(iid) = o;
    else
        o = comp_dict(iid);
    end
    
    result = o.step(y_ref, y, dt);
    
    comp_dict(iid) = o;
    log = eval_log(o.data);
end


function log = eval_log(data)
        log.u_pred = double(data.u_pred);
    log.y_pred = double(data.y_pred);

end


