function [u, log] = fcn_8881cb015dbf4ed9b43b6a5b55060d73_ext(y_ref, y, dt, u_ic, cfg_path, pid__, iid__)

    persistent comp_dict
    if isempty(comp_dict)
        comp_dict = dictionary;
    end

    iid = char(iid__);
    if comp_dict.numEntries == 0 || ...
        ~comp_dict.isKey(iid)
        o = load_casadi_component(char(cfg_path));
        o.configure();
    else
        o = comp_dict(iid);
    end
    
    result = o.step(y_ref, y, dt);
    data = o.data_update();
    
    comp_dict(iid) = o;
    log = eval_log(o.data);
end


function log = eval_log(data)
        log.u_pred = double(data.u_pred);
    log.y_pred = double(data.y_pred);

end


