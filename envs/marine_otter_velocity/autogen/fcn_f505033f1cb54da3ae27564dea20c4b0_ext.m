function [u, log] = fcn_f505033f1cb54da3ae27564dea20c4b0_ext(y_ref, y, dt, u_ic, cfg_path, data, pid__, iid__)

    persistent comp_dict
    if isempty(comp_dict)
        comp_dict = dictionary;
    end

    iid = char(iid__);
    if comp_dict.numEntries == 0 || ...
        ~comp_dict.isKey(iid)
        o = load_casadi_component(char(cfg_path));
        o.data = data;
        o.configure();
    else
        o = comp_dict(iid);
    end
    
    [o, u] = o.step(o, y_ref, y, dt);
    
    comp_dict(iid) = o;
    log = eval_log(o.data);
end


function log = eval_log(data)
        log.u_pred = double(data.u_pred);
    log.y_pred = double(data.y_pred);

end


