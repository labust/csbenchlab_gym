function [u, log] = fcn_8fca807993a945929ced6204fb63db44_ext(y_ref, y, dt, u_ic, params, data, pid__, iid__)

    persistent comp_dict
    if isempty(comp_dict)
        comp_dict = dictionary;
    end

    iid = char(iid__);
    if comp_dict.numEntries == 0 || ...
        ~comp_dict.isKey(iid)
        o = DeePC('Params', params, 'Data', data, 'pid', pid__, 'iid', iid__);
        o = o.configure(u_ic, size(y), size(y_ref));
        comp_dict(iid) = o;
    else
        o = comp_dict(iid);
    end
    
    [o,  u] = o.step(y_ref, y, dt);
    comp_dict(iid) = o;
    log = eval_log(o.data);
end


function log = eval_log(data)
        log.x_op_u = double(data.x_op_u);
    log.x_op_y = double(data.x_op_y);
    log.x_op_g = double(data.x_op_g);

end


