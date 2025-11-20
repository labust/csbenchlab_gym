function  u = fcn_3d720a34a98b4a66bf956a8e8c46a416_ext(y_ref, y, dt, u_ic, params, data, pid__, iid__)

    persistent comp_dict
    if isempty(comp_dict)
        comp_dict = dictionary;
    end

    iid = char(iid__);
    if comp_dict.numEntries == 0 || ...
        ~comp_dict.isKey(iid)
        o = ATExplicitDeePC('Params', params, 'Data', data, 'pid', pid__, 'iid', iid__);
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
        log = 0;

end


