function [u, log] = fcn_f6e1d73cce7e4c17926a122faad0b370_ext(y_ref, y, dt, u_ic, params, data, pid__, iid__)

    persistent comp_dict
    if isempty(comp_dict)
        comp_dict = dictionary;
    end

    iid = char(iid__);
    if comp_dict.numEntries == 0 || ...
        ~comp_dict.isKey(iid)
        o = DeePC(params, 'Data', data, 'pid', pid__, 'iid', iid__);
        o = o.configure(1, size(y), size(y_ref));
        comp_dict(iid) = o;
    else
        o = comp_dict(iid);
    end
    
    [o, u, log] = o.step(y_ref, y, dt);
    comp_dict(iid) = o;
end

