function [u, log] = fcn_6119cc4ae8b54e6dbdbd59c4323f7501_ext(y_ref, y, dt, u_ic, params, data, pid__, iid__)

    persistent comp_dict
    if isempty(comp_dict)
        comp_dict = dictionary;
    end

    iid = char(iid__);
    if comp_dict.numEntries == 0 || ...
        ~comp_dict.isKey(iid)
        o = ExplicitDeePC(params, 'Data', data, 'pid', pid__, 'iid', iid__);
        o = o.configure(1, size(y), size(y_ref));
        comp_dict(iid) = o;
    else
        o = comp_dict(iid);
    end
    
    [o, u, log] = o.step(y_ref, y, dt);
    comp_dict(iid) = o;
end

