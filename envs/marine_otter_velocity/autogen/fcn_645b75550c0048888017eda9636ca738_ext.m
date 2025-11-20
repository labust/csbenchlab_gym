function  y_n = fcn_645b75550c0048888017eda9636ca738_ext(y, dt, params, data, pid__, iid__)

    persistent comp_dict
    if isempty(comp_dict)
        comp_dict = dictionary;
    end

    iid = char(iid__);
    if comp_dict.numEntries == 0 || ...
        ~comp_dict.isKey(iid)
        o = ZeroNoise('Params', params, 'Data', data, 'pid', pid__, 'iid', iid__);
        o = o.configure();
        comp_dict(iid) = o;
    else
        o = comp_dict(iid);
    end
    
    [o,  y_n] = o.step(y, dt);
    comp_dict(iid) = o;
    log = eval_log(o.data);
end


function log = eval_log(data)
        log = 0;

end


