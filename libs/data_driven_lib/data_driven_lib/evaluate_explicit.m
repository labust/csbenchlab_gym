function optim_u = evaluate_explicit(sol_or_union, y_ref, uu, data, params)
   
    [sol, idx] = select_sol(sol_or_union, y_ref, data);
    if params.is_incremental
        xini = data.model.O_pinv * (data.yini);
        optim_u = sol.feval([xini; uu; y_ref], 'primal', 'tiebreak', 'obj');
    else
        xini = data.model(idx).O_pinv * (data.yini - data.model(idx).Cl*data.uini);
        y_ref = y_ref + data.eta;
        optim_u = sol.feval([xini; data.uini(end); min(max(y_ref, params.y_min), params.y_max)], 'primal', 'tiebreak', 'obj');
    end
end



function [sol, idx] = select_sol(sol_or_union, y_ref, data)
    idx = 1;
    if isa(sol_or_union, 'PolyUnion')
        sol = sol_or_union;
        return
    end
    
    y = data.yini(end);
    idx = 1;
    sol = sol_or_union{1};
    for i=1:length(sol_or_union)
        if y >= sol_or_union{i}.region(1) && y <= sol_or_union{i}.region(2)
            sol = sol_or_union{i}.sol;
            idx = i;
            break
        end
    end
end