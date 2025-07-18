function optim_u = evaluate_explicit(sol_or_union, y_ref, uu, data, params)
   
    [sol, idx] = select_sol(sol_or_union, y_ref, data);
    if params.is_incremental
        xini = data.O_pinv * (data.yini);
        % optim_u = this.sol.feval([xini; this.uu], 'primal', 'tiebreak', 'obj');
        explicit_eval()
        optim_u = sol.feval([xini; uu; y_ref], 'primal', 'tiebreak', 'obj');
    else
        % xini = data.O_pinv * (data.yini);
        xini = data.O_pinv * (data.yini - data.Cl*data.uini);
        optim_u = sol.feval([xini(end); data.uini(end); y_ref], 'primal', 'tiebreak', 'obj');
    end
end



function [sol, idx] = select_sol(sol_or_union, y_ref, data)
    idx = -1;
    if isa(sol_or_union, 'PolyUnion')
        sol = sol_or_union;
        return
    end
    
    y = data.yini(end);
    idx = 1;
    sol = sol_or_union(1).sol;
    for i=1:length(sol_or_union)
        if y >= sol_or_union(i).range(1) && y <= sol_or_union(i).range(2)
            sol = sol_or_union(i).sol;
            idx = i;
            break
        end
    end
end