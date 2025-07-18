function optim_u = evaluate_explicit_codegen(y_ref, uu, data, params)

    xini = data.O_pinv * (data.yini - data.Cl*data.uini);
    optim_u = explicit_eval([xini(end); data.uini(end); y_ref], params.PH, params.PHe, params.PHidx, params.F, params.g);
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