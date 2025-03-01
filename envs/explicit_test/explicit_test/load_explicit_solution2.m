try
    get_workspace_variable('explicit_sol2');
    disp('Using preloaded explicit solution');
    return
catch
end

try
    disp('Loading explicit solution...')
    solution = load('explicit_solution2.mat', 'sss').sss;
    assignin('base', "explicit_sol2", solution);
    disp('Loading done.')
catch
    error('Cannot load explicit solution to matlab workspace.')
end

