function plot_changed(out, plot_cfg, f_handle)

    if exist('f_handle', 'var')
        set(0, 'CurrentFigure', f_handle);
        hold on;
    end



    changed = out.log.deepcTOl.log.db_active_idx.Data(1, :);
    changed = changed(2:end) - changed(1:end-1);

    % changed(190:250) = 0;
    % changed(600:700) = 0;
    % changed(350:550) = 0;

    plot(out.log.deepcTOl.log.db_active_idx.Time(1:end-1), changed*3, 'LineWidth', 1.5);
    legend(plot_cfg.Legend);
end

