function plot_changed(out, plot_cfg, f_handle)

    if exist('f_handle', 'var')
        set(0, 'CurrentFigure', f_handle);
        hold on;
    end
    
    if ~isfield(out.log, 'DeePC')
        return
    end


    changed = out.log.DeePC.log.idx.Data;
    changed = changed(2:end) - changed(1:end-1);

    % changed(190:250) = 0;
    % changed(600:700) = 0;
    % changed(350:550) = 0;

    plot(out.log.DeePC.log.idx.Time(2:end), changed*3, 'LineWidth', 1.5);
    if is_valid_field(plot_cfg, 'Legend')
        legend(plot_cfg.Legend);
    end
end

