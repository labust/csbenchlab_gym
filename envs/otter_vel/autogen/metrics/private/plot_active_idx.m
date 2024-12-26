function plot_active_idx(out, varargin)
    
    f = figure;
    idx = out.log.deepcTO.log.db_active_idx;
    
    plot(idx.Time, idx.Data(1, :));
end

