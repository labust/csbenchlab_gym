function idx = select_current_datasets(db_u, db_y, db_num_datasets, prev_idx, traj_u, traj_y, params)
    
    idx = prev_idx;
    if params.dataset_mode == 0 % do nothing mode
        return
    end

    if db_num_datasets == 1
        return 
    end

    if params.dataset_mode == 1 % select latest datasets
        idx = db_num_datasets;
        return
    end

    m = size(traj_u, 2);
    errs = ones(height(db_u), m) * 100000;
    if params.dataset_mode == 2 % select closest datasets
        for i=1:db_num_datasets
            test_traj = squeeze(db_y(end-i+1, :, :));
            errs(end-i+1) = sum(abs(test_traj - traj_y), 'all');
        end
        [~, idx] = mink(errs, 1);

        idx = height(db_u) - idx(end) + 1;
    end

    
end

