function otter_vel_run()
    file_path = matlab.desktop.editor.getActiveFilename;
    env_path = fileparts(file_path);
    [~, env_name] = fileparts(env_path);
    run(fullfile(env_path, strcat(env_name, '_source.m')));
    env_manager_app(env_name, env_path);
end

