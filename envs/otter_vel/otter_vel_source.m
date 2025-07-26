function otter_vel_source()
    file_path = matlab.desktop.editor.getActiveFilename;
    env_path = fileparts(file_path);
    [~, env_name] = fileparts(env_path);
    source_environment(env_path, env_name);
end

