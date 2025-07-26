

function gym_install()
    % install all libraries that are provided by csbenchlab_gym repository
    
    libs_path = 'libs';
    libs = dir(libs_path);
    disp('Installing gym libraries. This may take a few moments...');
    for i=1:length(libs)
        if startsWith(libs(i).name, '.') || startsWith(libs(i).name, '..')
            continue
        end
        lib_path  = fullfile(libs(i).folder, libs(i).name);
        if is_valid_component_library(lib_path)            
            register_component_library(lib_path);
        end
    end
    disp('Installation complete');
end