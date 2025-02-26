

db_params = load('db_params.mat', 'db_params');
db_params = db_params.db_params;
db_params.sigma_min = 20;
assignin('base', "db_params", db_params);
