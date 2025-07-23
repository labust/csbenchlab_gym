function sys = getLinearCrazyflieModel()
%GETLINEARCRAZYFLIEMODEL Returns the linearized Crazyflie state-space model.
%
%   sys = GETLINEARCRAZYFLIEMODEL() returns a struct with fields A, B, C, D 
%   containing the state-space matrices of a linearized Crazyflie quadrotor model.

    sys = struct;

    sys.A = [ 0     0     0     1.0000     0     0     0     0     0;
              0     0     0     0     1.0000     0     0     0     0;
              0     0     0     0     0     1.0000     0     0     0;
              0     0     0     0     0     0     0     9.8100     0;
              0     0     0     0     0     0    -9.8100     0     0;
              0     0     0     0     0     0     0     0     0;
              0     0     0     0     0     0     0     0     0;
              0     0     0     0     0     0     0     0     0;
              0     0     0     0     0     0     0     0     0 ];

    sys.B = [  0         0         0         0;
               0         0         0         0;
               0         0         0         0;
               0         0         0         0;
               0         0         0         0;
              35.7143    0         0         0;
               0         1.0000    0         0;
               0         0         1.0000    0;
               0         0         0         1.0000 ];

    sys.C = eye(9);  % 9x9 identity matrix (outputs all states)

    sys.D = zeros(9, 4);  % 9 outputs, 4 inputs
end