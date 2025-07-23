function sys = getBoeing747Model()
%GETBOEING747MODEL Returns the Boeing 747 state-space model.
%
%   sys = GETBOEING747MODEL() returns a struct with fields A, B, C, D 
%   containing the state-space matrices of a Boeing 747 aircraft model.
%
%   Source:
%   P.C.N. Verheijen, V. Breschi, M. Lazar,
%   "Handbook of linear data-driven predictive control: Theory, implementation and design,"
%   Annual Reviews in Control, Volume 56, 2023, 100914.

    sys = struct;

    sys.A = [ 0.9997  0.0038  -0.0001  -0.0322;
             -0.0056  0.9648   0.7446   0.0001;
              0.0020 -0.0097   0.9543  -0.0000;
              0.0001 -0.0005   0.0978   1.0000 ];

    sys.B = [  0.0010   0.1000;
              -0.0615   0.0183;
              -0.1133   0.0586;
              -0.0057   0.0029 ];

    sys.C = [  1.0000   0        0       0;
               0      -1.0000   0       7.7400 ];

    sys.D = zeros(size(sys.C, 1), size(sys.B, 2));
end