function [O, Cl] = construct_oc_matrices(A, B, C, D, L)

    O = C;
    Cl = D;
    
    a = A;
    m = size(B, 2);
    p = size(C, 1);
    h = C * B;
    for i=2:L
        O = [O; C * a];
        
        Cl = [Cl zeros(height(Cl), m); zeros(p, width(Cl)), D];
        Cl(end-p+1:end, m+1:(i*m)) = Cl(end-2*p+1:end-p, 1:((i-1)*m));
        Cl(end-p+1:end, 1:m) = h;
        h = C * a * B;
        a = a*A;
    end
   
end