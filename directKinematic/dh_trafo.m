function D_vi = dh_trafo(alpha, a, d, theta)
    % alpha(i-1), a(i-1), d(i), theta(i): DH-Parameter
    % D_vi ist homogene Transformationsmatrix von i -> i-1

    % DH-Matrix

    D1 = [cos(theta) -sin(theta) 0  0;
          sin(theta) cos(theta)  0  0;
          0            0         1  0;
          0            0         0  1];
      
    D2 = [1  0  0  0;
          0  1  0  0;
          0  0  1  d;
          0  0  0  1];
      
    D3 = [1  0  0  a;
          0  1  0  0;
          0  0  1  0;
          0  0  0  1];
      
    D4 = [1   0             0        0;
          0 cos(alpha)  -sin(alpha)  0;
          0 sin(alpha)   cos(alpha)  0;
          0   0             0        1];
        
    D_vi = D4*D3*D2*D1;

end
