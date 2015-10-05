function M = DCM(phi, theta, psi)
    M = zeros(3,3);
    C_bn = [1 0 0;
        0 cos(phi) sin(phi);
        0 -sin(phi) cos(phi)];

    C_nm = [cos(theta) 0 -sin(theta);
        0 1 0;
        sin(theta) 0 cos(theta)];

    C_mg = [cos(psi) sin(psi) 0;
        -sin(psi) cos(psi) 0;
        0 0 1];
    M = C_bn*C_nm*C_mg*;
end