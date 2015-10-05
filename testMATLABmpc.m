constants;

A = A_d_all;
B = B_d;
C = eye(9);
D = zeros(9,3);


CSTR = ss(A,B,C,D);
CSTR.InputName = {'accel_x', 'accel_y', 'accel_z'};
CSTR.OutputName = 
