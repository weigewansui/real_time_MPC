global A_d_all C H HORIZON dT barrier;

% settings
barrier = 2;
TEST = 0;


dT = 0.01;
A_d_triple = [1 dT 0; 0 1 0; dT dT^2/2 1];

HORIZON = 10;

%-------------------------------------
 
ZERO_MATRIX = zeros(3,3);
% A_d_all = [A_d_triple, ZERO_MATRIX, ZERO_MATRIX; ZERO_MATRIX, A_d_triple, ZERO_MATRIX; ZERO_MATRIX, ZERO_MATRIX, A_d_triple];
A_d_all = blkdiag(A_d_triple, A_d_triple, A_d_triple);

% B_d = [0 0 0; 
%        1 0 0;
%        0 0 0;
%        0 0 0;
%        0 1 0;
%        0 0 0;
%        0 0 0;
%        0 0 1;
%        0 0 0
%        ];

B_one_state = [dT^2/2; dT; dT^3/6];

B_d = blkdiag(B_one_state, B_one_state, B_one_state);
R = eye(3)*0.001;
 %%%%
Q_end = diag([1 0.2 0.01 1 1 0.01 0.01 0.01 0.001]);
%  Q = diag([1 1 0.01 1 1 0.01 0.01 0.01 0.01]);
Q = diag([1 0.2 0.01 1 1 0.01 0.01 0.01 0.0001]);
%  Q_end = Q;
 

 H = [];
 C = [];
 for i = 1:HORIZON
     if(i ~= HORIZON)
        H = blkdiag(H,R,Q);
     else
         H = blkdiag(H,R,Q_end);
     end
     if(i == 1)
        C = [C; -B_d, eye(9), zeros(9, 12*(HORIZON - 1))];
     
     elseif(i == HORIZON)
         C = [C;zeros(9,3), zeros(9, 12*(i - 2)), -A_d_all -B_d eye(9)];
     elseif(i == 2)
         C = [C; zeros(9,3), -A_d_all, -B_d, eye(9), zeros(9, 12*(HORIZON - 2) )];
     else
         C = [C; zeros(9, 3), zeros(9, (i-2)*12), -A_d_all, -B_d, eye(9),zeros(9, 12*(HORIZON - i))];
     end
     
 end


 
 u1_min = -5;
 u1_max = 5;
 u2_min = -5;
 u2_max = 5;
 u3_min = -5;
 u3_max = 7;

 z_max = 5;
 z_min = -5;
 
 mu_u = 0.01;
 mu_state = 0.1;
 tan_half_phi = 0.5;
 tan_half_psi = 0.5;
  
  better_height = 17;
  simu_base_height = 18;
  mu_all = 0.01;
 
 


