global A_d_all C H HORIZON dT barrier;

% settings
barrier = 2;
TEST = 0;


dT = 0.1;
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
Q_end = diag([0.1 0.1 0.01 0.1 0.1 0.001 0.001 0.001 0.0001]);
%  Q = diag([1 1 0.01 1 1 0.01 0.01 0.01 0.01]);
Q = diag([0.1 0.1 0.001 0.1 0.1 0.001 0.001 0.001 0.0001]);
%  Q_end = Q;



 
 u1_min = -5;
 u1_max = 5;
 u2_min = -5;
 u2_max = 5;
 u3_min = -5;
 u3_max = 7;

 z_max = 5;
 z_min = -5;
 
 mu_u =0.6;
 mu_state = 10;
 tan_half_phi = 0.5;
 tan_half_psi = 0.5;
  
 better_height = 17;
 simu_base_height = 16;
 
 mu_all = 0.01;
% mu_all = 0.1;
  
 
Fu = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
Fx = [1 0 0 0 0 0 tan_half_phi 0 0;
     -1 0 0 0 0 0 tan_half_phi 0 0;
      0 0 0 1 0 0 tan_half_psi 0 0;
      0 0 0 -1 0 0 tan_half_psi 0 0;
      0 0 0 0 0 0 1 0 0;
      0 0 0 0 0 0 -1 0 0
    ];

fu = [u1_max;-u1_min;u2_max;-u2_min;u3_max;-u3_min];
% fx = [0;0;0;0;z_max;-z_min];
    fx = [-better_height*tan_half_phi; -better_height*tan_half_phi; -better_height*tan_half_psi; -better_height*tan_half_psi;z_max;-z_min];

 H = [];
 C = [];
 P = [];
 h_constraint = [];
 for i = 1:HORIZON
     
     P = blkdiag(P,Fu,Fx);
     h_constraint = [h_constraint;fu;fx];
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

num_of_P_rows = 12*HORIZON;
 
 
MAX_INTERATION_NUM = 1e2;

x_des = 0;
y_des = 0;
z_des = 0;

