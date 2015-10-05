clear
clc
constants;

tmp0 = [0 0 0 1.2 0 0 1.3 0 0 0 0 0]';
z0 = [];
z0 = [z0; tmp0];
Aeq = C;
beq = [];
time_limit = 0.15;

global H HORIZON barrier C;

  
  T = 0;
  
x0 = [-7.6 0 0 -1 0 0 -20 0 0]';


% start LCM
javaaddpath /usr/local/share/java/lcm.jar
javaaddpath lcmtypes/lcmtypes.jar

lc = lcm.lcm.LCM.getSingleton();

accel_msg = lcmtypes.accel_t();

subscriber = lcm.lcm.MessageAggregator();
lc.subscribe('kalman_v', subscriber);
millis_to_wait = 1000;

% model noise
noise = 0.0*x0;
 beq = [beq; A_d_all*x0+noise];
 
for i = 2:HORIZON
    beq = [beq; noise];
    z0 = [z0;tmp0];
end

v0 = 0.05* randi([0 20],9*HORIZON,1);
 
%  initialize the loop


 z = z0;
 v = v0;

 r = 10;
%  r_d = 10;
 
 w_r_p = 1e-4;
 w_d_p = 1e-4;
 
 accumu_err_x = 0;
 accumu_err_y = 0;
 accumu_err_z = 0;
 d_GRADIENT = zeros(num_of_P_rows,1);

 z_good = z0;
 
 HESSIAN = zeros(12*HORIZON, 12*HORIZON);
 GRADIENT = zeros(12*HORIZON, 1);
 
 prev_ax = 0;
 prev_ay = 0;
 prev_az = 0;
 
 while(1)
     

     state_msg = subscriber.getNextMessage(millis_to_wait);
     state_data = lcmtypes.kalman_v_t(state_msg.data);
     current_state(1) = state_data.pos(1);
     current_state(2) = state_data.pos(2);
     current_state(3) = state_data.pos(3);
     
     noise = B_d*[state_data.accel(1) - prev_ax; state_data.accel(2) - prev_ay; state_data.accel(3) - prev_az];
     
     % set desired point at 18 meters
     % add minus to set state_data.position(3) negative,
     % substract -18 
     % add 16 to simulate it's at 15-25 meters high in the lab
     
     current_state(3) = -(current_state(3)+simu_base_height) + better_height;
%      velocity = [0 0 0]';
     velocity = state_data.vel;
     velocity(3) = -velocity(3);
     
     beq(1:9) = A_d_all*[current_state(1);velocity(1);accumu_err_x;current_state(2);velocity(2);accumu_err_y;current_state(3);velocity(3);accumu_err_z] + noise;
     for i = 2 : HORIZON
         beq((1 + (i - 1)*9) : (9 + (i - 1)*9)) = noise;
     end
     
     x0_debug = [current_state(1);velocity(1);accumu_err_x;current_state(2);velocity(2);accumu_err_y;current_state(3);velocity(3);accumu_err_z];
     
     iteration_count = 0;
%      t_get_info = toc
         tic
     while(norm(r) > 1e-3)%norm(r_p) > w_r_p & norm(r_d) > w_d_p)
        
         t = toc;
         if t > time_limit
             
%              accel_msg.accel(1) = (current_state(1) - x_des)*(-2.4142135) + (velocity(1))*(-2.210);
%              accel_msg.accel(2) = (current_state(2) - x_des)*(-2.4142135) + (velocity(2))*(-2.210);
%              accel_msg.accel(3) = (current_state(3) - x_des)*(-2.4142135) + (velocity(3))*(-2.210);
%              lc.publish('acceleration_MATLAB', accel_msg);
              
             break;
         end
    
         for i = 1 : HORIZON

         x = z (4 + (i - 1)*12);
         y = z (7 + (i - 1)*12);
         z_h = z (10 + (i - 1)*12);
         u1 = z(1 + 12*(i-1));
         u2 = z(2 + 12*(i-1));
         u3 = z(3 + 12*(i-1));
         h = -z_h + better_height;
        %     stack up gradient
             tmp_denum_grad_x_1 = h*tan_half_phi - x;
             tmp_denum_grad_x_2 = h*tan_half_phi + x;

             tmp_denum_grad_y_1 = h*tan_half_psi - y;
             tmp_denum_grad_y_2 = h*tan_half_psi + y;

             d_phi_dx = 1/tmp_denum_grad_x_1 - 1/tmp_denum_grad_x_2;
             d_phi_dy = 1/tmp_denum_grad_y_1 - 1/tmp_denum_grad_y_2;
             d_phi_dz = 1/(z_max - z_h) - 1/(z_h - z_min) + tan_half_phi/tmp_denum_grad_x_1 + tan_half_phi/tmp_denum_grad_x_2 ...
                 + tan_half_psi/tmp_denum_grad_y_1 + tan_half_psi/tmp_denum_grad_y_2;

             d_phi_du1 = 1/(u1_max - u1) - 1/(u1 - u1_min);
             d_phi_du2 = 1/(u2_max - u2) - 1/(u2 - u2_min);
             d_phi_du3 = 1/(u3_max - u3) - 1/(u3 - u3_min);

%              GRADIENT = [GRADIENT; mu_u*d_phi_du1; mu_u*d_phi_du2; mu_u*d_phi_du3; mu_state*d_phi_dx;0;0;mu_state*d_phi_dy;0;0;mu_state*d_phi_dz;0;0];
             GRADIENT(1 + 12*(i-1)) = d_phi_du1;
             GRADIENT(2 + 12*(i-1)) = d_phi_du2;
             GRADIENT(3 + 12*(i-1)) = d_phi_du3;
             GRADIENT(4 + 12*(i-1)) = d_phi_dx;
             GRADIENT(7 + 12*(i-1)) = d_phi_dy;
             GRADIENT(10 + 12*(i-1)) = d_phi_dz;
             
             % stack up hessian
             tmp_denum_x_1 = (h*tan_half_phi - x)^2;
             tmp_denum_x_2 = (h*tan_half_phi + x)^2;

             tmp_denum_y_1 = (h*tan_half_psi - y)^2;
             tmp_denum_y_2 = (h*tan_half_psi + y)^2;

             dd_phi_dxdx = 1/tmp_denum_x_1 + 1/tmp_denum_x_2;
             dd_phi_dxdz = tan_half_phi/tmp_denum_x_1 - tan_half_phi/tmp_denum_x_2;

             dd_phi_dydy = 1/tmp_denum_y_1 + 1/tmp_denum_y_2;
             dd_phi_dydz = tan_half_psi/tmp_denum_y_1 - tan_half_psi/tmp_denum_y_2;

             dd_phi_dzdz = 1/(z_max - z_h)^2 + 1/(z_h - z_min)^2 + tan_half_phi^2/tmp_denum_x_1 ...
                 + tan_half_phi^2/tmp_denum_x_2 + tan_half_psi^2/tmp_denum_y_1 + tan_half_psi^2/tmp_denum_y_2;

             dd_phi_du1du1 = 1/(u1_max - u1)^2 + 1/(u1 - u1_min)^2;
             dd_phi_du2du2 = 1/(u2_max - u2)^2 + 1/(u2 - u2_min)^2;
             dd_phi_du3du3 = 1/(u3_max - u3)^2 + 1/(u3 - u3_min)^2;

%              H_u = diag([dd_phi_du1du1 dd_phi_du2du2 dd_phi_du3du3]);
%              H_state = [dd_phi_dxdx 0 0 0 0 0 dd_phi_dxdz 0 0;
%                                 0 0 0 0 0 0 0 0 0;
%                                 0 0 0 0 0 0 0 0 0;
%                                 0 0 0 dd_phi_dydy 0 0 dd_phi_dydz 0 0;
%                                 0 0 0 0 0 0 0 0 0;
%                                 0 0 0 0 0 0 0 0 0;
%                                 dd_phi_dxdz dd_phi_dydz 0 0 0 0 dd_phi_dzdz 0 0;
%                                 0 0 0 0 0 0 0 0 0
%                                 0 0 0 0 0 0 0 0 0];

%               HESSIAN = blkdiag(HESSIAN, mu_u*H_u, mu_state*H_state);
              
              HESSIAN(1+12*(i-1),1+12*(i-1) ) = dd_phi_du1du1;
              HESSIAN(2+12*(i-1),2+12*(i-1) ) = dd_phi_du2du2;
              HESSIAN(3+12*(i-1),3+12*(i-1) ) = dd_phi_du3du3;
              
              HESSIAN(4+12*(i-1),4+12*(i-1) ) = dd_phi_dxdx;
              HESSIAN(4+12*(i-1),10+12*(i-1) ) = dd_phi_dxdz;

              HESSIAN(7+12*(i-1),7+12*(i-1) ) = dd_phi_dydy;
              HESSIAN(7+12*(i-1),10+12*(i-1) ) = dd_phi_dydz;
              
              HESSIAN(10+12*(i-1),10+12*(i-1) ) = dd_phi_dzdz;
              HESSIAN(10+12*(i-1),4+12*(i-1) ) = dd_phi_dxdz;
              HESSIAN(10+12*(i-1),7+12*(i-1) ) = dd_phi_dydz;
              
         end
        % initialize


         r_d = 2*H*z + GRADIENT + C'*v;
         r_p = C*z - beq;

         r = [r_d;r_p];


         inv_PHI= inv(2*H + HESSIAN);
         Y = C*inv_PHI*C';
         beta1 = -r_p + C*inv_PHI*r_d;
         
         d_v = -inv(Y)*beta1;
         d_z = inv_PHI*(-r_d - C'*d_v);
%             
%     ALL = [2*H+mu_all*HESSIAN, C'; C zeros(9*HORIZON,9*HORIZON)];
%     d_ALL = -inv(ALL)*r;
%     d_z = d_ALL(1:12*HORIZON);
%     d_v = d_ALL((12*HORIZON+1):(21*HORIZON));

    
        %  get step size via line search
        %  initialize line search step
        beta = 0.7;
        d = 1;
        while (norm([(2*H*(z + d*d_z) + GRADIENT + C'* (v + d*d_v)); (C*(z + d*d_z)-beq)]) > norm(r) + (1 - 0.4*d)*norm(r))
            d = beta*d;
        end
        
        % update z and v
        z = z + d*d_z;
        v = v + d*d_v;
     end

     r = 10e6*r;
     accel_msg.accel(1) = z(1);
     accel_msg.accel(2) = z(2);
     accel_msg.accel(3) = z(3);
     
   
     lc.publish('acceleration', accel_msg);
     
     prev_ax = z(1);
     prev_ay = z(2);
     prev_az = z(3);
     
     accumu_err_x = accumu_err_x + current_state(1)*dT;
     accumu_err_y = accumu_err_y + current_state(2)*dT;
     accumu_err_y = accumu_err_y + current_state(3)*dT;

    if( z(1) > u1_max | z(1) < u1_min | z(2) > u2_max | z(2) < u2_min | z(3) > u3_max | z(3) < u3_min )
        z = z_good;
    else
        z_good = z;
    end
    
    z_tmp = z(1:12);
    
    for ii = 1: HORIZON - 1
            
        z((1+(ii - 1)*12):(12 + (ii - 1)*12)) = z((1+ ii * 12):(12 + ii * 12));
        
    end
    
    z((1 + (HORIZON - 1)*12) : (12 + (HORIZON - 1)*12)) = z_tmp(1:12);
    
    
%     if( z(1) > u1_max | z(1) < u1_min | z(2) > u2_max | z(2) < u2_min | z(3) > u3_max | z(3) < u3_min )
%         
%         z((1 + (HORIZON - 1)*12) : (12 + (HORIZON - 1)*12)) = zeros(12,1);
%     else
%         
%     end

     
 end