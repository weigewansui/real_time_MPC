clear
clc
constants;

tmp0 = [0 0 0 1.2 0 0 1.3 0 0 0 0 0]';
z0 = [];
z0 = [z0; tmp0];
Aeq = C;
beq = [];



global H HORIZON barrier C;

  
  T = 0;
  
x0 = [-7.6 0 0 -1 0 0 -20 0 0]';


% start LCM
javaaddpath /usr/local/share/java/lcm.jar
javaaddpath lcmtypes/lcmtypes.jar

lc = lcm.lcm.LCM.getSingleton();

accel_msg = lcmtypes.accel_t();

subscriber = lcm.lcm.MessageAggregator();
lc.subscribe('vicon_state', subscriber);
millis_to_wait = 1000;

% model noise
noise = 0.01*x0;
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
 
 tic
 while(1)
     
     state_msg = subscriber.getNextMessage(millis_to_wait);
     state_data = lcmtypes.vicon_state_t(state_msg.data);
     current_state(1) = state_data.position(1)/1000;
     current_state(2) = state_data.position(2)/1000;
     current_state(3) = state_data.position(3)/1000;
     
     
     % set desired point at 18 meters
     % add minus to set state_data.position(3) negative,
     % substract -18 
     % add 16 to simulate it's at 15-25 meters high in the lab
     
     current_state(3) = -(current_state(3)+simu_base_height) + better_height;
%      velocity = [0 0 0]';
     velocity = state_data.velocity;
     velocity(3) = -velocity(3);
     
     beq(1:9) = A_d_all*[current_state(1);velocity(1);accumu_err_x;current_state(2);velocity(2);accumu_err_y;current_state(3);velocity(3);accumu_err_z];
     
     x0_debug = [current_state(1);velocity(1);accumu_err_x;current_state(2);velocity(2);accumu_err_y;current_state(3);velocity(3);accumu_err_z];
     
     tic;
     while(norm(r) > 1e-3)%norm(r_p) > w_r_p & norm(r_d) > w_d_p)

         fval = z'*H*z;


         HESSIAN = [];
         GRADIENT = [];

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

             GRADIENT = [GRADIENT; d_phi_du1; d_phi_du2; d_phi_du3; d_phi_dx;0;0;d_phi_dy;0;0;d_phi_dz;0;0];
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

             H_u = diag([dd_phi_du1du1 dd_phi_du2du2 dd_phi_du3du3]);
             H_state = [dd_phi_dxdx 0 0 0 0 0 dd_phi_dxdz 0 0;
                                0 0 0 0 0 0 0 0 0;
                                0 0 0 0 0 0 0 0 0;
                                0 0 0 dd_phi_dydy 0 0 dd_phi_dydz 0 0;
                                0 0 0 0 0 0 0 0 0;
                                0 0 0 0 0 0 0 0 0;
                                dd_phi_dxdz dd_phi_dydz 0 0 0 0 dd_phi_dzdz 0 0;
                                0 0 0 0 0 0 0 0 0
                                0 0 0 0 0 0 0 0 0];

              HESSIAN = blkdiag(HESSIAN, H_u, H_state);

         end

        % initialize


         r_d = 2*H*z + mu_all*GRADIENT + C'*v;
         r_p = C*z - beq;

         r = [r_d;r_p];


         inv_PHI= inv(2*H + mu_all*HESSIAN);
         Y = C*inv_PHI*C';
         beta1 = -r_p + C*inv_PHI*r_d;
         
         d_v = -inv(Y)*beta1;
         d_z = inv_PHI*(-r_d - C'*d_v);


        %  get step size via line search
        %  initialize line search step
        beta = 0.5;
        d = 1;
        while (norm([(2*H*(z + d*d_z) +mu_all* GRADIENT + C'* (v + d*d_v)); (C*(z + d*d_z)-beq)]) > (1 - 0.4*d)*norm(r))
            d = beta*d;
        end
        
        % update z and v
        z = z + d*d_z;
        v = v + d*d_v;
        
        norm(r)
     end

    toc
     r = 10e6*r;
     accel_msg.accel = z(1:3);
     lc.publish('acceleration_MATLAB', accel_msg);
     
     accumu_err_x = accumu_err_x + current_state(1)*dT;
     accumu_err_y = accumu_err_y + current_state(2)*dT;
     accumu_err_y = accumu_err_y + current_state(3)*dT;
    
    z_tmp = z(1:12);
    
    for ii = 1: HORIZON - 1
            
        z((1+(ii - 1)*12):(12 + (ii - 1)*12)) = z((1+ ii * 12):(12 + ii * 12));
        
    end
    
    z((1 + (HORIZON - 1)*12) : (12 + (HORIZON - 1)*12)) = z_tmp(1:12);
     
     pause(0.01)

 end
  
%   U = norm([z(1);z(2);z(3)-9.8]);
%   theta = asin(-u_result(1)/U);
%   phi = asin(u_result(2)/(U*cos(theta)));
%  t = toc
% z_min = z;
% plot_figure
