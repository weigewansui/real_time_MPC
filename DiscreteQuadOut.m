function fval = DiscreteQuadOut(z)

 global H HORIZON barrier;
 
 fval = z'*H*z;
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

 
 for i = 1 : HORIZON
    
% barrier 2

%     if (barrier == 2)
%         u1 = z(1 + 12*(i-1));
%         u2 = z(2 + 12*(i-1));
%         u3 = z(3 + 12*(i-1));
%         u_barrier = -log(u1 - u1_min) - log(u1_max - u1) - log(u2 - u2_min) - log(u2_max - u2) - log(u3 - u3_min) - log(u3_max - u3);
%         x = z (4 + (i - 1)*12);
%         y = z (7 + (i - 1)*12);
%         z_h = z (10 + (i - 1)*12);
% 
%         h = better_height - z_h;
%         state_barrier  = -log(h*tan_half_phi - x) - log(x + h*tan_half_phi) - log(h*tan_half_psi - y) - log(y + h*tan_half_psi) ...
%             -log(z_h - z_min) - log(z_max - z_h);
%         
%         fval = fval + mu_all*u_barrier;
%         fval = fval + mu_all*state_barrier;
%     end
 end
 
    
end