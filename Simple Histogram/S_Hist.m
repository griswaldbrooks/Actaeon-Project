clc, clear, cla
hold all
axis auto
%%% Initialization %%%
N = 10;
N_ang = 4;
ang_res = 10;
%p_space_prior = (1/N.^2)*ones(N,N,N_ang);
p_space_post = (1/N.^2)*ones(N,N,N_ang);
landmark = [1, 2]; %x, y
r_pose_act = [5,5,0];
r_pose_est = [5,5,0];
u_t = [0,0];
%%%%%%%%%%%%%%%%%%%%%%

p_space_prior = p_space_post;

for k_x = 1:N
    for k_y = 1:N
        for k_th = 1:N_ang
% %%% Motion Model %%%
%     for i_x = 1:N
%         for i_y = 1:N
%             for i_th = 1:N_ang
% %                p_space(k) = prob_motion_model(x_t, u_t, x_tm1);
%                 p_space_post(k_x,k_y,k_th) = p_space_post(k_x,k_y,k_th) + prob_motion_model([k_x,k_y,k_th*ang_res], u_t, [i_x,i_y,i_th*ang_res])*p_space_prior(i_x,i_y,i_th);
%                 
%             end
%         end
%     end

%%%%%%%%%%%%%%%%%%
    
    k_z = 1;
    z_noise = k_z*rand(1);
    z_sensed = sqrt((r_pose_act(1) - landmark(1)).^2 + (r_pose_act(2) - landmark(2)).^2) + z_noise;
    
%%% Sensor Model %%%
    p_space_post(k_x,k_y,k_th) = prob_sensor_model(z_sensed, [k_x,k_y,k_th*ang_res], landmark)*p_space_post(k_x,k_y,k_th);
%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
%mesh(p_space);
surf(p_space_post);




  
