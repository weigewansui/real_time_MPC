state_fname = 'data/2/vicon_state_16103440.txt';
states = importdata(state_fname);
format long;

Timestamp_tmp = states(:,1);
Time0 = Timestamp_tmp(1);

Timestamp_tmp = (Timestamp_tmp - Time0)/1e6;
x_pos_tmp = states(:,2);
y_pos_tmp = states(:,3);
z_pos_tmp = states(:,4);

x_vel_tmp = states(:,5);
y_vel_tmp = states(:,6);
z_vel_tmp = states(:,7);

Timestamp = [];
x_pos = [];
y_pos = [];
z_pos = [];

x_vel = [];
y_vel = [];
z_vel = [];

for i = 1:length(Timestamp_tmp)
    if Timestamp_tmp(i) < 10000
        Timestamp = [Timestamp;Timestamp_tmp(i)];
        x_pos = [x_pos;x_pos_tmp(i)];
        y_pos = [y_pos;y_pos_tmp(i)];
        z_pos = [z_pos;z_pos_tmp(i)];
        x_vel = [x_vel;x_vel_tmp(i)];
        y_vel = [y_vel;y_vel_tmp(i)];
        z_vel = [z_vel;z_vel_tmp(i)];
    end
end

plot3(x_pos,y_pos,z_pos)
