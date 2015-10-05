fname = 'data/imu_19020455.txt';

data = importdata(fname);
accel_x_bias = mean(data(:,2),'omitnan' )
accel_y_bias = mean(data(:,3),'omitnan' )
accel_z_bias = mean(data(:,4),'omitnan' )

data(:,2) = (data(:,2) - accel_x_bias)/1000; 
data(:,3) = (data(:,3) - accel_y_bias)/1000; 
data(:,4) = (data(:,4) - accel_z_bias)/1000; 

Covariance_x = cov(data(:,2)) 

Covariance_y = cov(data(:,3)) 
3.3562e-04
Covariance_z = cov(data(:,4)) 