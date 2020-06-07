close all
disp('============================')
load('IMUdata.mat');
addpath(genpath('matlab_utilities/'));
addpath(genpath('matlab-utils/'));
% %DATA EXTRACTION AND INITIALIZATION% %

% IMU sensor time (s)
t_imu     = bagdata.imu.data.t;
% IMU accelerometer measurements about axes (m/s^2)
x_acc_imu = bagdata.imu.data.acc(1,:);
y_acc_imu = bagdata.imu.data.acc(2,:);
z_acc_imu = bagdata.imu.data.acc(3,:);
% IMU gyro measurements about axes (rad/s)
x_gyr_imu = bagdata.imu.data.gyro(1,:);
y_gyr_imu = bagdata.imu.data.gyro(2,:);
z_gyr_imu = bagdata.imu.data.gyro(3,:);

% IMU bias estimates time (s)
t_imub     = bagdata.imu_bias.t;
% IMU accelerometer bias estimates about axes (m/s^2)
x_acc_imub = bagdata.imu_bias.acc(1,:);
y_acc_imub = bagdata.imu_bias.acc(2,:);
z_acc_imub = bagdata.imu_bias.acc(3,:);
% IMU gyro bias estimates about axes (rad/s)
x_gyr_imub = bagdata.imu_bias.gyro(1,:);
y_gyr_imub = bagdata.imu_bias.gyro(2,:);
z_gyr_imub = bagdata.imu_bias.gyro(3,:);

% ROSflight command time (s)
t_com = bagdata.command.t;
% ROSflight throttle command anything in the range [0-1]
F_com = bagdata.command.F;

% Truth state time (s)
t_tru = bagdata.odometry.t;
% North position (m)
N_tru = bagdata.odometry.pose.position(1,:);
% East position (m)
E_tru = bagdata.odometry.pose.position(2,:);
% Down position (m)
D_tru = bagdata.odometry.pose.position(3,:);
% Body-x velocity (m/s)
u_tru = bagdata.odometry.twist.linear(1,:);
% Body-y velocity (m/s)
v_tru = bagdata.odometry.twist.linear(2,:);
% Body-z velocity (m/s)
w_tru = bagdata.odometry.twist.linear(3,:);
% Roll (phi), Pitch (tht), Yaw (psi) (rad)
[phi_tru, tht_tru, psi_tru] = quat_to_euler(bagdata.odometry.pose.orientation(4,:),...
                                bagdata.odometry.pose.orientation(1,:),...
                                bagdata.odometry.pose.orientation(2,:),...
                                bagdata.odometry.pose.orientation(3,:));
                            
% Angular velocities about x, y, z axes (rad/s)
p_tru   = bagdata.odometry.twist.angular(1,:);
q_tru   = bagdata.odometry.twist.angular(2,:);
r_tru   = bagdata.odometry.twist.angular(3,:);

% Regularize time
t0 = min([t_tru t_com t_imub t_imu]);
t_tru  = t_tru - t0;
t_com  = t_com - t0;
t_imu  = t_imu - t0;
t_imub = t_imub - t0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%DATA PROCESSING%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%No arrays are currently preallocated, consider doing this to improve speed
% %This Section plots relevant position and velocity data% %

%% Position and Velocity Plots

%This graphs the position and path the drone took during its flight
figure
plot3(N_tru,E_tru,-D_tru) %Made Down negative to plot the drone as it would actually be flying
xlabel('North Position (Meters)')
ylabel('East Position (Meters)')
zlabel('Up Position (Meters)')
title('3D Position Vector')
legend('Drone Path')


% %Produces a singular plot of respective velocities
figure
plot(t_tru,u_tru)
hold on
plot(t_tru,v_tru)
plot(t_tru,w_tru)
hold off
xlabel('Time')
ylabel('Velocity')
title('Three Dimensional Velocity')
legend('X-Velocity','Y-Velocity','Z-Velocity')

%% Phi and Theta Attitude

% %This section analyzes the phi and theta attitude measurements from the IMU% %

%Calculates phi and theta from the IMU accelerometer readings using
%Andrew's quaternion conversion function
phi_imu = [];
tht_imu = [];
for i=1:length(x_acc_imu)
    grav_vector = [x_acc_imu(i); y_acc_imu(i); z_acc_imu(i)];
    grav_vector = grav_vector/norm(grav_vector); %Gravity Vector needed to be normalized for quatd function
    q = Quatd_from_two_unit_vectors([0; 0; -1],grav_vector).inverse(); %Had to invert quatd fuction
    phi_imu = [phi_imu; q.roll()];
    tht_imu = [tht_imu; q.pitch()];
end
phi_imu = phi_imu.';
tht_imu = tht_imu.';

%These error functions only calculate to the end of the smaller array of
%data (IMU or True)

%This calls a custom RMSE function to return RMSE for phi attitude along
%with the custon noise function that returns noise STD and bias terms
RMSE_phi = Root_error(phi_imu, phi_tru)
[phi_std_thrust_on, phi_std_thrust_off, phi_bias_vect] = noise_calc(phi_imu, phi_tru, t_tru, t_imu, t_com, F_com);
phi_std_thrust_on
phi_std_thrust_off

%This calls a custom RMSE function to return RMSE for theta attitude along
%with the custon noise function that returns noise STD and bias terms
RMSE_tht = Root_error(tht_imu, tht_tru)
[tht_std_thrust_on, tht_std_thrust_off, tht_bias_vect] = noise_calc(tht_imu, tht_tru, t_tru, t_imu, t_com, F_com);
tht_std_thrust_on
tht_std_thrust_off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Plots the IMU and true readings for phi attitude and imposes the RMSE
%value on the plot
figure
plot(t_tru, phi_tru)
hold on
plot(t_imu, phi_imu)
hold off
xlabel('Time')
ylabel('Radians')
title('Phi IMU vs True')
legend('Phi True','Phi IMU')
txt = ['RMSE = ' num2str(RMSE_phi)];
text(20,2,txt)

%Plots the IMU and true readings for theta attitude and imposes the RMSE
%value on the plot
figure
plot(t_tru, tht_tru)
hold on
plot(t_imu, tht_imu)
hold off
xlabel('Time')
ylabel('Radians')
title('Theta IMU vs True')
legend('Theta True','Theta IMU')
txt = ['RMSE = ' num2str(RMSE_tht)];
text(20,1,txt)

%% Gyroscope X, Y, and Z

disp('Gyroscope Values')
disp('============================')

% %This section analyzes the IMU gyroscope data% %

%Calculates RMSE and calls noise function for each direction of gyroscope
RMSE_x_gyr = Root_error(x_gyr_imu, p_tru)
[x_gyr_std_thrust_on, x_gyr_std_thrust_off, x_gyr_bias_vect] = noise_calc(x_gyr_imu, p_tru, t_tru, t_imu, t_com, F_com);
RMSE_y_gyr = Root_error(y_gyr_imu, q_tru)
[y_gyr_std_thrust_on, y_gyr_std_thrust_off, y_gyr_bias_vect] = noise_calc(y_gyr_imu, q_tru, t_tru, t_imu, t_com, F_com);
RMSE_z_gyr = Root_error(z_gyr_imu, r_tru)
[z_gyr_std_thrust_on, z_gyr_std_thrust_off, z_gyr_bias_vect] = noise_calc(z_gyr_imu, r_tru, t_tru, t_imu, t_com, F_com);

%Displays STD vectors for gyroscope note: -10 displayed if too few data 
%points to approximate gaussian in STD calculation
x_gyr_std_thrust_on
x_gyr_std_thrust_off
y_gyr_std_thrust_on
y_gyr_std_thrust_off
z_gyr_std_thrust_on
z_gyr_std_thrust_off

%Plots X-Gyroscope vs True Data
figure
plot(t_imu,x_gyr_imu)
hold on
plot(t_tru,p_tru)
hold off
xlabel('Time')
ylabel('Radians per Second')
title('X-Gyroscope/True')
legend('X-gyro IMU','X-gyro True')
txt = ['RMSE = ' num2str(RMSE_x_gyr)];
text(15,5,txt)

%Plots Y-Gyroscope vs True Data
figure
plot(t_imu,y_gyr_imu)
hold on
plot(t_tru,q_tru)
hold off
xlabel('Time')
ylabel('Radians/Second')
title('Y-Gyroscope vs True')
legend('Y-gyro IMU','Y-gyro True')
txt = ['RMSE = ' num2str(RMSE_y_gyr)];
text(15,5,txt)

%Plots Z-Gyroscope vs True Data
figure
plot(t_imu,z_gyr_imu)
hold on
plot(t_tru,r_tru)
hold off
xlabel('Time')
ylabel('Radians/Second')
title('Z-Gyroscope vs True')
legend('Z-gyro IMU','Z-gyro True')
txt = ['RMSE = ' num2str(RMSE_z_gyr)];
text(15,4,txt)
%% NOTES
% - see report.pdf for a brief overview of the IMU sensor model
%   (particularly the Accelerometer and Rate Gyro); the sections on filters
%   are an interesting read, but not critical to this analysis
% - REMEMBER that the frames in use here are North-East-Down for the
%   inertial frame and Front-Right-Down for the UAV body frame
% - load the .mat file to get the above data vectors with:
%   load('IMUdata.mat');
% - use https://github.com/goromal/matlab_utilities to determine the alleged 
%   orientation (ignoring yaw) of the uav according to the imu accel vector:
%   addpath('matlab-utils/');
%   grav_vector = [x_acc_imu(i); y_acc_imu(i); z_acc_imu(i)];
%   q = Quatd_from_two_unit_vectors([0; 0; -1],grav_vector);
%   phi_acc = q.roll();
%   tht_acc = q.pitch();