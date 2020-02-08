clc
close all
clear all
addpath('functions')

if(verLessThan('matlab', '9'))
    fprintf('Please use a newer version of Matlab\n');
    return
end

raw_data = load('prova_HR.txt');


time = raw_data(:,1); %% tens of milliseconds
gyr = raw_data(:,2:4);
acc = raw_data(:,5:7);


raw_matrix = zeros(1,10,length(time));

raw_matrix(1,1:3,:) = acc';
raw_matrix(1,4:6,:) = gyr';
raw_matrix(1,10,:) = time;

dt_ = round(mean(diff(raw_matrix(1,10,:))));

q_initial = [0.9998    0.0009    0.0008   -0.0212];

%Calibration Samples
init=100;
stop=1000;

%% Number of IMUs
imu=1;
ang_magnitude =1;
%% calculate the calibration parameters
[acc_magnitude,ang_magnitude, acc_bias, gyro_bias, Sigma_acc, Sigma_acc_bias, Sigma_gyr, Sigma_gyr_bias, Sigma_mag, Sigma_mag_bias]=biasAndVariances_vicon(imu,init,stop,raw_matrix,ang_magnitude);

%% axis selection
% vector creation
acc_data=acc;%squeeze(raw_matrix(imu,[x y z],1:end))';
ang_data=gyr;%squeeze(raw_matrix(imu,3+[x y z],1:end))';
%mag_data=squeeze(raw_matrix(imu,6+[x y z],1:end))';

%% Use magnitude if the output is m/s^2

acc_data=acc_data./acc_magnitude;
ang_data=ang_data./ang_magnitude;


T=length(ang_data);

%% Magnetometer calibration

% bias correction
ang_data=ang_data-(gyro_bias'*ones(1,T))';



% state dimension
S=6;

% variance matrices
Q=[Sigma_gyr zeros(3)
    zeros(3) Sigma_gyr_bias];

R=[Sigma_acc zeros(3)
    zeros(3) Sigma_gyr];

% initial conditions
x0=zeros(1,S);
e1 = 0.001;
P0=[e1*eye(3) zeros(3)
    zeros(3) e1*eye(3)];

q0 = q_initial;% [1 0 0 0];
q=zeros(T,4);
q(1,:)=q0;

% stationary minimum time for correction
N=20*dt_;
%mag_ref=(q2rotm(q0)*mean(mag_data(10:100,:))')';
gacc=0.1;
gmag=6;


x=zeros(T,S);
P=zeros(S,S,T);
x(1,:)=x0;
P(:,:,1)=P0;
l=1;
c=1;


q_old = q_initial;
x=x0;
dt = dt_;
P= P0;
%% MEKF
for t=1:T-1
    [q_old, x, P] = computeQ(acc_data(t,:), ang_data(t,:),q_old, dt, x, P,Q,R,N,acc_magnitude, Sigma_acc, Sigma_gyr);
    q(t,:) = q_old;
end



%% estimaated q
figure('Name','Estimated q');
plot(q);
plotGt


