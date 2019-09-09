
clc
close all
clear all
addpath('functions')

if(verLessThan('matlab', '9'))
    fprintf('Please use a newer version of Matlab\n');
    return
end

raw_data = load('prova_HR.txt');

acc = raw_data(:,2:4);
gyr = raw_data(:,5:7);
time = raw_data(:,1); %% tens of milliseconds

use_magnetometer = 0;
raw_matrix = zeros(1,10,length(time));

raw_matrix(1,1:3,:) = acc';
raw_matrix(1,4:6,:) = gyr';
raw_matrix(1,10,:) = time;


dt_ = round(mean(diff(raw_matrix(1,10,:))));

%%lanci l'algoritmo
MEKF

angoli = quat2eul(q, 'XYZ');
plot(angoli*180/pi);
