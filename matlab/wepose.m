
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

raw_matrix(1,4:6,:) = acc';
raw_matrix(1,1:3,:) = gyr';
raw_matrix(1,10,:) = time;


dt_ = round(mean(diff(raw_matrix(1,10,:))));

groundTruth = raw_data(:,[18 15:17]);
for i=1:length(groundTruth)
    if (groundTruth(i,1) <0)
        groundTruth(i,:) = - groundTruth(i,:);
    end
end

a = find(groundTruth);
firstViconQindex = a(1);
startingPoint = 300*100/dt_;
q_initial = groundTruth(firstViconQindex,:);


MEKF

%% estimaated q
figure('Name','Estimated q');
plot(q);

 

figure('Name','Real q');
plot(groundTruth);