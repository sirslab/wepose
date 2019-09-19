function [acc_magnitude,ang_magnitude, acc_bias, gyro_bias, Sigma_acc, Sigma_acc_bias, Sigma_gyr, Sigma_gyr_bias, Sigma_mag, Sigma_mag_bias]=biasAndVariances_vicon(imu,init,T,raw_matrix,ang_magnitude)
    % axis selection
    x=1;
    y=2;
    z=3;

    % vector creation
    acc_data=squeeze(raw_matrix(imu,[x y z],1:T))';
    ang_data=squeeze(raw_matrix(imu,3+[x y z],1:T))';
    mag_data=squeeze(raw_matrix(imu,6+[x y z],1:T))';

    % acc_data(:,3)=-acc_data(:,3);
    % ang_data(:,3)=-ang_data(:,3);
    % mag_data(:,3)=-mag_data(:,3);

    acc_magnitude=norm(mean(acc_data));

    acc_data=acc_data;%./acc_magnitude;
    ang_data=ang_data./ang_magnitude;
%     ang_data=degtorad(ang_data);
%     figure
%     plot(acc_data)
%     figure
%     plot(ang_data)
%     figure
%     plot(mag_data)

    acc_bias=mean(acc_data-([0 0 acc_magnitude]'*ones(1,T))');
    % figure
    % plot(acc_data-(acc_bias'*ones(1,T))')
    %acc_data=acc_data-(acc_bias'*ones(1,TotalTime))';

    Sigma_acc=diag(var(acc_data(1:T,:)));
    Sigma_acc_bias=diag(var(diff(acc_data(1:T,:))));

    Sigma_mag=diag(var(mag_data(1:T,:)));
    Sigma_mag_bias=diag(var(diff(mag_data(1:T,:))));


    gyro_bias=mean(ang_data);
    % figure
    % plot(ang_data-(gyro_bias'*ones(1,T))')

    Sigma_gyr=diag(var(ang_data(1:T,:)));
    Sigma_gyr_bias=diag(var(diff(ang_data(1:T,:))));

  
end