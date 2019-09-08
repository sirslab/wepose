

tracking = 0;
offsetTracking =1;% 110; %%150

use_magnetometer = 0;



raw_matrix = zeros(1,10,length(prova.t));

raw_matrix(1,1:3,:) = prova.acc;
raw_matrix(1,4:6,:) = prova.gyr;
raw_matrix(1,10,:) = prova.t


dt_ = round(mean(diff(raw_matrix(1,10,:))));

%%lanci l'algoritmo
MEKF

angoli = quat2eul(q, 'XYZ');
plot(angoli*180/pi);
