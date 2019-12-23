ang_magnitude=1; 

%Calibration Samples
init=100;
stop=1000;

%% Number of IMUs
imu=1;

%% calculate the calibration parameters
[acc_magnitude,ang_magnitude, acc_bias, gyro_bias, Sigma_acc, Sigma_acc_bias, Sigma_gyr, Sigma_gyr_bias, Sigma_mag, Sigma_mag_bias]=biasAndVariances_vicon(imu,init,stop,raw_matrix,ang_magnitude);

%% axis selection
x=1;
y=2;
z=3;

% vector creation
acc_data=squeeze(raw_matrix(imu,[x y z],1:end))';
ang_data=squeeze(raw_matrix(imu,3+[x y z],1:end))';
mag_data=squeeze(raw_matrix(imu,6+[x y z],1:end))';

%% Use magnitude if the output is m/s^2

acc_data=acc_data;%./acc_magnitude;
ang_data=ang_data./ang_magnitude;
%ang_data=deg2rad(ang_data);


% Sym time
min_dim=size(ang_data);
T=min_dim(1);

%% Magnetometer calibration

% bias correction
ang_data=ang_data-(gyro_bias'*ones(1,T))';



%% %%%%%%%%%%% Attitude from magnetometer and accelerometer %%%%%%%%%%%%%%%%%
%%% pitch angle (roll)
phi=atan2(acc_data(:,2),acc_data(:,3));
%%% roll angle (pitch)
theta=atan(-acc_data(:,1)./((acc_data(:,2).*sin(phi))+(acc_data(:,3).*cos(phi))));
%%% yaw angle = compass heading
xi=atan2((mag_data(:,3)).*sin(phi)-(mag_data(:,2)).*cos(phi),(mag_data(:,1)).*cos(theta)+(mag_data(:,2)).*sin(theta).*sin(phi)+(mag_data(:,3)).*sin(theta).*cos(phi));

for i=1:T
    RCompassAcc(:,:,i)=RZ(-xi(i))*RY(-theta(i))*RX(-phi(i));
    QCompassAcc(:,i)=M2q(RCompassAcc(:,:,i));
end
QCompassAcc=QCompassAcc([1 2 3 4],:);


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
acc_ref=[0 0 acc_magnitude];
mag_ref=(q2rotm(q0)*mean(mag_data(10:100,:))')';
gacc=0.1;
gmag=6;


x=zeros(T,S);
P=zeros(S,S,T);
x(1,:)=x0;
P(:,:,1)=P0;
l=1;
c=1;
pred_on=1;

interTimes=diff(squeeze(raw_matrix(imu,10,:)));%./10^6;
a_all = [0 0 0]; 
vb_all = [0 0 0];
h_all = [0 0 0 0 0 0];
x_all = [0 0 0 0 0 0];


%% MEKF
for t=1:T-1
    dt=interTimes(t)/10000; %time in seconds
    q_now=q(t,:);
    x_pred=[0 0 0 x(t,4:end)];
    omega=ang_data(t,:)-x_pred(4:6);
    F=eye(S)+dt.*[-SkewMat(omega) -eye(3)
        zeros(3) zeros(3)];
    G=dt*[-eye(3) zeros(3)
        zeros(3) eye(3)];
    P_now=squeeze(P(:,:,t));

    if pred_on==1
        P_pred=F*P_now*F'+G*Q*G';
    else
        P_pred=P_now;
    end
    
    x(t+1,:)=x_pred;
    P(:,:,t+1)=P_pred;
    om_t=[cos((norm(omega)*dt)/2) omega/norm(omega)*sin((norm(omega)*dt)/2)]';
    q(t+1,:)=qmul(q(t,:),om_t');
    
    % correction step (when stationary) ACC & GYRO
    if t>N && 0
        x_pred=x(t+1,:);
        q_now=q(t+1,:);
        P_pred =P(:,:,t+1);
        
        cA=var(acc_data(t-N:t,:))<=diag(Sigma_acc)';
        CA=abs((norm(mean(acc_data(t-N:t,:)))-norm(acc_ref))/norm(acc_ref))<=0.01;
        CAA=var(sqrt(sum(acc_data(t-N:t,:).^2,2))-ones(N+1,1))<=0.01;
        cG=var(ang_data(t-N:t,:))<=diag(Sigma_gyr)';
        CAAA=abs(norm(acc_data(t,:))-1)<=0.01;
        
        if all(CA) && all (cA) && all(cG) && all(CAA) %all(cA) && all(CA) && all(CAA) && all(cG) && CAAA
            
            c=c+1;
            pred_on=0;
            
            c=c+1;
            g=mean(acc_data(t-N:t,:))';
            an=mean(ang_data(t-N:t,:))';
            y=[g; an];
            vB=q2rotm(q_now)'*acc_ref';
            h=[vB; x_pred(4:6)'];
            Ha=SkewMat(vB);
            H=[Ha zeros(3)
                zeros(3) eye(3)];
            K=P_pred*H'*inv(H*P_pred*H'+R);
            K_all(:,:,t+1) = K;
            x(t+1,:)=x_pred'+K*(y-h);
            P(:,:,t+1)=P_pred-K*(H*P_pred*H'+R)*K';
            %x_all = [x_all;x];
            h_all = [h_all; h'];
            
            a=[x(t+1,1:3)];
            a_all = [a_all; a];
            vb_all = [vb_all; vB'];
            dq=[cos(norm(a)/2) a/norm(a)*sin(norm(a)/2)];
            q(t+1,:)=quatmultiply(q(t+1,:),dq);
        else
            pred_on=1;
        end
    end   
end

