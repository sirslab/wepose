function  [q_,x, P_corr] = computeQ(acc, gyr,q_old, interTimes, x,P_,Q,R,N,acc_magnitude,Sigma_acc,Sigma_gyr)
    persistent acc_data;
    persistent ang_data;
    persistent t_;
    if isempty(t_)
        t_ = 0;
        acc_data = zeros(N,3);
        ang_data = zeros(N,3);
    end
    t_ = t_+1;
    

    i = mod(t_,N)+1;
    acc_data(i,:)=acc;
    ang_data(i,:)=gyr;

    acc_ref=[0 0 acc_magnitude];

    
    S= 6;
    pred_on=1;
    dt=interTimes/10000; %time in seconds
    
    

    q_now=q_old;
    x_pred=[0 0 0 x(4:end)];
    omega=gyr-x_pred(4:6);
    F=eye(S)+dt.*[-SkewMat(omega) -eye(3)
        zeros(3) zeros(3)];
    G=dt*[-eye(3) zeros(3)
        zeros(3) eye(3)];
    P_now=P_;

    if pred_on==1
        P_pred=F*P_now*F'+G*Q*G';
    else
        P_pred=P_now;
    end
    
    x=x_pred;
    P_corr=P_pred;
    om_t=[cos((norm(omega)*dt)/2) omega/norm(omega)*sin((norm(omega)*dt)/2)]';
    q_=qmul(q_old,om_t');
    
    % correction step (when stationary) ACC & GYRO
    if t_>N 
        %x_pred=x(t+1,:);
        q_now=q_;
        P_pred =P_corr;
        
        cA=var(acc_data)<=diag(Sigma_acc)';
        CA=abs((norm(mean(acc_data))-norm(acc_ref))/norm(acc_ref))<=0.01;
        CAA=var(sqrt(sum(acc_data.^2,2))-ones(N,1))<=0.01;

        cG=var(ang_data)<=diag(Sigma_gyr)';
        CAAA=abs(norm(acc_data)-1)<=0.01;
        
        if all(CA) && all (cA) && all(cG) && all(CAA) %all(cA) && all(CA) && all(CAA) && all(cG) && CAAA
            
            c=c+1;
            pred_on=0;
            
            c=c+1;
            g=mean(acc_data)';
            an=mean(ang_data)';
            y=[g; an];
            vB=q2rotm(q_now)'*acc_ref';
            h=[vB; x_pred(4:6)'];
            Ha=SkewMat(vB);
            H=[Ha zeros(3)
                zeros(3) eye(3)];
            K=P_pred*H'*inv(H*P_pred*H'+R);
            K_all(:,:,t+1) = K;
            x(t+1,:)=x_pred'+K*(y-h);
            P_corr=P_pred-K*(H*P_pred*H'+R)*K';
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