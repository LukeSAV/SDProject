
points = [];
samples = [];

est = [[0; 0; 2*pi(); 1; 0; 0]];
err = [[0.000, 0, 0, 0, 0, 0; 
        0, 0.000, 0, 0, 0, 0; 
        0, 0, 0.000, 0, 0, 0; 
        0, 0, 0, 0.000, 0, 0;
        0, 0, 0, 0, 0.000, 0;
        0, 0, 0, 0, 0, 0.000;]];
    
predict_est = [];
predict_err = [];
K = [];
gains = [];

measVels = [];
measAccels = [];

m = 10;
dt = 1;
dt2 = dt*dt;

R = [10, 0, 0, 0, 0, 0; 
     0, 10, 0, 0, 0, 0; 
     0, 0, 100, 0, 0, 0; 
     0, 0, 0, 100, 0, 0;
     0, 0, 0, 0, 1000, 0;
     0, 0, 0, 0, 0, 1000;]; % Measurement Covariance
 
Q = [0.00001, 0, 0, 0, 0, 0; 
     0, 0.00001, 0, 0, 0, 0; 
     0, 0, 0.00001, 0, 0, 0; 
     0, 0, 0, 0.00001, 0, 0;
     0, 0, 0, 0, 0.00001, 0;
     0, 0, 0, 0, 0, 0.00001;];  %state covariance

A = [1, 0, dt, 0, dt2, 0; 
     0, 1, 0, dt, 0, dt2; 
     0, 0, 1, 0, dt, 0; 
     0, 0, 0, 1, 0, dt;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;];
 
H = [1, 0, 0, 0, 0, 0; 
     0, 1, 0, 0, 0, 0; 
     0, 0, 1, 0, 0, 0; 
     0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;];

for i = 0:1:2000
    y = i;
    dy = 1;
    d2y = 0;
    
    x = 2000*sin(((2*pi())/2000)*i);
    dx = -2*pi()*cos((2*pi()/2000)*i);
    d2x = ((4*pi()*pi())/2000)*sin((2*pi()/2000)*i);
    
    newPoint = [x,y];
    newVel = [dx, dy];
    newAccel = [d2x, d2y];
    points = [points; newPoint, newVel, newAccel];
    
    if mod(i, 10) == 0
        samples = [samples; [normrnd(x,100), normrnd(y,100)]];
        %T = samples(end,:);

        %predict state and error covariance
        pe = A*est(:,end);
        predict_est = [predict_est, pe]; %#ok<*AGROW>
        
        Qfudge = [0, 0, 0, 0, 0, 0;
                  0, 0, 0, 0, 0, 0;
                  0, 0, 0, 0, 0, 0;
                  0, 0, 0, 0, 0, 0;
                  0, 0, 0, 0, 0, 0;
                  0, 0, 0, 0, 0, 0;];
         
        Q = Q + Qfudge;
         
        pk = A*err(end-5:end,:)*transpose(A) + Q;
        predict_err = [predict_err; pk];

        %compute Kalman Gain
        K = pk*transpose(H)*(H*pk*transpose(H) + R)^-1;
        gains = [gains; K];

        %compute error covariance
        p = pk - K*H*pk;
        err = [err; p];
        
        %compute estimate
        measVel = (transpose(samples(end,:))-est(1:2, end))./dt;
        measVels = [measVels, measVel];
        
        measAccel = (measVel - est(3:4, end))./dt;
        measAccels = [measAccels, measAccel];
        
        
        e = pe + K*([transpose(samples(end,:)); measVel; measAccel] - H*pe);
        est = [est, e]; 
    end
    
end

est = transpose(est);
close all;
plot(points(:,1), points(:,2), 'k')
hold on;
plot(samples(:,1), samples(:,2), 'rx')
hold on
plot(est(:,1), est(:,2), 'b')
xlabel('x');
ylabel('y');
title('Sample GPS Datapoints');












