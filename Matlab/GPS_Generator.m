
points = [];
samples = [];
m = 10;

est = [[0; 0; 1.8; 1.8]];
err = [[1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1]];
predict_est = [];
predict_err = [];
K = [];

tests = [];
dt = 1;

R = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 50, 0; 0, 0, 0, 50]; % Measurement Covariance
Q = [0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0];  %state covariance

A = [1, 0, dt, 0; 0, 1, 0, dt; 0, 0, 1, 0; 0, 0, 0, 1];
H = [1, 0, 0, 0; 0, 1, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0];

for i = 0:.001:2
    y = i;
    x = (y-2)^3 + 3*(y-2)^2 + 2*(y-2);
    points = [points; m*[x,y]];
    
    if mod(m*i, .18) == 0
        samples = [samples; [normrnd(m*x,1), normrnd(m*y,1)]];
        %T = samples(end,:);

        %predict state and error covariance
        pe = A*est(:,end);
        predict_est = [predict_est, pe]; %#ok<*AGROW>
        pk = A*err(end-3:end,:)*transpose(A) + Q;
        predict_err = [predict_err; pk];

        %compute Kalman Gain
        K = pk*transpose(H)*(H*pk*transpose(H) + R)^-1;

        %compute error covariance
        p = pk - K*H*pk;
        err = [err; p];
        
        %compute estimate
        test = (transpose(samples(end,:))-est(1:2, end))./dt;
        Q(3,3) = p(3,3);
        Q(4,4) = p(4,4);
        
        
        tests = [tests, test];
        e = pe + K*([transpose(samples(end,:)); test] - H*pe);
        est = [est, e]; 
    end
    
end

est = transpose(est);
close all;
plot(points(:,1), points(:,2), 'm')
hold on;
plot(samples(:,1), samples(:,2), 'rx')
hold on
plot(est(:,1), est(:,2), 'b')
xlabel('x');
ylabel('y');
title('Sample GPS Datapoints');












