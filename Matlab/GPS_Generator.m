
points = [];
samples = [];
m = 10;

est = [[0; 0], [0; 0]];
err = [[0, 0; 0, 0]; [1, 0.1; 0.1, 1]];
predict_est = [];
predict_err = [];
K = [];

F = est(:,end);
%G = err(end-1:end,:);

R = [1, 0.01; 0.01, 1]; % Measurement Covariance
Q = [1, 0; 0, 1];  %state covariance

A = [1, 0; 0, 1];
H = [1, 0; 0, 1];

for i = 0:.001:2
    y = i;
    x = (y-2)^3 + 3*(y-2)^2 + 2*(y-2);
    points = [points; m*[x,y]];
    
    if mod(m*i, .18) == 0
        samples = [samples; [normrnd(m*x,1), normrnd(m*y,1)]];
        %T = samples(end,:);

        %predict stae and error covariance
        pe = A*est(:,end);
        predict_est = [predict_est, pe]; %#ok<*AGROW>
        pk = A*err(end-1:end,:)*transpose(A) + Q
        predict_err = [predict_err; pk];

        %compute Kalman Gain
        K = pk*transpose(H)*(H*pk*transpose(H) + R)^-1;

        %compute estimate
        e = pe + K*(transpose(samples(end,:)) - H*pe);
        est = [est, e];

        %compute error covariance
        p = pk - K*H*pk;
        err = [err; p];
    end
    
end

est = transpose(est)

plot(points(:,1), points(:,2), 'b')
hold on;
plot(samples(:,1), samples(:,2), 'rx')
hold on
plot(est(:,1), est(:,2), 'b')
xlabel('x');
ylabel('y');
title('Sample GPS Datapoints');












