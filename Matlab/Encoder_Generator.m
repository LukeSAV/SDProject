
thetas = [];
points = [];
samples = [];

est = [[0; 0; 0; 1.8; 0; 0]];
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
Rs = [];
centers = [];
as = [];
states = [];
preds = [0; 0; 0; 0; 0];

measVels = [];
measAccels = [];

m = 10;
phis = [0];
ys = [0];
dys = [];
xs = [0];
dxs = [];
dt = 1;
dt2 = dt*dt;

R = [1000, 0, 0, 0, 0, 0; 
     0, 1000, 0, 0, 0, 0; 
     0, 0, 1000, 0, 0, 0; 
     0, 0, 0, 1000, 0, 0;
     0, 0, 0, 0, 1000, 0;
     0, 0, 0, 0, 0, 1000;]; % Measurement Covariance
 
Q = [0.0000, 0, 0, 0, 0, 0; 
     0, 0.0000, 0, 0, 0, 0; 
     0, 0, 0.0000, 0, 0, 0; 
     0, 0, 0, 0.0000, 0, 0;
     0, 0, 0, 0, 0.0000, 0;
     0, 0, 0, 0, 0, 0.0000;];  %state covariance

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

for i = 0:1:100
    newTheta = [pi()/6, pi()/6 - i*pi()/10000];
    thetas = [thetas; newTheta];
    phi = phis(end) + (newTheta(1)-newTheta(2))*(25.5/(27*2));
    phis = [phis; phi];
    
    %radius of circular path
    if (newTheta(1)-newTheta(2)) == 0
        R = 1000000;
    else
        R = 13.5*(newTheta(1)+newTheta(2))/(newTheta(1)-newTheta(2));
    end
    Rs = [Rs; R];
    
    %x and y coordinates of the center of the circular path 
    rx = xs(end) + R*sin(phi);
    ry = ys(end) - R*cos(phi);
    center = [rx, ry];
    centers = [centers; center];
    
    
    %angle of circular path traversed in one time step
    a = 2*pi()*((newTheta(1)/(2*pi())) * (25.5*pi()))/(2*pi()*(R+13.5));
    as = [as; a];
    x = rx+(xs(end)-rx)*cos(a)-(ys(end)-ry)*sin(a);
    y = ry+(xs(end)-rx)*sin(a)+(ys(end)-ry)*cos(a);
    
    ys = [ys; y];
    xs = [xs; x];
    newPoint = [x,y];
    
    state = [phi; a; R; x; y];
    states = [states, state];
    T1 = 1;
    T2 = 0;
    T3 = state(3)*cos(state(1)) - state(3)*cos(state(2))*cos(state(1)) + state(3)*sin(state(2))*sin(state(1));
    T4 = state(3)*sin(state(1))*sin(state(2)) - state(3)*cos(state(1))*cos(state(2));
    T5 = sin(state(1)) - sin(state(1))*cos(state(2)) - cos(state(1))*sin(state(2));
    T6 = state(3)*sin(state(1)) - state(3)*sin(state(2))*cos(state(1)) - state(3)*cos(state(2))*sin(state(1));
    T7 = -1*state(3)*sin(state(1))*cos(state(2)) - state(3)*cos(state(1))*sin(state(2));
    T8 = -1*cos(state(1)) - sin(state(1))*sin(state(2)) + cos(state(1))*cos(state(2));
    J = [1, T1, T2,  0,  0;
         0,  1,  0,  0,  0;
         0,  0,  1,  0,  0;
         T3, T4, T5, 1,  0;
         T6, T7, T8, 0,  1;];
    pred = J*state;
    preds = [preds, pred];
    points = [points; newPoint]; %newVel, newAccel];
    
%     if mod(i, 1) == 0
%         samples = [samples; [normrnd(x,100), normrnd(y,100)]];
%         %T = samples(end,:);
% 
%         %predict state and error covariance
%         pe = A*est(:,end);
%         predict_est = [predict_est, pe]; %#ok<*AGROW>
%         
%         Qfudge = [0, 0, 0, 0, 0, 0;
%                   0, 0, 0, 0, 0, 0;
%                   0, 0, 0, 0, 0, 0;
%                   0, 0, 0, 0, 0, 0;
%                   0, 0, 0, 0, 0, 0;
%                   0, 0, 0, 0, 0, 0;];
%          
%         Q = Q + Qfudge;
%          
%         pk = A*err(end-5:end,:)*transpose(A) + Q;
%         predict_err = [predict_err; pk];
% 
%         %compute Kalman Gain
%         K = pk*transpose(H)*(H*pk*transpose(H) + R)^-1;
%         gains = [gains; K];
% 
%         %compute error covariance
%         p = pk - K*H*pk;
%         err = [err; p];
%         
%         %compute estimate
%         measVel = (transpose(samples(end,:))-est(1:2, end))./dt;
%         measVels = [measVels, measVel];
%         
%         measAccel = (measVel - est(3:4, end))./dt;
%         measAccels = [measAccels, measAccel];
%         
%         
%         e = pe + K*([transpose(samples(end,:)); measVel; measAccel] - H*pe);
%         est = [est, e]; 
%     end
    
end

est = transpose(est);
close all;
plot(points(:,1), points(:,2), 'kx')
% hold on;
% plot(samples(:,1), samples(:,2), 'rx')
% hold on
% plot(est(:,1), est(:,2), 'b')
xlabel('x');
ylabel('y');
title('Sample Encoder Datapoints');












