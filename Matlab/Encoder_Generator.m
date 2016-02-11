
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
as = [0];
states = [0; 0; 0; 0; 0];
preds = [];

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

for i = 0:1:16
    %predict next path point
    last_state = states(:,end);
    R = last_state(3)
    a = last_state(2)
    phi = last_state(1)/1.2
    T3 = R*cos(phi)-R*cos(a)*cos(phi) + R*sin(a)*sin(phi);
    T4 = R*sin(phi)*sin(a) - R*cos(phi)*cos(a);
    T5 = sin(phi) - sin(phi)*cos(a) - cos(phi)*sin(a);
    T6 = R*sin(phi) - R*sin(a)*cos(phi) - R*cos(a)*sin(phi);
    T7 = -R*sin(phi)*cos(a) - R*cos(phi)*sin(a);
    T8 = -cos(phi) - sin(phi)*sin(a) + cos(phi)*cos(a);
    J = [1,  1,  0,  0,  0;
         0,  1,  0,  0,  0;
         0,  0,  1,  0,  0;
         T3, T4,  T5, 1,  0;
         T6, T7,  T8, 0,  1;];
    pred = J*last_state;
    preds = [preds, pred];
    
    %calculate next position
    newTheta = [(54/25.5)*pi()/8, 0];
    
    thetas = [thetas; newTheta];
    phi = phis(end) + (newTheta(1)-newTheta(2))*(25.5/(27*2));
    phi = mod(phi, 2*pi());
    phis = [phis; phi];
    
    %radius of circular path
    if (newTheta(1)-newTheta(2)) == 0
        R = 1000000;
    else
        R = 13.5*(newTheta(1)+newTheta(2))/(newTheta(1)-newTheta(2));
    end
    Rs = [Rs; R];
    
    %x and y coordinates of the center of the circular path 
    rx = xs(end) + R*sin(phis(end-1));
    ry = ys(end) - R*cos(phis(end-1));
    center = [rx, ry];
    centers = [centers; center];
    
    %angle of circular path traversed in one time step
    a = 2*pi()*((newTheta(1)/(2*pi())) * (25.5*pi()))/(2*pi()*(R+13.5));
    as = [as; a];
    %x = rx+(xs(end)-rx)*cos(a)-(ys(end)-ry)*sin(a);
    %y = ry+(xs(end)-rx)*sin(a)+(ys(end)-ry)*cos(a);
    x = xs(end)+R*sin(phis(end-1))-R*sin(phis(end-1))*cos(a) - R*cos(phis(end-1))*sin(a);
    y = ys(end)-R*cos(phis(end-1))-R*sin(phis(end-1))*sin(a) + R*cos(phis(end-1))*cos(a);
    
    ys = [ys; y];
    xs = [xs; x];
    newPoint = [x,y];
    
    state = [phi; a; R; x; y];
    states = [states, state];
    points = [points; newPoint]; %newVel, newAccel];
    
    if mod(i, 1) == 0
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
        jjk
        measAccel = (measVel - est(3:4, end))./dt;
        measAccels = [measAccels, measAccel];
        
        
        e = pe + K*([transpose(samples(end,:)); measVel; measAccel] - H*pe);
        est = [est, e]; 
    end
    
end

est = transpose(est);
close all;
plot(states(4,:), states(5,:), 'kx')
hold on
plot(preds(4,:), preds(5,:), 'bx')
% hold on;
% plot(samples(:,1), samples(:,2), 'rx')
% hold on
% plot(est(:,1), est(:,2), 'b')
xlim([-40;40])
ylim([-40;40])
xlabel('x');
ylabel('y');
title('Sample Encoder Datapoints');












