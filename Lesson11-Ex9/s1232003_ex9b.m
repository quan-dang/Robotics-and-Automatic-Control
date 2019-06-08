clear;

% vehicle mechanical parameter
% distance between two wheels
b = 0.3;

% motion disturbance param
Qmu = 0;
% variance of left and right wheel speed disturbance
Qsigma = 0.1;
Q = [Qsigma.^2, 0;...
    0, Qsigma.^2];

% observation noise param
% noise of position estimation [m/s]
Rmu = 0;
RsigmaXY = 0.2;
RsigmaQ = 0.1;
R = [RsigmaXY.^2, 0, 0;...
    0, RsigmaXY.^2, 0;...
    0, 0, RsigmaQ.^2];


% simulation params
% time step
dt = 0.1;
% simulate up to 20s
t = [0:dt:20];

% constants
% state dimension
cN = 3;

% two target positions
rD1 = [10; 0];
rD2 = [10; 3];
rD = [rD1, rD2];

% initial values of predicred position
k = 1;

% simulation setup
% variables
% true position variables
% rT(:,k) = [xT(k), yT(k), qT(k)]
% [m/s, m/s, rad]
rT = zeros(cN, length(t));
rT(:,1) = [0; 0; pi];

% predicted position variables (odemetry)
% rO(:,t) = [xT(t), yT(t), qT(t)]
% [m/s, m/s, rad]
rO = zeros(cN, length(t));

% initial prediction is right
rO(:,k) = rT(:,k);

% odemetry cov matrix
PO = zeros(cN, cN, length(t));
k = 1;

PO(:,:,1) = [0.01, 0, 0;...
    0, 0.01, 0;...
    0, 0, 0.01];
 
% observation value
rZ = zeros(cN, length(t));

% estimated pos variables after observation
rH = zeros(cN, length(t));
k = 1;
rH(:,k) = [0;0;pi];

% observation cov matrix
PH = zeros(cN, cN, length(t));
PH(:,:,1) = [0.01, 0, 0;...
    0, 0.01, 0;...
    0, 0, 0.01];

% variables for kalman filter
% error vector
E = zeros(cN, length(t));

% kalman gain matrix
K = zeros(cN, cN, length(t));

% % initial values of predicred position
% k = 1;

% control: transitional and rotational speed
% [m/s, rad/s]
v = zeros(1, length(t));
w = zeros(1, length(t));

% control: left and right wheel speed: vL, vR
vL = zeros(1, length(t));
vR = zeros(1, length(t));

% disturbance (slip)
vRd = random('Normal', Qmu, Qsigma, 1, length(t));
vLd = random('Normal', Qmu, Qsigma, 1, length(t));
u = zeros(2, length(t));
ud = zeros(2, length(t));

% task number
cL = 2; % rD1 and rD2, 2 target points
l = 1; % index

% maximum vehicle transitional and rotational speed
vc = 1;
wc = pi/2;

% margin distance to reach to target
dc = 0.1; % 0.01 is also ok

% main loop
for k = 1:199
    % find a vehicle control from desired and current plan
    % plan a transitional and rotational speed of a vehicle
    r = rD(:,l) - rH(1:2,k);
    v(k) = norm(r) / dt;
    
    % speed limit
    if v(k) > vc
       v(k) = vc; 
    end
    
    % rotational speed
    w(k) = (atan2(r(2), r(1)) - rH(3,k)) / dt;
    
    % speed limit
    if w(k) > wc
        w(k) = wc;
    elseif w(k) < -wc
        w(k) = -wc;
    end
    
    % convert to left and right wheel speed
    vL(k) = v(k) - b*w(k)/2;
    vR(k) = v(k) + b*w(k)/2;
    u(:,k) = [vL(k); vR(k)];
    
    % motion disturbance slip
    ud(:,k) = [vLd(k); vRd(k)];
    % forward kinematrics of vehicle
    rT(:,k+1) = rT(:,k) + dt * J(rT(3,k),b) * (u(:,k) + ud(:,k));
    
    % predicted position calculation (odometry)
    % rO(:,k+1) = rO(:,k) + dt * J(rO(3,k),b) * (u(:,k));
    rO(:,k+1) = rH(:,k) + dt *  J(rH(3,k),b) * (u(:,k));
    
    % odemetry cov
    PO(:,:,k+1) = PH(:,:,k) + dt.^2 * J(rO(3,k),b) * Q * J(rO(3,k),b).';
    
    % sensor value
    % add noise to true pos
    rZ(:,k+1) = rT(:,k+1) + ...
        [random('Normal', Rmu, RsigmaXY);...
        random('Normal', Rmu, RsigmaXY);...
        random('Normal', Rmu, RsigmaQ)];
    
    % kalman filter
    % error vector: observation - odemetry
    E(:,k+1) = rZ(:,k+1) - rO(:,k+1);
    
    % kalman gain: P/(R+P)
    K(:,:,k+1) = PO(:,:,k+1) * inv(R + PO(:,:,k+1));
    rH(:,k+1) = rO(:,k+1) + K(:,:,k+1) * E(:,k+1);
    PH(:,:,k+1) = (eye(3) - K(:,:,k+1))* PO(:,:,k+1);
    
    % if the vehicle is close to target, move to next target
    if norm(rD(:,l) - rT(1:2,k+1)) < dc
        l = l + 1;
    end
    if (l > cL)
        break;
    end    
end

% plot true and estimated x-y position
figure(1);
hold on;

plot(rT(1,:), rT(2,:), 'o', ...
    rO(1,:), rO(2,:), 'o',...
    rZ(1,:), rZ(2,:), 'o',...
    rH(1,:), rH(2,:), 'o');
plot(rD(1,1), rD(2,1), 'ro');
plot(rD(1,2), rD(2,2), 'ro');
xlim([-2, 12]);
ylim([-2, 5]);
axis equal
hold off;

% J function
function [m] = J(q, b)
    m = [cos(q)/2, cos(q)/2; sin(q)/2, sin(q)/2; -1/b, 1/b];
end

