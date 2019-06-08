clear;
% vehicle mechanical parameter
% distance between two wheels
b = 0.3;

% motion disturbance param
Qmu = 0;
Qsigma = 0.1;
Q = [Qsigma.^2, 0; 0, Qsigma.^2];

% observation noise param
% noise of position estimation [m/s]
Rmu = 0;
RsigmaXY = 0.2;
RsigmaQ = 0.1;
R = [RsigmaXY.^2, 0, 0;...
    0, RsigmaXY.^2, 0;...
    0, 0, RsigmaQ.^2];

% simulation parameter
% time step
dt = 0.1;

% constants
% state dimension is 3 (x, y, q)
cN = 3;

% control: planned wheel speed
% 1st segment: rotation pi/2 [rad/s] for 2 sec
% vR = 0.2355 [m/s], vL = -0.2355 [m/s]
t1 = [0:dt:2-dt];
vR1 = 0.2355 * ones(size(t1));
vL1 = -0.2355 * ones(size(t1));
vR1d = random('Normal', Qmu, Qsigma, 1, length(t1));
vL1d = random('Normal', Qmu, Qsigma, 1, length(t1));
u1 = [vL1; vR1];
w1 = [vL1d; vR1d];

% 2nd segment: 
t2 = [2:dt:11.99];
vR2 = 1 * ones(size(t2));
vL2 = 1 * ones(size(t2));
vR2d = random('Normal', Qmu, Qsigma, 1, length(t2));
vL2d = random('Normal', Qmu, Qsigma, 1, length(t2));
u2 = [vL2; vR2];
w2 = [vL2d; vR2d];

% 3rd segment: translation 
t3 = [12:dt:13-dt];
vR3 = 0.2355 * ones(size(t3));
vL3 = -0.2355 * ones(size(t3));
vR3d = random('Normal', Qmu, Qsigma, 1, length(t3));
vL3d = random('Normal', Qmu, Qsigma, 1, length(t3));
u3 = [vL3; vR3];
w3 = [vL3d; vR3d];


% 4th segment: translation 
t4 = [13:dt:16-dt];
vR4 = 1 * ones(size(t4));
vL4 = 1 * ones(size(t4));
vR4d = random('Normal', Qmu, Qsigma, 1, length(t4));
vL4d = random('Normal', Qmu, Qsigma, 1, length(t4));
u4 = [vL4; vR4];
w4 = [vL4d; vR4d];

% simulation setup
t = [t1, t2, t3, t4];
vR = [vR1, vR2, vR3, vR4];
vL = [vL1, vL2, vL3, vL4];
vRd = [vR1d, vR2d, vR3d, vR4d];
vLd = [vL1d, vL2d, vL3d, vL4d]; 
u = [u1, u2, u3, u4];
w = [w1, w2, w3, w4];


% variables
% true position variables
% tY(:,k) = [xT(k); yT(k); qT(k)]
% [m/s; m/s; rad]
rT = zeros(cN, length(t));

% initial values
% tY(:,1) = [xT(1); yT(1); qT(1)]
% [m/s; m/s; rad]
k = 1;
rT(:,k) = [0; 0; pi];

% predicted position variables (odemetry)
rO = zeros(cN, length(t));

% initial values of predicred position
k = 1;

% initial prediction is right
rO(:,k) = rT(:,k);

% odemetry cov matrix
PO = zeros(cN, cN, length(t));
k = 1;

PO(:,:,1) = [0.01, 0, 0;...
    0, 0.01, 0;...
    0, 0, 0.01];
 
% loop
for k = 1:159
    % true position calculation (real robot)
    rT(:,k+1) = rT(:,k) + dt * J(rT(3,k),b) * (u(:,k) + w(:,k));
    
    % predicted position calculation (odometry)
    rO(:,k+1) = rO(:,k) + dt * J(rO(3,k),b) * (u(:,k));
    
    % odemetry cov
    PO(:,:,k+1) = PO(:,:,k) + dt.^2 * J(rO(3,k), b) * Q * J(rO(3,k), b).';
    
    % sensor value
    % add noise to true pos
    rZ(:,k+1) = rT(:,k+1) + ...
        [random('Normal', Rmu, RsigmaXY);...
        random('Normal', Rmu, RsigmaXY);...
        random('Normal', Rmu, RsigmaQ)];
end

% plot TRUE and ESTIMATED x-y position
figure(1);
plot(rT(1,:),rT(2,:),'o', rO(1,:), rO(2,:), 'o', rZ(1,:), rZ(2,:), 'o');
% xlim([-1 11]);
% ylim([-1 4]);
legend('Actual', 'Estimated', 'Sensor');

% plot of orietntation
figure(2);
plot(t, rT(3,:), 'o', t, rO(3,:), 'o', t, rZ(3,:), 'o');
legend('Actual', 'Estimated', 'Sensor');
