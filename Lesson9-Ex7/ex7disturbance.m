clear;
% vehicle mechanical parameter
% distance between two wheels
b = 0.3;

% motion disturbance param
Qmu = 0;
Qsigma = 0.1;
Q = [Qsigma.^2, 0; 0, Qsigma.^2];

% simulation parameter
% time step
dt = 0.01;

% 1st segment: rotation pi/2 [rad/s] for 2 sec
% vR = 0.2355 [m/s], vL = -0.2355 [m/s]
t1 = [0:dt:2-dt];
vR1 = 0.2355 * ones(size(t1));
vL1 = -0.2355 * ones(size(t1));
vR1d = random('Normal', Qmu, Qsigma, 1, length(t1));
vL1d = random('Normal', Qmu, Qsigma, 1, length(t1));

% 2nd segment: 
t2 = [2:dt:11.99];
vR2 = 1 * ones(size(t2));
vL2 = 1 * ones(size(t2));
vR2d = random('Normal', Qmu, Qsigma, 1, length(t2));
vL2d = random('Normal', Qmu, Qsigma, 1, length(t2));

% 3rd segment: translation 
t3 = [12:dt:13-dt];
vR3 = 0.2355 * ones(size(t3));
vL3 = -0.2355 * ones(size(t3));
vR3d = random('Normal', Qmu, Qsigma, 1, length(t3));
vL3d = random('Normal', Qmu, Qsigma, 1, length(t3));


% 4th segment: translation 
t4 = [13:dt:16-dt];
vR4 = 1 * ones(size(t4));
vL4 = 1 * ones(size(t4));
vR4d = random('Normal', Qmu, Qsigma, 1, length(t4));
vL4d = random('Normal', Qmu, Qsigma, 1, length(t4));

% simulation setup
t = [t1, t2, t3, t4];
vR = [vR1, vR2, vR3, vR4];
vL = [vL1, vL2, vL3, vL4];
vRd = [vR1d, vR2d, vR3d, vR4d];
vLd = [vL1d, vL2d, vL3d, vL4d]; 

% true position variables
xT = zeros(size(t));
yT = zeros(size(t));
qT = zeros(size(t));

% initial values
xT(1) = 0;
yT(2) = 0;
qT(1) = pi;

% predicted position variables (odemetry)
xO = zeros(size(t));
yO = zeros(size(t));
qO = zeros(size(t));

% initial values of predicred position
xO(1) = 0;
yO(2) = 0;
qO(1) = pi;

% odemetry cov matrix
PO = zeros(3, 3, length(t));
PO (:,:,1) = [0.01, 0, 0; 0, 0.01, 0; 0, 0, 0.01];
 
% loop
for k = 1:1599
    % true position calculation (real robot)
    xT(k+1) = xT(k) + dt*cos(qT(k))/2*(vL(k)+vLd(k)) + dt*cos(qT(k))/2*(vR(k)+vRd(k));
    yT(k+1) = yT(k) + dt*sin(qT(k))/2*(vL(k)+vLd(k)) + dt*sin(qT(k))/2*(vR(k)+vRd(k));
    qT(k+1) = qT(k) + dt*(vR(k)+vRd(k))/b - dt*(vL(k)+vLd(k))/b;
    
    % predicted position calculation (odometry)
    xO(k+1) = xO(k) + dt*cos(qO(k))/2*(vL(k)) + dt*cos(qO(k))/2*(vR(k));
    yO(k+1) = yO(k) + dt*sin(qO(k))/2*(vL(k)) + dt*sin(qO(k))/2*(vR(k));
    qO(k+1) = qO(k) + dt*(vR(k))/b - dt*(vL(k))/b;
    
    % odemetry cov
    PO(:, :, k+1) = PO(:, :, k) + dt*dt*J(qO(k), b)*Q*J(qO(k), b).';
end

% plot TRUE and ESTIMATED x-y position
figure(1);
plot(xT,yT,'o', xO, yO, 'o');
% xlim([-1 11]);
% ylim([-1 4]);
legend('Actual', 'Estimated');

% plot of orietntation
figure(2);
plot(t, qT, 'o', t, qO, 'o');
legend('Actual', 'Estimated');
