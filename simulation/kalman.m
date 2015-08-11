clear;
hold on;

DT = 0.001;
DURATION = 1000;

A = [[1 0 0 DT 0 0 0 0];
     [0 1 0 0 DT 0 0 0]; 
     [0 0 1 0 0 DT 0 0]; 
     [0 0 0 1 0 0 0 0];
     [0 0 0 0 1 0 0 0]; 
     [0 0 0 0 0 1 0 0]; 
     [0 0 0 0 0 0 1 0]; 
     [0 0 0 0 0 0 0 1]];
B = [[0 0 0 0 0 0 0 0];
     [0 0 0 0 0 0 0 0];
     [0 0 0 0 0 0 0 0];
     [0 0 0 1 0 0 0 0];
     [0 0 0 0 1 0 0 0];
     [0 0 0 0 0 1 0 0];
     [0 0 0 0 0 0 0 0];
     [0 0 0 0 0 0 0 1]];
C = [[1 0 0 0 0 0 0 0];
     [0 1 0 0 0 0 0 0];
     [0 0 1 0 0 0 0 0];
     [0 0 0 1 0 0 0 0];
     [0 0 0 0 1 0 0 0];
     [0 0 0 0 0 1 0 0];
     [0 0 0 0 0 0 1 0];
     [0 0 0 0 0 0 0 1]];
 
 x = [1 1 0 0 0 0 0 0]';
 xhat = [1 1 0 0 0 0 0 0]';
 x_real = [1 1 0 0 0 0 0 0]';
 y = [1 1 0 0 0 0 0 0]';
  
 % Q is process covariance, R is measurement covariance
 Q =  [[.002 0 0 0 0 0 0 0];
      [0 .002 0 0 0 0 0 0];
      [0 0 0.005 0 0 0 0 0];
      [0 0 0 .005 0 0 0 0];
      [0 0 0 0 .005 0 0 0];
      [0 0 0 0 0 0.005 0 0];
      [0 0 0 0 0 0 .001 0];
      [0 0 0 0 0 0 0 0.05]];
 R = [[0.01 0 0 0 0 0 0 0];
      [0 0.01 0 0 0 0 0 0];
      [0 0 .005 0 0 0 0 0];
      [0 0 0 .01 0 0 0 0];
      [0 0 0 0 .01 0 0 0];
      [0 0 0 0 0 .05 0 0];
      [0 0 0 0 0 0 .001 0];
      [0 0 0 0 0 0 0 0.01]];    
  P = zeros(8);
  
kin_out = zeros(DURATION, 2);
kin_ideal = zeros(DURATION, 2);
kin_kalman = zeros(DURATION, 2);
error = zeros(DURATION,1);

% Kalman filter loop
for i = 1:DURATION
    
    % Generate the commands for the drone
    if (i < 0) 
        u = [0 0 0 1 1 0 0 0]';
    else
        u = [0 0 0 0 0 0 0 0]';
    end
    
    % Generate process and measurement noise
    w = zeros(8, 1);
    v = zeros(8, 1);
    for j = 1:8 
        w(j) = norminv(rand, 0, Q(j, j));
        v(j) = norminv(rand, 0, R(j, j));
    end
    
    % Simulate ideally
    x_real = A*x_real + B*u;
    kin_ideal(i, 1) = x_real(1);
    kin_ideal(i, 2) = x_real(2);   
       
    
    % Simulate linear systm
    x = A*x + B*u + w;
    y = C*x + v;
    kin_out(i, 1) = y(1);
    kin_out(i, 2) = y(2);
    
    % Kalman predict
    xhat = A*xhat + B*u;
    P = A*P*A' + Q;
    
    % Kalman gain
    K = P*C'*inv(C*P*C' + R);
    
    % Correction 
    xhat = xhat + K*(y-C*xhat);
    P = P - K*C*P;
  
    kin_kalman(i, 1) = xhat(1);
    kin_kalman(i, 2) = xhat(2);
    
    error(i,1) = sqrt((kin_ideal(i,1)-kin_kalman(i,1))^2 + (kin_ideal(i,2)-kin_kalman(i,2))^2);
      
end

% Plot data
figure(1);
plot(kin_ideal(:,1), kin_ideal(:,2), 'k');
plot(kin_out(:,1), kin_out(:,2), 'b');
%plot(kin_kalman(:,1), kin_kalman(:,2), 'g');
figure(2); clf;
plot(error(:,1));

