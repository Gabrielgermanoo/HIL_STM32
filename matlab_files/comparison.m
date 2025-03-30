clear s;

%% Configure the devkit (USE ONLY IF U HAVE THE BOARD)
serialPort = 'COM3';
baudRate = 115200;
s = serialport(serialPort, baudRate);

dt = 0.001;
time = 0:dt:7;
position_data = zeros(length(time), 1);

% Read the data from the serial port
for i = 1:length(time)
     data = readline(s);
     values = sscanf(data, '%f %f %f');

     if length(values) == 3
        x1_hil = values(1);
        x2_hil = values(2);
        x3_hil = values(3);

        % Store the data for plotting
        position_data(i, 1) = x1_hil;
        position_data(i, 2) = x2_hil;
        position_data(i, 3) = x3_hil;
     else
        disp('Erro na leitura de dados!');
        continue;
     end

end

clear s;
%% Saving the data in a csv

csvwrite('serial_data.csv', position_data);
position_data = csvread('serial_data.csv');
time = position_data(:, 1);
position_data = position_data(:, 2:end);

%% Configure the simulated MATLAB data

% Defining the parameters of the system
c1 = 80;
c2 = c1;
c3 = c1;
c4 = c1;

k1 = 696;
k2 = 595;
k3 = 461;
k4 = 454;

m1 = 0.31;
m2 = 0.712;
m3 = 0.383;


% Defining the parameters of the system
A = [0   0    0    1    0   0;
      0   0    0    0    1   0;
      0   0    0    0    0   1;
      (k2-k1)/m1   -k2/m1    0    (c2-c1)/m1    -c2/m1   0;
      k2/m2   (k3-k2)/m2    -k3/m2    c2/m2    c3-c2/m2   -c3/m2;
      0   k3/m3    (k4-k3)/m3    0    c3/m3   (c4-c3)/m3];

B = [0;
      0;
      0;
      1/m2;
      0;
      0];

C = [1 0 0 0 0 0];

D = 0;

% Sample time
T = 0.001;

% Identity matrix for the size of A
I = eye(size(A));

% Applying the Tustin method for discretization
A_d = (I - (T/2) * A) \ (I + (T/2) * A);
B_d = (I - (T/2) * A) \ (T * B);

% C and D remain the same
C_d = C;
D_d = D;

% Create the discrete-time state-space system
sys_d = ss(A_d, B_d, C_d, D_d, T);

% Time vector and step input
t = 0:T:7;
u = ones(size(t)) * 100;

% Simulate the response of the discretized system

x_zero = [0.1; 0; 0; 0; 0; 0;];

[y, t, x] = lsim(sys_d, u, t, x_zero);

%% Plotting

subplot(2, 1, 1);
stairs(t, x(:, 1), 'r', 'LineWidth', 1.5); hold on;
stairs(t, x(:, 2), 'b', 'LineWidth', 1.5); hold on;
stairs(t, x(:, 3), 'g', 'LineWidth', 1.5); 
title('Simulated Block Position');
xlabel('Time (s)');
ylabel('Position (m)');
legend('Block Position 1 (x1)', 'Block Position 2 (x2)', 'Block Position 3 (x3)');
grid on;
hold off;

subplot(2, 1, 2);
plot(time, position_data(:, 1), 'Color', [0.5412, 0.0078, 0.0078], 'LineWidth', 1.5); hold on; 
plot(time, position_data(:, 2), 'Color', [0.8745, 0.5490, 0.3647], 'LineWidth', 1.5); hold on; 
plot(time, position_data(:, 3), 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);          
title('Serial Data');
xlabel('Time (s)');
ylabel('Position (m)');
legend('Block Position 1 (HIL) (x1)', 'Block Position 2 (HIL) (x2)', 'Block Position 3 (HIL) (x3)');
grid on;
hold off;

figure;
plot(t, x(:, 3), 'g', 'LineWidth', 1.5); hold on; % Posição simulada do bloco 3
plot(time, position_data(:, 3), 'Color', [0.5412, 0.0078, 0.0078], 'LineWidth', 1.5); % Posição serial do bloco 3
title('Comparison of Block 3 Position');
xlabel('Time (s)');
ylabel('Position (m)');
legend('Simulated Block 3 (x3)', 'Serial Block 3 (HIL)');
grid on;
hold off;
