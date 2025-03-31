clc; clear; close all;

serialPort = "COM3";
baudRate = 115200;
s = serialport(serialPort, baudRate);

h = 0.001;
beta1 = 200;
beta2 = 2000;
beta3 = 20000;
kp = 100;
kd = 20;
ko = 300;

z = zeros(3, 1); 
u_ant = 0;
r = 0.05;
y = 0;

disp("ADRC Controler init...");

time = 0:h:10;

x3_data = zeros(length(time), 1);
u_data = zeros(length(time), 1);
r_data = ones(length(time), 1) * r;

for i = 1:length(time)
    try
        data = readline(s);
        values = sscanf(data, "%f %f %f");
        
        if length(values) == 3
            x1 = values(1);
            x2 = values(2);
            x3 = values(3);
        else
            continue;
        end

        y = x3;
        
        e = y - z(1);
        z(1) = z(1) + h * (z(2) - beta1 * e);
        z(2) = z(2) + h * (z(3) - beta2 * e);
        z(3) = z(3) + h * (-beta3 * e);

        u = -kp * (z(1) - r) - kd * z(2) - z(3) / ko;
        
        u = max(min(u, 10), -10);
        
        writeline(s, num2str(u));

        x3_data(i) = x3;
        u_data(i) = u;

        fprintf("r: %.2f, x3: %.2f, u: %.2f\n", r, x3, u);
        
        u_ant = u;
        
        pause(h);
    
    catch
        warning("Erro na leitura da serial. Tentando novamente...");
        continue; % Continua tentando
    end
end

disp("Execução do controlador ADRC finalizada.");

clear s;

% Getting data and passing to csv
data = [time', x3_data, u_data, r_data];
csvwrite('ADRC_data.csv', data);

subplot(2, 1, 1);
plot(time, x3_data, 'b', 'LineWidth', 1.5); hold on;
plot(time, r_data, 'r--', 'LineWidth', 1.5);
title('Block 3 Position');
xlabel('Time (s)');
ylabel('Position (m)');
legend('x3 (Block 3)', 'Reference (r)');
grid on;

subplot(2, 1, 2);
plot(time, u_data, 'g', 'LineWidth', 1.5);
title('Applied Control (u)');
xlabel('Time (s)');
ylabel('Control (u)');
grid on;
