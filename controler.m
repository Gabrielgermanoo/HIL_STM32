% Configuração da comunicação serial com serialport
serialPort = 'COM9'; % Modifique para o nome correto da porta serial
baudRate = 115200;
s = serialport(serialPort, baudRate);

% Defina parâmetros do PID
Kp_position = 1.0;
Ki_position = 0.5;
Kd_position = 0.1;

Kp_velocity = 1.0;
Ki_velocity = 0.5;
Kd_velocity = 0.1;

% Inicialize variáveis para o controlador
error_position_sum = 0;
error_velocity_sum = 0;
previous_error_position = 0;
previous_error_velocity = 0;

% Parâmetros de simulação
dt = 0.001; % Tempo de amostragem
target_position = 1; % Defina a posição desejada (setpoint)
target_velocity = 0; % Defina a velocidade desejada (setpoint)

% Inicialize vetores para armazenamento de dados
time = 0:dt:5;
position_data = zeros(length(time), 1);
velocity_data = zeros(length(time), 1);
control_position_data = zeros(length(time), 1);
control_velocity_data = zeros(length(time), 1);

% Loop de controle
for i = 1:length(time)

    % Leia a saída da UART
    data = readline(s); % Leitura de uma linha de dados
    
    % Converte os dados lidos para números
    values = sscanf(data, '%f %f %f %f');
    disp(values(1));
    
    % Atribua as leituras de posição e velocidade aos estados
    if length(values) == 4
        x1 = values(1); % Posição do bloco 1
        x2 = values(2); % Posição do bloco 2
        x3 = values(3); % Velocidade do bloco 1
        x4 = values(4); % Velocidade do bloco 2
    else
        disp('Erro na leitura de dados!');
        continue;
    end

    % Erro de posição e velocidade
    error_position = target_position - x1; % Erro de posição (exemplo para bloco 1)
    error_velocity = target_velocity - x3; % Erro de velocidade (exemplo para bloco 1)

    % Controle PID para a posição
    error_position_sum = error_position_sum + error_position * dt;
    derivative_position = (error_position - previous_error_position) / dt;
    control_position = Kp_position * error_position + Ki_position * error_position_sum + Kd_position * derivative_position;
    previous_error_position = error_position;

    % Controle PID para a velocidade
    error_velocity_sum = error_velocity_sum + error_velocity * dt;
    derivative_velocity = (error_velocity - previous_error_velocity) / dt;
    control_velocity = Kp_velocity * error_velocity + Ki_velocity * error_velocity_sum + Kd_velocity * derivative_velocity;
    previous_error_velocity = error_velocity;

    % Envie o sinal de controle para a UART
    writeline(s, sprintf('%f %f\n', control_position, control_velocity));

    % Armazene os dados para análise
    position_data(i) = x1; % posição do bloco 1
    velocity_data(i) = x3; % velocidade do bloco 1
    control_position_data(i) = control_position;
    control_velocity_data(i) = control_velocity;
end

% Limpe o objeto serial para fechar a conexão
clear s;

% Plotar os dados
figure;
subplot(2,1,1);
plot(time, position_data, 'b', 'DisplayName', 'Posição');
hold on;
plot(time, target_position * ones(size(time)), 'r--', 'DisplayName', 'Setpoint de Posição');
xlabel('Tempo (s)');
ylabel('Posição');
title('Posição do Bloco');
legend;
grid on;

subplot(2,1,2);
plot(time, velocity_data, 'b', 'DisplayName', 'Velocidade');
hold on;
plot(time, target_velocity * ones(size(time)), 'r--', 'DisplayName', 'Setpoint de Velocidade');
xlabel('Tempo (s)');
ylabel('Velocidade');
title('Velocidade do Bloco');
legend;
grid on;

% Plot do sinal de controle
figure;
subplot(2,1,1);
plot(time, control_position_data, 'g', 'DisplayName', 'Controle de Posição');
xlabel('Tempo (s)');
ylabel('Sinal de Controle (Posição)');
title('Sinal de Controle para Posição');
legend;
grid on;

subplot(2,1,2);
plot(time, control_velocity_data, 'g', 'DisplayName', 'Controle de Velocidade');
xlabel('Tempo (s)');
ylabel('Sinal de Controle (Velocidade)');
title('Sinal de Controle para Velocidade');
legend;
grid on;


%%
% Configuração da comunicação serial com serialport
serialPort = 'COM9'; % Modifique para o nome correto da porta serial
baudRate = 115200;
s = serialport(serialPort, baudRate);

% Defina parâmetros do PID
Kp_position = 0.4;
Ki_position = 0.062;
Kd_position = 0.03;

Kp_velocity = 1.0;
Ki_velocity = 0.5;
Kd_velocity = 0.1;

% Inicialize variáveis para o controlador
error_position_sum = 0;
error_velocity_sum = 0;
previous_error_position = 0;
previous_error_velocity = 0;

% Parâmetros de simulação
dt = 0.001; % Tempo de amostragem
target_position = 1; % Defina a posição desejada (setpoint)
target_velocity = 0; % Defina a velocidade desejada (setpoint)

% Inicialize vetores para armazenamento de dados
time = 0:dt:10;
position_block1_data = zeros(length(time), 1);
position_block2_data = zeros(length(time), 1);
velocity_block1_data = zeros(length(time), 1);
velocity_block2_data = zeros(length(time), 1);
control_position_data = zeros(length(time), 1);
control_velocity_data = zeros(length(time), 1);

% Loop de controle
for i = 1:length(time)

    % Leia a saída da UART
    data = readline(s); % Leitura de uma linha de dados
    
    % Converte os dados lidos para números
    values = sscanf(data, '%f %f %f %f');
    
    % Atribua as leituras de posição e velocidade aos estados
    if length(values) == 4
        x1 = values(1); % Posição do bloco 1
        x2 = values(2); % Posição do bloco 2
        x3 = values(3); % Velocidade do bloco 1
        x4 = values(4); % Velocidade do bloco 2
    else
        disp('Erro na leitura de dados!');
        continue;
    end

    % Erro de posição e velocidade
    error_position = target_position - x1; % Erro de posição (exemplo para bloco 1)
    error_velocity = target_velocity - x3; % Erro de velocidade (exemplo para bloco 1)

    % Controle PID para a posição
    error_position_sum = error_position_sum + error_position * dt;
    derivative_position = (error_position - previous_error_position) / dt;
    control_position = Kp_position * error_position + Ki_position * error_position_sum + Kd_position * derivative_position;
    previous_error_position = error_position;

    % Controle PID para a velocidade
    error_velocity_sum = error_velocity_sum + error_velocity * dt;
    derivative_velocity = (error_velocity - previous_error_velocity) / dt;
    control_velocity = Kp_velocity * error_velocity + Ki_velocity * error_velocity_sum + Kd_velocity * derivative_velocity;
    previous_error_velocity = error_velocity;

    % Envie o sinal de controle para a UART
    writeline(s, sprintf('%f %f\n', control_position, control_velocity));

    % Armazene os dados para análise
    position_block1_data(i) = x1; % posição do bloco 1
    position_block2_data(i) = x2; % posição do bloco 2
    velocity_block1_data(i) = x3; % velocidade do bloco 1
    velocity_block2_data(i) = x4; % velocidade do bloco 2
    control_position_data(i) = control_position;
    control_velocity_data(i) = control_velocity;
end

% Limpe o objeto serial para fechar a conexão
clear s;

% Plotar os dados
figure;

% Gráfico de posição dos blocos
subplot(2,1,1);
plot(time, position_block1_data, 'b', 'DisplayName', 'Posição Bloco 1');
hold on;
plot(time, position_block2_data, 'g', 'DisplayName', 'Posição Bloco 2');
plot(time, target_position * ones(size(time)), 'r--', 'DisplayName', 'Setpoint de Posição');
xlabel('Tempo (s)');
ylabel('Posição');
title('Posição dos Blocos');
legend;
grid on;

% Gráfico de velocidade dos blocos
subplot(2,1,2);
plot(time, velocity_block1_data, 'b', 'DisplayName', 'Velocidade Bloco 1');
hold on;
plot(time, velocity_block2_data, 'g', 'DisplayName', 'Velocidade Bloco 2');
plot(time, target_velocity * ones(size(time)), 'r--', 'DisplayName', 'Setpoint de Velocidade');
xlabel('Tempo (s)');
ylabel('Velocidade');
title('Velocidade dos Blocos');
legend;
grid on;

% Plot do sinal de controle
figure;
subplot(2,1,1);
plot(time, control_position_data, 'c', 'DisplayName', 'Controle de Posição');
xlabel('Tempo (s)');
ylabel('Sinal de Controle (Posição)');
title('Sinal de Controle para Posição');
legend;
grid on;

subplot(2,1,2);
plot(time, control_velocity_data, 'm', 'DisplayName', 'Controle de Velocidade');
xlabel('Tempo (s)');
ylabel('Sinal de Controle (Velocidade)');
title('Sinal de Controle para Velocidade');
legend;
grid on;
