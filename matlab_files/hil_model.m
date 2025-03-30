
% Definir as matrizes do sistema contínuo

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


% Definir as matrizes do sistema contínuo
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

C = [0 1 0 0 0 0];

D = 0;

% Período de amostragem
T = 0.1;

% Matriz identidade
I = eye(size(A));

% Aplicar a transformada bilinear para obter as matrizes discretas
A_d = (I - (T/2) * A) \ (I + (T/2) * A);
B_d = (I - (T/2) * A) \ (T * B);

% C e D permanecem os mesmos
C_d = C;
D_d = D;

% Criar o sistema discreto no espaço de estados
sys_d = ss(A_d, B_d, C_d, D_d, T);

t = 0:T:10;
u = ones(size(t));
% Usar lsim para obter a resposta do sistema e os estados
[y, t, x] = lsim(sys_d, u, t);

% Plotar a resposta ao degrau
figure;
subplot(2, 1, 1);
stairs(t, y, 'LineWidth', 1.5);
title('Resposta ao Degrau do Sistema Discretizado');
xlabel('Tempo (s)');
ylabel('Saída');
grid on;

% Plotar os estados individuais
subplot(2, 1, 2);
stairs(t, x, 'LineWidth', 1.5);
title('Estados do Sistema Discretizado');
xlabel('Tempo (s)');
ylabel('Estados');
legend('x1', 'x2', 'x3', 'x4');
grid on;

%%


k1 = 696;
k2 = 595;
k3 = 461;
k4 = 454;

m1 = 0.31;
m2 = 0.712;
m3 = 0.383;


% Definir as matrizes do sistema contínuo
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

% Período de amostragem
T = 0.001;

% Matriz identidade
I = eye(size(A));

% Aplicar a transformada bilinear para obter as matrizes discretas
A_d = (I - (T/2) * A) \ (I + (T/2) * A);
B_d = (I - (T/2) * A) \ (T * B);

% C e D permanecem os mesmos
C_d = C;
D_d = D;

% Criar o sistema discreto no espaço de estados
sys_d = ss(A_d, B_d, C_d, D_d, T);

% Vetor de tempo e entrada degrau
t = 0:T:20;
u = ones(size(t)) * 100;

% Simular a resposta do sistema discretizado

x_zero = [0.1; 0; 0; 0; 0; 0;];

[y, t, x] = lsim(sys_d, u, t, x_zero);

% Plotar as posições juntas em um gráfico
figure;
subplot(2, 1, 1);
stairs(t, x(:, 1), 'r', 'LineWidth', 1.5); hold on;
stairs(t, x(:, 2), 'b', 'LineWidth', 1.5); hold on;
stairs(t, x(:, 3), 'g', 'LineWidth', 1.5); 
title('Posições dos Blocos');
xlabel('Tempo (s)');
ylabel('Posição (m)');
legend('Posição Bloco 1 (x1)', 'Posição Bloco 2 (x2)', 'Posição Bloco 3 (x3)');
grid on;
hold off;

% Plotar as velocidades juntas em um gráfico
subplot(2, 1, 2);
stairs(t, x(:, 4), 'r', 'LineWidth', 1.5); hold on;
stairs(t, x(:, 5), 'b', 'LineWidth', 1.5); hold on;
stairs(t, x(:, 6), 'g', 'LineWidth', 1.5);
title('Velocidades dos Blocos');
xlabel('Tempo (s)');
ylabel('Velocidade (m/s)');
legend('Velocidade Bloco 1 (x4)', 'Velocidade Bloco 2 (x5)', 'Velocidade Bloco 3 (x6)');
grid on;
hold off;
