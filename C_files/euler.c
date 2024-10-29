#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>

// Definições das dimensões das matrizes
#define ROWS_A 4
#define COLS_A 4
#define ROWS_B 4
#define COLS_B 1
#define ROWS_C 1
#define COLS_C 4

// Matrizes do sistema discreto
float A_d[ROWS_A][COLS_A] = {
    { 1.0000, 0.0000, 0.0010, 0.0000 },
    { 0.0000, 1.0000, 0.0000, 0.0010 },
    { -0.0299, 0.0299, 0.9970, 0.0030 },
    { 0.0075, -0.0424, 0.0008, 0.9977 }
};

float B_d[ROWS_B][COLS_B] = {
{ 0.0001 },
{ 0.0000 },
{ 0.1997 },
{ 0.0001 }
};

float C_d[ROWS_C][COLS_C] = {
  { 1, 0, 0, 0 }
};
float D_d[1][1] = {
  { 0 }
};

// Vetor de estado inicial
float x[ROWS_A] = { 0.1, 0, 0, 0 };
float u[ROWS_A] = { 100, 100, 100, 100 };

// Função para calcular a derivada dos estados
static inline float x1_dot() {
  return A_d[0][0] * x[0] + A_d[0][1] * x[1] + A_d[0][2] * x[2] + A_d[0][3] * x[3] + B_d[0][0] * u[0];
}

static inline float x2_dot() {
  return A_d[1][0] * x[0] + A_d[1][1] * x[1] + A_d[1][2] * x[2] + A_d[1][3] * x[3] + B_d[1][0] * u[1];
}

static inline float x3_dot() {
  return A_d[2][0] * x[0] + A_d[2][1] * x[1] + A_d[2][2] * x[2] + A_d[2][3] * x[3] + B_d[2][0] * u[2];
}

static inline float x4_dot() {
  return A_d[3][0] * x[0] + A_d[3][1] * x[1] + A_d[3][2] * x[2] + A_d[3][3] * x[3] + B_d[3][0] * u[3];
}

int main() {

  unsigned long long i = 1;
  float x1 = 0.1, x2 = 0, x3 = 0, x4 = 0;
  clock_t start;
  float x_dot[ROWS_A];
  float delta_t = 0.01;  // Intervalo de tempo (passo de integração)

  while (true) {
    start = clock();

    // Calcula as derivadas dos estados
    x_dot[0] = x1_dot();
    x_dot[1] = x2_dot();
    x_dot[2] = x3_dot();
    x_dot[3] = x4_dot();

    // Integra os estados usando o método de Euler
    x1 = x[0] + x_dot[0] * delta_t;
    x2 = x[1] + x_dot[1] * delta_t;
    x3 = x[2] + x_dot[2] * delta_t;
    x4 = x[3] + x_dot[3] * delta_t;
    
    // Atualiza os estados
    x[0] = x1;
    x[1] = x2;
    x[2] = x3;
    x[3] = x4;
    
    printf("x1: %f, x2: %f, x3: %f, x4: %f\n", x1, x2, x3, x4);
    printf("Tempo de execucao: %lf segundos\n", ((double)clock() - start) / CLOCKS_PER_SEC);
    _sleep(1);
    i++;
    printf("Time: %lf\n", i * delta_t);
  }
  return 0;
}
