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
  { 0.9985, 0.0014, 0.0098, 0.0002 },
  { 0.0004, 0.9979, 0.0000, 0.0099 },
  { -0.2942, 0.2889, 0.9691, 0.0306 },
  { 0.0729, -0.4187, 0.0077, 0.9758 }
};
float B_d[ROWS_B][COLS_B] = {
  { 0.0000 },
  { 0.0000 },
  { 0.0020 },
  { 0.0000 }
};
float C_d[ROWS_C][COLS_C] = {
  { 1, 0, 0, 0 }
};
float D_d[1][1] = {
  { 0 }
};

// Vetor de estado inicial
float x[ROWS_A] = { 0, 0, 0, 0 };

float u[ROWS_A] = { 1, 1, 1, 1 };

float dxdt(float x, float y) {
  return ((x - y) / 2);
}

// Finds value of y for a given x using step size h
// and initial value x0 at t0.
float rungeKutta(float t0, float x0, float t, float h) {
  // Count number of iterations using step size or
  // step height h
  int n = (int)((t - t0) / h);

  float k1, k2, k3, k4, k5;

  // Iterate for number of iterations
  float y = x0;
  for (int i = 1; i <= n; i++) {
    // Apply Runge Kutta Formulas to find
    // next value of y
    k1 = h * dxdt(t0, y);
    k2 = h * dxdt(t0 + 0.5 * h, y + 0.5 * k1);

    // Update next value of y
    y = y + (1.0 / 6.0) * (k1 + 2 * k2);

    // Update next value of x
    t0 = t0 + h;
  }

  return y;
}

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
  float x1 = 0, x2 = 0, x3 = 0, x4 = 0;
  clock_t start;
  float x_dot[ROWS_A];
  int t = 0;
  while (true) {
    start = clock();

    x_dot[0] = x1_dot();
    x_dot[1] = x2_dot();
    x_dot[2] = x3_dot();
    x_dot[3] = x4_dot();

    x1 = rungeKutta(0, x_dot[1], i, 0.001);
    x2 = rungeKutta(0, x_dot[2], i, 0.001);
    x3 = rungeKutta(0, x_dot[3], i, 0.001);
    x4 = rungeKutta(0, x_dot[4], i, 0.001);
    
    x[0] = x1;
    x[1] = x2;
    x[2] = x3;
    x[3] = x4;
    
    printf("x1: %f, x2: %f, x3: %f, x4: %f\n", x1, x2, x3, x4);
    printf("tempo final do clock %f\n", ((double)clock() - start) / CLOCKS_PER_SEC);
    i++;
  }
  return 0;
}
