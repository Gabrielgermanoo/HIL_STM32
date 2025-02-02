#include <stdio.h>
#include <time.h>

// Função para multiplicação de matriz e vetor
void mat_vec_mul(double *result, double *matrix, double *vector, int rows, int cols) {
    for (int i = 0; i < rows; ++i) {
        result[i] = 0.0;
        for (int j = 0; j < cols; ++j) {
            result[i] += matrix[i * cols + j] * vector[j];
        }
    }
}

// Função para calcular a derivada do estado: f(x, u) = A * x + B * u
void state_derivative(double *dx, double *A, double *B, double *x, double *u, int n) {
    mat_vec_mul(dx, A, x, n, n); // dx = A * x
    double Bu[n];
    mat_vec_mul(Bu, B, u, n, 1); // Bu = B * u
    for (int i = 0; i < n; ++i) {
        dx[i] += Bu[i]; // dx = A * x + B * u
    }
}

// Função de integração por Runge-Kutta de 4ª ordem para sistemas de espaço de estados
void rk4_step(double *x, double *A, double *B, double *u, double h, int n) {
    double k1[n], k2[n], k3[n], k4[n], xtemp[n];

    // k1 = h * f(x, u)
    state_derivative(k1, A, B, x, u, n);
    for (int i = 0; i < n; ++i) {
        k1[i] *= h;
    }

    // k2 = h * f(x + 0.5 * k1, u)
    for (int i = 0; i < n; ++i) {
        xtemp[i] = x[i] + 0.5 * k1[i];
    }
    state_derivative(k2, A, B, xtemp, u, n);
    for (int i = 0; i < n; ++i) {
        k2[i] *= h;
    }

    // k3 = h * f(x + 0.5 * k2, u)
    for (int i = 0; i < n; ++i) {
        xtemp[i] = x[i] + 0.5 * k2[i];
    }
    state_derivative(k3, A, B, xtemp, u, n);
    for (int i = 0; i < n; ++i) {
        k3[i] *= h;
    }

    // k4 = h * f(x + k3, u)
    for (int i = 0; i < n; ++i) {
        xtemp[i] = x[i] + k3[i];
    }
    state_derivative(k4, A, B, xtemp, u, n);
    for (int i = 0; i < n; ++i) {
        k4[i] *= h;
    }

    // Atualização do estado: x = x + (1/6) * (k1 + 2*k2 + 2*k3 + k4)
    for (int i = 0; i < n; ++i) {
        x[i] += (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0;
    }
}

int main() {
    // Exemplo de uso
    int n = 2; // Número de estados
    double A[4] = {1.0, 0.010, -0.010, -0.9995}; // Matriz A (2x2)
    double B[2] = {0.0, 0.0010}; // Matriz B (2x1)
    double u[1] = {1.0}; // Entrada do sistema
    double x[2] = {0.0, 0.0}; // Estado inicial
    double h = 0.01; // Passo de integração

    // Realizar a integração por alguns passos
    for (int i = 0; i < 5000; ++i) {
        rk4_step(x, A, B, u, h, n);
        printf("Passo %d: x[0] = %f, x[1] = %f\n", i, x[0], x[1]);
    }

    return 0;
}
