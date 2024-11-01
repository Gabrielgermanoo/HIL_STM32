#include <stdio.h>
#include <math.h>

// Função que representa f(x) = sin(x)
double f(double x) {
    return sin(x);
}

// Método de Runge-Kutta de quarta ordem para integração numérica
double rungeKutta(double a, double b, int n) {
    double h = (b - a) / n; // Tamanho do passo
    double x = a;
    double integral = 0.0;

    // Loop para calcular a integral
    for (int i = 0; i < n; i++) {
        double k1 = f(x);
        double k2 = f(x + h / 2.0);
        double k3 = f(x + h / 2.0);
        double k4 = f(x + h);

        printf("k1: %f, k2: %f, k3: %f, k4: %f\n", k1, k2, k3, k4);
        
        // Atualiza a integral
        integral += (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
        
        // Avança para o próximo ponto
        x += h;
    }

    return integral;
}

int main() {
    double a = 0.0; // Limite inferior
    double b = 3.14; // Limite superior (por exemplo, integral de 0 a pi)
    int n = 100; // Número de passos

    double resultado = rungeKutta(a, b, n);
    printf("A integral de sin(x) de %.2f a %.2f é aproximadamente %.5f\n", a, b, resultado);

    return 0;
}