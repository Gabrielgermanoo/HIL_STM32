#include <Arduino.h>

// Definir as dimensões das matrizes
#define ROWS_A 4
#define COLS_A 4
#define ROWS_B 4
#define COLS_B 1
#define ROWS_C 1
#define COLS_C 4

float A_d[ROWS_A][COLS_A] = {
    {0.9927, 0.0071, 0.0215, 0.0008},
    {0.0018, 0.9896, 0.0002, 0.0217},
    {-0.6406, 0.6136, 0.9287, 0.0700},
    {0.1570, -0.9166, 0.0175, 0.9414}
};

float B_d[ROWS_B][COLS_B] = {
    {0.0000},
    {0.0000},
    {0.0043},
    {0.0000}
};

float C_d[ROWS_C][COLS_C] = {
    {1, 0, 0, 0}
};

float D_d[1][1] = {
    {0}
};

float x[ROWS_A][1] = { {0}, {0}, {0}, {0} };

void matrixMultiply(float a[][COLS_A], float b[][1], float result[][1], int rowsA, int colsA, int colsB) {
    for (int i = 0; i < rowsA; i++) {
        result[i][0] = 0;
        for (int j = 0; j < colsA; j++) {
            result[i][0] += a[i][j] * b[j][0];
        }
    }
}

void matrixAdd(float a[][1], float b[][1], float result[][1], int rows) {
    for (int i = 0; i < rows; i++) {
        result[i][0] = a[i][0] + b[i][0];
    }
}

float stepResponse(float u) {
    float temp_x[ROWS_A][1];
    float temp_Bu[ROWS_B][1] = { {0}, {0}, {0}, {0} };
    float temp_Cx[1][1] = { {0} };

    // x(k+1) = A_d * x(k) + B_d * u(k)
    matrixMultiply(A_d, x, temp_x, ROWS_A, COLS_A, 1);  // A_d * x(k)

    for (int i = 0; i < ROWS_B; i++) {
        temp_Bu[i][0] = B_d[i][0] * u;  // B_d * u(k)
    }

    matrixAdd(temp_x, temp_Bu, x, ROWS_A);  // x(k+1) = A_d * x(k) + B_d * u(k)

    // y(k) = C_d * x(k) + D_d * u(k)
    matrixMultiply(C_d, x, temp_Cx, ROWS_C, COLS_C, 1);  // C_d * x(k)

    return temp_Cx[0][0];  // Retornando y(k)
}

void setup() {
    Serial.begin(115200);

    // Vetor de entrada de impulso
    float impulse_input[5] = {1, 0, 0, 0, 0};

    // Iterar sobre o vetor de entrada de impulso
    for (int i = 0; i < 5; i++) {
        float u = impulse_input[i];

        float y = stepResponse(u);  

        Serial.println(y); 
        delay(100);  
    }
}

void loop() {

}
