/*
 * Copyright (c) 2022 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include <string.h>

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 32

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/**
* \brief Read characters from UART until line end is detected. Afterward push the data to the message queue.
 * \param dev device used for the callback
 * \param user_data pointer to the variable that will store the received data
 */
void serial_cb(const struct device *dev, void *user_data);

/**
 * \brief Print a null-terminated string character by character to the UART interface
 * \param buf buffer containing the string to be printed
 */
void print_uart(char *buf);

/**
 * \brief Operation to multiply the matrix by the vector
 * \param result vector to store the result of the operation
 * \param matrix matrix to be multiplied
 * \param vector vector to be multiplied
 * \param rows number of rows of the matrix
 * \param cols number of columns of the matrix
 */
void mat_vec_mul(double *result, const double *matrix, const double *vector, const int rows, const int cols);

/**
 * \brief der
 * \param dx 
 * \param A 
 * \param B 
 * \param x 
 * \param u 
 * \param n 
 */
void state_derivative(double *dx, double *A, double *B, double *x, double *u, int n);

/**
 * \brief 
 * \param x
 * \param A 
 * \param B 
 * \param u 
 * \param h 
 * \param n 
 */
void rk4_step(double *x, double *A, double *B, double *u, double h, int n);

void print_uart(char *buf) {
    int msg_len = strlen(buf);

    for (int i = 0; i < msg_len; i++) {
        uart_poll_out(uart_dev, buf[i]);
    }
}

void serial_cb(const struct device *dev, void *user_data) {
    uint8_t c;
    double *u = (double *) user_data;

    if (!uart_irq_update(uart_dev)) {
        return;
    }

    if (!uart_irq_rx_ready(uart_dev)) {
        return;
    }

    /* read until FIFO empty */
    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
            /* terminate string */
            rx_buf[rx_buf_pos] = '\0';

            /* if queue is full, message is silently dropped */
            k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

            /* reset the buffer (it was copied to the msgq) */
            rx_buf_pos = 0;

            double new_u = atof(rx_buf);
            u[0] = new_u;

        } else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
            rx_buf[rx_buf_pos++] = c;
        }
        /* else: characters beyond buffer size are dropped */
    }
}

void mat_vec_mul(double *result, const double *matrix, const double *vector, const int rows, const int cols) {
    for (int i = 0; i < rows; ++i) {
        result[i] = 0.0;
        for (int j = 0; j < cols; ++j) {
            result[i] += matrix[i * cols + j] * vector[j];
        }
    }
}

void state_derivative(double *dx, double *A, double *B, double *x, double *u, int n) {
    mat_vec_mul(dx, A, x, n, n); // dx = A * x
    double Bu[n];
    mat_vec_mul(Bu, B, u, n, 1); // Bu = B * u
    for (int i = 0; i < n; ++i) {
        dx[i] += Bu[i]; // dx = A * x + B * u
    }
}

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

void spring_mass(double *x, double *A, double *B, double *u, const double h, const int n) {
    for (int i = 0; i < 10000; ++i) {
        rk4_step(x, A, B, u, h, n);
        printk("%f %f %f %f \n", x[0], x[1], x[2], x[3]);
    }
    print_uart("End of simulation!");
}

int main(void) {
    char tx_buf[MSG_SIZE];
    int n = 4; // Número de estados
    double A[16] = {
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
        -30.0, 30.0, -3.0, 3.0,
        7.5, -42.5, 0.75, -2.25
    }; // Matriz A (4x4)
    double B[4] = {0.0, 0.0, 0.2, 0.0}; // Matriz B (4x1)
    double u[1] = {100.0}; // Entrada do sistema
    double x[4] = {0.1, 0.0, 0.0, 0.0}; // Estado inicial
    double h = 0.001; // Passo de integração

    if (!device_is_ready(uart_dev)) {
        printk("UART device not found!");
        return 0;
    }

    /* configure interrupt and callback to receive data */
    int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, u);

    if (ret < 0) {
        if (ret == -ENOTSUP) {
            printk("Interrupt-driven UART API support not enabled\n");
        } else if (ret == -ENOSYS) {
            printk("UART device does not support interrupt-driven API\n");
        } else {
            printk("Error setting UART callback: %d\n", ret);
        }
        return 0;
    }
    uart_irq_rx_enable(uart_dev);

    while(k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0) {
        print_uart(tx_buf);
        switch (rx_buf[0]) {
            case 's':
                spring_mass(x, A, B, u, h, n);
                k_msgq_cleanup(&uart_msgq);
                break;
            default:
                print_uart("Unrecognized command\n");
                break;
        }
    }

    return 0;
}