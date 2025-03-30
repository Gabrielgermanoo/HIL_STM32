#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 32

#define SIMULATION_TIME 10000

#define SIMULATION_STEP 0.001

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/**
 * @brief UART callback function to handle incoming data
 * 
 * @param dev The UART device
 * @param user_data Pointer to user data (in this case, the input vector u)
 * @note This function is called in the context of the UART interrupt handler.
 */
void serial_cb(const struct device *dev, void *user_data);

/**
 * \brief Print a null-terminated string character by character to the UART interface
 * \param buf a null-terminated string to be printed
 * \note This function is blocking and will wait until the entire string is printed before returning.
 */
void print_uart(char *buf);

/**
 * @brief Function to calculate the state derivative of the system
 * 
 * @param dx The resulting state derivative
 * @param A The matrix A of the system
 * @param B The matrix B of the system
 * @param x The current state vector
 * @param u The input vector
 * @param n The number of states
 */
void state_derivative(double *dx, double *A, double *B, double *x, double *u, int n);

/**
 * @brief Function to perform a single RK4 step for the system
 * 
 * @param x The current state vector
 * @param A The matrix A of the system
 * @param B The matrix B of the system
 * @param u The input vector
 * @param h The time step size
 * @param n The number of states
 */
void rk4_step(double *x, double *A, double *B, double *u, double h, int n);


static void reset_system_handler(struct k_work *work) {
    k_msleep(2000);

    sys_reboot(SYS_REBOOT_COLD);
}


K_WORK_DEFINE(reset_system, reset_system_handler);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

void print_uart(char *buf) {
    int msg_len = strlen(buf);

    for (int i = 0; i < msg_len; i++) {
        uart_poll_out(uart_dev, buf[i]);
    }
}


void mat_vec_mul(double *result, double *matrix, double *vector, int rows, int cols) {
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

    /**
     * update of state: x = x + (1/6) * (k1 + 2*k2 + 2*k3 + k4)
     */
    for (int i = 0; i < n; ++i) {
        x[i] += (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0;
    }
}

void spring_mass(double *x, double *A, double *B, double *u, const double h, const int n) {
    printk("Spring mass simulation started.\n");
      for (int i = 0; i < SIMULATION_TIME; ++i) {
        rk4_step(x, A, B, u, h, n);
        printk("%f %f %f %f %f %f \n", (double)x[0], (double)x[1], (double)x[2], (double)x[3], (double)x[4], (double)x[5]);
      }
}

int main(void) {

    int n = 6;
    double A[36] = {
        0.0,     0.0,     0.0,     1,  0.0,     0.0,
        0.0,     0.0,     0.0,     0.0,     1,  0.0,
        0.0,     0.0,     0.0,     0.0,     0.0,     1,
       -325.8065, -1919.4000,  0.0,     0.0,    -258.0645,  0.0,
       835.6742, -188.2022, -647.4719,  112.3596, -32.3596, -112.3596,
        0.0,     1203.6553, -18.2768,  0.0,     208.8773,  0.0
    };
    double B[6] = {0.0, 0.0, 0.0, 1.4045, 0.000, 0.00};
    double u[1] = {100};
    double x[6] = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0};
    double h = SIMULATION_STEP;

    if (!device_is_ready(uart_dev)) {
        LOG_DBG("UART device not found!");
        return 0;
    }

    /* configure interrupt and callback to receive data */
    int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, u);

    if (ret < 0) {
        if (ret == -ENOTSUP) {
            LOG_ERR("Interrupt-driven UART API support not enabled\n");
        } else if (ret == -ENOSYS) {
            LOG_ERR("UART device does not support interrupt-driven API\n");
        } else {
            printk("Error setting UART callback: %d\n", ret);
        }
        return 0;
    }

    uart_irq_rx_enable(uart_dev);
    
    // while (1) {
    //     k_msgq_get(&uart_msgq, &rx_buf, K_FOREVER);
    //     rx_buf_pos = strlen(rx_buf);
    //     if (rx_buf_pos == 0) {
    //         continue;
    //     }

    //     switch (rx_buf[0]) {
    //         case '1':
    //             spring_mass(x, A, B, u, h, n);
    //             memset(rx_buf, 0, sizeof(rx_buf));
    //             break;
    //         case '2':
    //             print_uart("Simulation stopped.\n");
    //             break;
    //         default:
    //             print_uart("Invalid command. Please try again.\n");
    //             break;
    //     }

    //     memset(rx_buf, 0, sizeof(rx_buf));
    //     rx_buf_pos = 0;
    // }
    int i = 0;
    while(1) {
        if (i > SIMULATION_TIME) {
            break;
        }
        // Verify the input obtained from the UART
        if (k_msgq_get(&uart_msgq, &rx_buf, K_NO_WAIT) == 0) {
            rx_buf_pos = strlen(rx_buf);
            if (rx_buf_pos == 0) {
                continue;
            }
            double new_u = atof(rx_buf);
            u[0] = new_u;
            memset(rx_buf, 0, sizeof(rx_buf));
        }
        // reset the system if the input is 0
        if (u[0] == 0) {
            print_uart("Resetting system...\n");
            k_work_submit(&reset_system);
            break;
        }
        rk4_step(x, A, B, u, h, n);
        printk("%f %f %f\n", x[0], x[1], x[2]);
        i++;
    }

    return 0;
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

            LOG_DBG("Received: %s", rx_buf);

            /* if queue is full, message is silently dropped */
            if (k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT) != 0) {
                LOG_ERR("Message queue full, dropping data");
            }

            /* reset the buffer (it was copied to the msgq) */
            rx_buf_pos = 0;

            double new_u = atof(rx_buf);
            u[0] = new_u;

        } else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
            rx_buf[rx_buf_pos++] = c;
        } else {
            LOG_ERR("Buffer overflow, dropping character");
        }
        /* else: characters beyond buffer size are dropped */
    }
}