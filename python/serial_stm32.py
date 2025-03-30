from machine import Pin, UART
import time
import numpy as np

uart = UART(2, baudrate=115200, tx=Pin(17), rx=Pin(16))

A_d = np.array([[0.9927, 0.0071, 0.0215, 0.0008],
                [0.0018, 0.9896, 0.0002, 0.0217],
                [-0.6406, 0.6136, 0.9287, 0.0700],
                [0.1570, -0.9166, 0.0175, 0.9414]])

B_d = np.array([[0.0000],
                [0.0000],
                [0.0043],
                [0.0000]])

C_d = np.array([[1, 0, 0, 0]])
D_d = np.array([[0]])

x = np.zeros((4, 1)) 

def step_response(u):
    # Função para calcular a resposta do sistema discreto
    global x
    # Atualizar o estado x(k+1) = A_d * x(k) + B_d * u(k)
    x = np.dot(A_d, x) + np.dot(B_d, u)
    # Saída y(k) = C_d * x(k) + D_d * u(k)
    y = np.dot(C_d, x) + np.dot(D_d, u)
    return y

while True:
    if uart.any():
        u = float(uart.readline().decode('utf-8').strip())
        
        y = step_response(u)
        
        uart.write(str(y[0]) + '\n')  # Enviando apenas a primeira saída
        time.sleep(0.1)  # Pequeno delay para evitar sobrecarga