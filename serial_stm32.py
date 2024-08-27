import numpy as np
from scipy.signal import StateSpace, lti
import subprocess
import serial
import time
import matplotlib.pyplot as plt
import stm32loader

serial_port = "COM1"


# Configurações da porta serial
ser = serial.Serial(port=serial_port, baudrate=9600, timeout=1)

def send_data(data):
    ser.write(data.encode())
    time.sleep(0.1)
    print('Data sent: ' + data)

def receive_data():
    data = ser.readline().decode('utf-8').strip()
    print('Data received: ' + data)
    return data

try:
    # Verificar se a porta está aberta
    if ser.isOpen():
        print(ser.name + ' is open...')

    # Parâmetros do sistema
    L = 1e-3     # Indutância (H)
    R = 1        # Resistência (Ohms)
    M = 0.02     # Massa (kg)
    b1 = 4.1e-3  # Atrito (N/(m/s))
    Km = 0.1025  # Constante do motor (N/A)

    # Matrizes de estado
    A = np.array([[0, 1],
                  [-R*b1/(L*M), -(L*b1 + R*M)/(L*M)]])
    
    B = np.array([[0],
                  [Km/(L*M)]])
    
    C = np.array([[1, 0]])
    
    D = np.array([[0]])

    # Cria o sistema no espaço de estados
    system = StateSpace(A, B, C, D)

    # Definir a resposta ao degrau
    t, y = lti(A, B, C, D).step()

    # Enviar valores via serial
    for i in range(len(t)):
        data_to_send = f'Tempo: {t[i]:.4f}, Resposta: {y[i]:.4f}'
        send_data(data_to_send)
        time.sleep(0.5)  # Aguardar meio segundo entre envios para evitar sobrecarga na serial

        # Opcional: Receber confirmação do dispositivo
        response = receive_data()
        print(response)
        
    # Plota a resposta ao degrau (opcional)
    plt.plot(t, y)
    plt.xlabel('Tempo [s]')
    plt.ylabel('Resposta')
    plt.title('Resposta ao Degrau do Sistema')
    plt.grid(True)
    plt.show()

finally:
    ser.close()
    print(ser.name + ' is closed...')