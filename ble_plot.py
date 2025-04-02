import pandas as pd
from matplotlib import pyplot as plt

def plot_ble_data(data):
    df = pd.read_csv(data, names=["timestamp", "acc_X", "acc_Y", "acc_Z"])
    
    # Plotando os dados
    plt.figure(figsize=(10, 6))
    plt.plot(df["timestamp"], df["acc_X"], label="Aceleração X", color='r')
    plt.plot(df["timestamp"], df["acc_Y"], label="Aceleração Y", color='g')
    plt.plot(df["timestamp"], df["acc_Z"], label="Aceleração Z", color='b')

    # Personalizando o gráfico
    plt.title("Dados do Acelerômetro", fontsize=14)
    plt.xlabel("Tempo (ms)", fontsize=12)
    plt.ylabel("Aceleração (mg)", fontsize=12)
    plt.legend(loc="upper right")
    plt.grid(True)
    
    plt.xticks(rotation=45)
    
    plt.tight_layout()
    plt.show()

data_file = "./bluetooth/dados_sensores.csv"

plot_ble_data(data_file)
