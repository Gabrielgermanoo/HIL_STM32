import asyncio
import struct
from bleak import BleakClient
import time

# Endereço do dispositivo BLE
address = "C1:06:55:F1:4D:A2"

# UUIDs das características
UUID_ENVIO = "00000005-1000-2000-3000-111122223333"  # Para envio
UUID_RECEBIMENTO = "00000006-1000-2000-3000-111122223333"  # Para recebimento


# Callback para receber notificações
def notification_handler(sender, data):
    timestamp, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z = (
        struct.unpack("<Ihhhhhhhhh", data)
    )
    
    acc_x *= 2 * 9.81 / 32768
    acc_y *= 2 * 9.81 / 32768
    acc_z *= 2 * 9.81 / 32768

    current_time = time.time()
    
    print(f"Timestamp: {timestamp}")
    print(f"Acelerômetro - X: {acc_x}, Y: {acc_y}, Z: {acc_z}")
    
    # Armazenar os dados em um arquivo
    with open("./bluetooth/dados_sensores.csv", "a") as file:
        file.write(f"{current_time},{acc_x},{acc_y},{acc_z}\n")
    

async def communicate():
    async with BleakClient(address) as client:
        print(f"🔗 Conectado a {address}")

        if client.is_connected:
            # Ativar notificações para receber a resposta
            await client.start_notify(UUID_RECEBIMENTO, notification_handler)

            time.sleep(5)
            
            mensagem = bytes([0x07, 0x00, 0x00, 0x00, 0x00])
            await client.write_gatt_char(UUID_ENVIO, mensagem)
            print(f"Mensagem enviada: {mensagem.hex()}")

            # Aguardar notificações por 10 segundos (ajuste conforme necessário)
            await asyncio.sleep(20)

            # Desativar notificações
            await client.stop_notify(UUID_RECEBIMENTO)

        else:
            print("Falha ao conectar ao dispositivo.")


# Executar comunicação
asyncio.run(communicate())
