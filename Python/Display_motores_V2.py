import tkinter as tk
import serial
import threading
import time
import math

# Configurações iniciais
largura_retangulo = 400
altura_retangulo = 600
raio_esfera = 5
velocidade = 1
novo_x = 50
novo_y = 50

# Abrir a conexão serial
esp32 = serial.Serial('COM13', baudrate=115200, timeout=1)

# Função para mover a esfera aleatoriamente
def mover_esfera():
    global novo_x
    global novo_y
    x_atual, y_atual, _, _ = canvas.coords(esfera)
    novo_x = novo_x + 50
    novo_y = novo_y + 50
    
    # Atualize a posição da esfera
    canvas.move(esfera, novo_x - x_atual-raio_esfera, novo_y - y_atual-raio_esfera)
    
    # Atualize a posição das linhas
    for linha in linhas:
        canvas.coords(linha, novo_x, novo_y, canvas.coords(linha)[2], canvas.coords(linha)[3])


# Função para atualizar os rótulos dos motores com os dados recebidos da porta serial
def atualizar_rotulos_motores(motores):
    for i, (ident, estado, comprimento, comp_final) in enumerate(motores):
        motor_id_label = motor_labels[i * 4]
        motor_estado_label = motor_labels[i * 4 + 1]
        motor_comprimento_label = motor_labels[i * 4 + 2]
        motor_comp_final_label = motor_labels[i * 4 + 3]

        motor_id_label.config(text=f"ID: {ident}", fg="green" if estado else "red")
        motor_estado_label.config(text=f"Status: {estado}", fg="green" if estado else "red")
        motor_comprimento_label.config(text=f"Comprimento atual: {comprimento:.3f}", fg="green" if estado else "red")
        motor_comp_final_label.config(text=f"Comprimento final: {comp_final:.3f}", fg="green" if estado else "red")

# Função para ler os dados dos motores da porta serial
def ler_dados_motores():
    global novo_x
    global novo_y
    global esp32
        
    # Loop para ler os dados dos motores
    while True:
        motores = []
        dado = False
        while not dado:
            linha = esp32.readline().decode().strip()
            if "Dados" in linha:
                dado = True
        if dado:
            linha = esp32.readline().decode().strip()
            e = linha.split(";")
            linha = esp32.readline().decode().strip()
            c = linha.split(";")
            linha = esp32.readline().decode().strip()
            f = linha.split(";")
            novo_x = int(f[0])
            novo_y = int(f[1])
            for ident in range(4):
                estado = int(e[ident]) != 0
                comprimento = int(c[ident])
                match ident:
                    case 0:
                        comp_final = math.sqrt(math.pow(novo_x,2)+(math.pow(novo_y,2)))
                    case 1:
                        comp_final = math.sqrt(math.pow(largura_retangulo-novo_x,2)+(math.pow(novo_y,2)))
                    case 2:
                        comp_final = math.sqrt(math.pow(novo_x,2)+(math.pow(altura_retangulo-novo_y,2)))
                    case 3:
                        comp_final = math.sqrt(math.pow(largura_retangulo-novo_x,2)+(math.pow(altura_retangulo-novo_y,2)))

                motores.append((ident, estado, comprimento, comp_final))
               
            # Atualizar os rótulos na interface gráfica com os valores obtidos
            mover_esfera()
            janela.after(0, atualizar_rotulos_motores, motores)

# Função para enviar as coordenadas para o ESP32
def enviar_coordenadas():
    global esp32
    x = int(coordenada_x_entry.get())
    y = int(coordenada_y_entry.get())
    coordenadas = f"{x},{y}\n"
    esp32.write(coordenadas.encode())
    print(f"Enviando coordenadas x={x}, y={y} para o ESP32")

# Criar a janela principal
janela = tk.Tk()
janela.title("Dados dos Motores")

# Crie o canvas
canvas = tk.Canvas(janela, width=largura_retangulo+100, height=altura_retangulo+100)
canvas.pack(side=tk.RIGHT)  # Coloque a animação à esquerda

# Desenhe o retângulo
canvas.create_rectangle(50, 50, largura_retangulo+50, altura_retangulo+50, outline='black')

# Desenhe os quadrados nos cantos e as linhas
quadrados = []
linhas = []
for x, y in [(50, 50), (50, altura_retangulo+50), (largura_retangulo+50, 50), (largura_retangulo+50, altura_retangulo+50)]:
    quadrado = canvas.create_rectangle(x - raio_esfera, y - raio_esfera, x + raio_esfera, y + raio_esfera, fill='blue')
    quadrados.append(quadrado)
    linha = canvas.create_line(largura_retangulo/2, altura_retangulo/2, x, y, fill='gray')
    linhas.append(linha)

# Desenhe a esfera central
esfera = canvas.create_oval(largura_retangulo/2 - raio_esfera, altura_retangulo/2 - raio_esfera,
                            largura_retangulo/2 + raio_esfera, altura_retangulo/2 + raio_esfera, fill='red')

# Crie um Frame para os rótulos dos motores
motor_frame = tk.Frame(janela)
motor_frame.pack(side=tk.LEFT, padx=10, pady=10)

# Criar rótulos para exibir os dados dos motores
motor_labels = []

for i in range(4):
    motor_id_label = tk.Label(motor_frame, text="ID: -")
    motor_estado_label = tk.Label(motor_frame, text="Status: -")
    motor_comprimento_label = tk.Label(motor_frame, text="Comprimento atual: -")
    motor_comp_final_label = tk.Label(motor_frame, text="Comprimento final: -")

    motor_labels.extend([motor_id_label, motor_estado_label, motor_comprimento_label, motor_comp_final_label])

# Posicione os rótulos dos motores verticalmente
for label in motor_labels:
    label.pack(anchor=tk.W)

# Crie um Frame para as entradas de coordenadas e o botão de envio
coordenadas_frame = tk.Frame(janela)
coordenadas_frame.pack(side=tk.LEFT, padx=10, pady=10)

# Crie entradas de texto para coordenadas x e y
coordenada_x_label = tk.Label(coordenadas_frame, text="Coordenada X:")
coordenada_x_label.pack(anchor=tk.W)
coordenada_x_entry = tk.Entry(coordenadas_frame)
coordenada_x_entry.pack()

coordenada_y_label = tk.Label(coordenadas_frame, text="Coordenada Y:")
coordenada_y_label.pack(anchor=tk.W)
coordenada_y_entry = tk.Entry(coordenadas_frame)
coordenada_y_entry.pack()

# Crie um botão para enviar as coordenadas
enviar_coordenadas_button = tk.Button(coordenadas_frame, text="Enviar Coordenadas", command=enviar_coordenadas)
enviar_coordenadas_button.pack()

# Iniciar a thread para ler os dados dos motores
thread_leitura = threading.Thread(target=ler_dados_motores)
thread_leitura.daemon = True
thread_leitura.start()

# Iniciar a interface gráfica
mover_esfera()
janela.mainloop()
