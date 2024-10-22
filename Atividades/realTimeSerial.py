import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import time

# Configuração da porta serial (ajustar conforme necessário)
ser = serial.Serial('COM7', 115200, timeout=0.1)  # Ajuste 'COM7' conforme a porta do seu Arduino

# Configurações da janela de plotagem usando PyQtGraph
app = QtGui.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="Leitura Serial em Tempo Real")
win.resize(800, 600)
win.setWindowTitle('PID Controller - Arduino UNO')

# Criando os gráficos
plot = win.addPlot(title="System Output (Actual) and Setpoint vs Time")
curve_actual = plot.plot(pen='b', name='Actual')  # Curva para os valores reais
curve_setpoint = plot.plot(pen='w', name='Setpoint', linestyle='--')  # Curva para os valores de setpoint

# Listas para armazenar os dados
time_data = []
actual_data = []
setpoint_data = []

# Duração total da coleta de dados (em segundos)
total_duration = 40
start_time = time.time()

print(f"Coletando dados por {total_duration} segundos...")

def update():
    """ Função de atualização chamada a cada iteração. """
    global time_data, actual_data, setpoint_data
    
    # Lendo os dados da porta serial
    line = ser.readline().decode('utf-8').strip()
    if line:
        data = line.split(',')
        if len(data) == 5:  # Certificando que há 5 valores
            try:
                # Capturando o tempo, setpoint e valor atual
                current_time = float(data[0])  # Primeiro valor é o tempo
                setpoint = float(data[1])  # Segundo valor é o setpoint
                actual = float(data[2])  # Terceiro valor é o "actual"
                
                # Calcula o tempo decorrido em relação ao início da coleta
                elapsed_time = current_time - start_time
                
                # Adiciona os dados nas listas
                time_data.append(elapsed_time)
                setpoint_data.append(setpoint)
                actual_data.append(actual)
                
                # Atualiza as curvas do gráfico
                curve_actual.setData(time_data, actual_data)
                curve_setpoint.setData(time_data, setpoint_data)
                
            except ValueError:
                pass  # Ignora erros de conversão

# Configurando o timer para atualizar os gráficos em tempo real
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)  # Atualiza a cada 50 ms

# Executando a aplicação PyQtGraph
if __name__ == '__main__':
    try:
        QtGui.QApplication.instance().exec_()
    except KeyboardInterrupt:
        print("Coleta de dados interrompida pelo usuário.")
    finally:
        ser.close()  # Fechar a porta serial corretamente
