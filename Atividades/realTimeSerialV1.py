import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import time

# Configuração da porta serial (ajustar conforme necessário)
ser = serial.Serial('COM6', 115200, timeout=0.1)  # Ajuste 'COM6' conforme a porta do seu Arduino

# Configurações da janela de plotagem usando PyQtGraph
app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="Leitura Serial em Tempo Real")
win.resize(800, 600)
win.setWindowTitle('PID Controller - Arduino UNO')

# Criando os gráficos
plot = win.addPlot(title="System Output (Actual) and Setpoint vs Time")
curve_actual = plot.plot(pen='b', name='Actual')  # Curva para os valores reais
curve_setpoint = plot.plot(pen='w', name='Setpoint', linestyle='--')  # Curva para os valores de setpoint
curve_error = plot.plot(pen='r', name='Error', linestyle=':')  # Curva para o erro

# Listas para armazenar os dados
time_data = []
actual_data = []
setpoint_data = []
error_data = []


# Duração total da coleta de dados (em segundos)
total_duration = 40
window_duration = 10  # Duração da janela de tempo (em segundos)
start_time = time.time()

# Parâmetros PID
Kp = 0.67  # Ganho proporcional
Ki = 0.3  # Ganho integral
Kd = 0.05  # Ganho derivativo
integral = 0.0
last_error = 0.0

print(f"Coletando dados por {total_duration} segundos...")

def update():
    """ Função de atualização chamada a cada iteração. """
    global time_data, actual_data, setpoint_data, error_data#, pid_output_data
    global integral, last_error

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
                
                # Calcula e armazena o erro
                error = setpoint - actual
                error_data.append(error)

                # Cálculo do PID
                integral += error * 0.1  # Supondo um intervalo de tempo fixo de 0.1s
                derivative = (error - last_error) / 0.1
                pid_output = Kp * error + Ki * integral + Kd * derivative
                #pid_output_data.append(pid_output)

                # Atualiza as curvas do gráfico
                curve_actual.setData(time_data, actual_data)
                curve_setpoint.setData(time_data, setpoint_data)
                curve_error.setData(time_data, error_data)
                #curve_pid_output.setData(time_data, pid_output_data)

                # Mantém os dados dentro da janela de tempo
                while time_data and time_data[0] < elapsed_time - window_duration:
                    time_data.pop(0)
                    setpoint_data.pop(0)
                    actual_data.pop(0)
                    error_data.pop(0)
                    #pid_output_data.pop(0)

                # Ajusta limites do eixo Y
                plot.setYRange(min(min(actual_data), min(setpoint_data), min(error_data)) - 10,
                               max(max(actual_data), max(setpoint_data), max(error_data)) + 10)

                last_error = error  # Atualiza o erro anterior

            except ValueError:
                pass  # Ignora erros de conversão

# Configurando o timer para atualizar os gráficos em tempo real
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(1)  # Atualiza a cada 1 ms

# Executando a aplicação PyQtGraph
if __name__ == '__main__':
    try:
        app.exec_()
    except KeyboardInterrupt:
        print("Coleta de dados interrompida pelo usuário.")
    finally:
        ser.close()  # Fechar a porta serial corretamente
