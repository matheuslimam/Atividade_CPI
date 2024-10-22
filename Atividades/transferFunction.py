import matplotlib.pyplot as plt
import numpy as np

# Parâmetros do sistema
setpoint_motor1 = 10.0  # Setpoint para o motor 1
setpoint_motor2 = 8.0   # Setpoint para o motor 2
Kp = 2.0  # Ganho proporcional
Ki = 0.5  # Ganho integral
Kd = 0.1  # Ganho derivativo
tempo_total = 20  # Simulação em segundos
delta_t = 0.1  # Intervalo de tempo em segundos

# Função de transferência do motor (simplificada)
def motor_response(u):
    return 0.8 * u  # Resposta linear simplificada para fins didáticos

# PID Controller
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.previous_error = 0

    def compute(self, setpoint, measurement, delta_t):
        error = setpoint - measurement
        self.integral += error * delta_t
        derivative = (error - self.previous_error) / delta_t
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

# Inicializar os controladores PID para os dois motores
pid_motor1 = PIDController(Kp, Ki, Kd)
pid_motor2 = PIDController(Kp, Ki, Kd)

# Listas para armazenar os dados para plotagem
time_data = np.arange(0, tempo_total, delta_t)

# Dados do motor 1
setpoint_data_motor1 = [setpoint_motor1] * len(time_data)
pid_output_data_motor1 = []
motor_output_data_motor1 = []

# Dados do motor 2
setpoint_data_motor2 = [setpoint_motor2] * len(time_data)
pid_output_data_motor2 = []
motor_output_data_motor2 = []

# Inicializa os valores dos motores
motor_output1 = 0.0
motor_output2 = 0.0

# Simulação ao longo do tempo para os dois motores
for t in time_data:
    # Computa a saída do PID para o motor 1
    pid_output_motor1 = pid_motor1.compute(setpoint_motor1, motor_output1, delta_t)
    motor_output1 = motor_response(pid_output_motor1)

    # Computa a saída do PID para o motor 2
    pid_output_motor2 = pid_motor2.compute(setpoint_motor2, motor_output2, delta_t)
    motor_output2 = motor_response(pid_output_motor2)

    # Armazena os dados para o motor 1
    pid_output_data_motor1.append(pid_output_motor1)
    motor_output_data_motor1.append(motor_output1)

    # Armazena os dados para o motor 2
    pid_output_data_motor2.append(pid_output_motor2)
    motor_output_data_motor2.append(motor_output2)

# Plotar os resultados
plt.figure(figsize=(12, 10))

# Plot do setpoint e da saída do motor 1
plt.subplot(3, 1, 1)
plt.plot(time_data, setpoint_data_motor1, label='Setpoint Motor 1', linestyle='--', color='r')
plt.plot(time_data, motor_output_data_motor1, label='Motor 1 Output', color='b')
plt.title('Motor 1 Output vs Setpoint')
plt.xlabel('Time (s)')
plt.ylabel('Output')
plt.legend()
plt.grid(True)

# Plot do setpoint e da saída do motor 2
plt.subplot(3, 1, 2)
plt.plot(time_data, setpoint_data_motor2, label='Setpoint Motor 2', linestyle='--', color='r')
plt.plot(time_data, motor_output_data_motor2, label='Motor 2 Output', color='b')
plt.title('Motor 2 Output vs Setpoint')
plt.xlabel('Time (s)')
plt.ylabel('Output')
plt.legend()
plt.grid(True)

# Plot da saída do PID para os dois motores
plt.subplot(3, 1, 3)
plt.plot(time_data, pid_output_data_motor1, label='PID Output Motor 1', color='g')
plt.plot(time_data, pid_output_data_motor2, label='PID Output Motor 2', color='purple')
plt.title('PID Controller Output for Motor 1 and Motor 2')
plt.xlabel('Time (s)')
plt.ylabel('PID Output')
plt.legend()
plt.grid(True)

# Mostrar os gráficos
plt.tight_layout()
plt.show()
