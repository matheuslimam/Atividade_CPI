const int INPUT_PIN = A0;  // Pino de entrada analógico (sensor)
const int OUTPUT_PIN = 3;  // Pino de saída PWM (controlador)
const int POT_PIN = A1;    // Pino onde o potenciômetro está conectado

double dt, last_time;
double integral = 0, previous_error = 0, previous_filtered_derivative = 0, output = 0;
double kp, ki, kd;
double setpoint = 200;  // Setpoint inicial

unsigned long startTime;   // Para armazenar o tempo inicial
unsigned long currentTime; // Para calcular o tempo atual

bool use_derivative_filter = true;  // Habilitar ou desabilitar o filtro derivativo
bool use_anti_windup = true;  // Habilitar ou desabilitar o anti-windup
bool use_reference_weighting = true;  // Habilitar ou desabilitar a ponderação de setpoint

double Kbc;  // Ganho de retrocálculo anti-windup
double beta;  // Fator de ponderação proporcional
double alpha; // Coeficiente de filtro derivativo
double K = 1.0;  // Ganho estático do sistema (ajustar conforme necessário)
double tau = 0.5;  // Constante de tempo do sistema (ajustar conforme necessário)

void setup()
{
  // Aplicando a sintonia de Skogestad
  kp = 1.0 / (K * (tau + 0));  // Skogestad sugere o uso do atraso de transporte theta, que aqui é considerado 0
  ki = kp / tau;
  kd = kp * tau / 2;

  last_time = 0;
  Kbc = ki; // Ganho de retrocálculo anti-windup
  beta = 0.5;  // Fator de ponderação proporcional
  alpha = 0.02;  // Coeficiente de filtro derivativo

  Serial.begin(115200);
  analogWrite(OUTPUT_PIN, 0);  // Inicializa a saída como 0
  startTime = millis();  // Armazena o tempo inicial

  // Loop inicial para imprimir zeros na fase de configuração do sistema
  for (int i = 0; i < 50; i++)
  {
    currentTime = (millis() - startTime) / 1000.0;  // Tempo decorrido em segundos
    Serial.print(currentTime);  // Imprime o tempo decorrido
    Serial.print(",");
    Serial.println(0);  // Imprime a saída inicial (0) para estabelecer uma linha de base
    delay(100);
  }
  delay(100);  // Pequeno atraso para permitir que o sistema estabilize
}

void loop()
{
  // Lê o valor do potenciômetro (entre 0 e 1023) e ajusta o setpoint dinamicamente
  // setpoint = map(analogRead(POT_PIN), 0, 1023, 0, 255);  // Ajusta o setpoint entre 0 e 255

  double now = millis();
  dt = (now - last_time) / 1000.00;  // Calcula o intervalo de tempo (dt)
  last_time = now;

  // Lê o valor do sensor (de 0 a 1023) e mapeia para a faixa de 0 a 255
  double actual = map(analogRead(INPUT_PIN), 0, 1023, 0, 255);
  double error = setpoint - actual;

  // Calcula a saída PID com ponderação de setpoint
  output = pid_skogestad(error, actual, setpoint);  // Passa o valor atual e o setpoint para a função PID

  analogWrite(OUTPUT_PIN, output);  // Aplica a saída calculada

  // Envia o tempo decorrido, setpoint, valor atual, erro e saída para o Plotter Serial
  currentTime = millis() - startTime;  // Calcula o tempo decorrido desde o início
  Serial.print(currentTime / 1000.0);  // Imprime o tempo decorrido em segundos
  Serial.print(",");                   // Separador de vírgula
  Serial.print(setpoint);              // Imprime o valor do setpoint
  Serial.print(",");
  Serial.print(actual);                // Imprime o valor atual do sensor
  Serial.print(",");
  Serial.print(error);                 // Imprime o erro
  Serial.print(",");
  Serial.println(output);              // Imprime o valor de saída PID

  delay(12);  // Insere um atraso no circuito (opcional)
}

double pid_skogestad(double error, double actual, double setpoint)
{
  // Termo proporcional com ponderação de setpoint
  double proportional;
  if (use_reference_weighting) {
    proportional = beta * (setpoint - actual);  // Influência ponderada do setpoint
  } else {
    proportional = error;  // Proporcional padrão sem ponderação de setpoint
  }

  // Termo integral
  integral += error * dt;

  // Termo derivativo com filtro
  double derivative;
  if (use_derivative_filter) {
    // Aplica o filtro derivativo
    double raw_derivative = (error - previous_error) / dt;
    derivative = alpha * raw_derivative + (1 - alpha) * previous_filtered_derivative;
    previous_filtered_derivative = derivative;
  } else {
    // Sem filtro, usa derivada bruta
    derivative = (error - previous_error) / dt;
  }
  
  previous_error = error;

  // Calcula a saída PID com ou sem derivada filtrada
  double output = kp * (proportional + integral * ki + derivative * kd);

  // Anti-windup para limitar a integral se necessário
  if (use_anti_windup) {
    double max_output = 255;
    double min_output = 0;
    if (output > max_output) {
      output = max_output;
      integral -= Kbc * (output - max_output) * dt;  // Corrige a integral se o sistema saturar
    } else if (output < min_output) {
      output = min_output;
      integral -= Kbc * (output - min_output) * dt;  // Corrige a integral se o sistema saturar
    }
  }

  return output;
}
