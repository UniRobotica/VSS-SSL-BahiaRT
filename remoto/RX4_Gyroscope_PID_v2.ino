// ACSO-VSSS-F_PID

// Código de controle joystick via ESP-NOW do ACSO-VSSS-F com Giroscópio Acelerômetro e controle PID

// Bibliotecas
#include <PID_v1.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Atrinuição da biblioteca do MPU6050 à variável "mpu"
Adafruit_MPU6050 mpu;

// PWM mínimo e máximo para aplicação sem o PID
#define MIN_MOTOR_SPEED 50
#define MAX_MOTOR_SPEED 75

// Configuração do PWM
const int rightMotorPWMSpeedChannel = 0; // Canal interno do ESP32 para utilização da saída em PWM
const int leftMotorPWMSpeedChannel = 1; // Canal interno do ESP32 para utilização da saída em PWM
const int PWMFreq = 500; // Frequência dos pulsos
const int PWMResolution = 8; // Taxa de bits

// Motor Direito
const int enableRightMotor = 32;
const int rightMotorPin1 = 25;
const int rightMotorPin2 = 33;

// Motor Esquerdo
const int enableLeftMotor = 13;
const int leftMotorPin1 = 26;
const int leftMotorPin2 = 27;

// Variáveis
int Time; // Tempo em milisegundos para multiplicar pelo aceleração angular e obter a velocidade angular (quanto maior o tempo, maior o ganho no PID)
double Angle; // Aceleração em rad/s^2 = 0
double AngleError; // Diferença entre a aceleração em rad/s^2 medida e Angle
double AngleErrorM1; // Diferença entre a aceleração em rad/s^2 medida e Angle, multiplicado por Time
double AngleErrorM2; // Diferença entre a aceleração em rad/s^2 medida e Angle, multiplicado por Time
double Input1; // Entrada para o PID da roda direita
double Input2; // Entrada para o PID da roda esquerda
double Output1; // Saída  para o PID da roda direita
double Output2; // Saída  para o PID da roda esquerda
double Setpoint1; // Setpoint para o PID da roda direita
double Setpoint2; // Setpoint para o PID da roda esquerda

// Constantes do Controle PID
#define MIN_PWM 0
#define MAX_PWM 75
#define KP1 1.00
#define KP2 1.00
#define KI1 0.00
#define KI2 0.00
#define KD1 0.000
#define KD2 0.000

// Cria PID para controle
PID motorDireitoPID(&Input1, &Output1, &Setpoint1, KP1, KI1, KD1, DIRECT);
PID motorEsquerdoPID(&Input2, &Output2, &Setpoint2, KP2, KI2, KD2, DIRECT);

// Definição do tempo máximo de tolerância da ausência do sinal recebido para manter o robô funcionando
#define SIGNAL_TIMEOUT 1000  // Tempo escolhido em milisegundos
unsigned long lastRecvTime = 0;

// Estrutura de envio/recebimento de dados
typedef struct struct_message
{
  byte xAxisValue;  // Eixo X
  byte yAxisValue;  // Eixo Y
  byte switchPressed;  // Ativação/Desativação do PID
}struct_message;

struct_message rcv_commands;

// Função Callback para recebimento dos dados
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len)
{
  memcpy(&rcv_commands, incomingData, sizeof(rcv_commands));
  lastRecvTime = millis();
  rcv_commands.xAxisValue;
  rcv_commands.yAxisValue;
  rcv_commands.switchPressed;
  Serial.println();
}

void setup()
{
  // Setup da pinagem dos motores
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  // Setup do PWM para a velocidade dos motores
  ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);  
  ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
  ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel); 

  // Mantém os motores parados na inicialização
  ledcWrite(rightMotorPWMSpeedChannel, 0);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
  ledcWrite(leftMotorPWMSpeedChannel, 0);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);

  // Iniciação da Serial
  Serial.begin(115200);
  
  while (!Serial) // Irá pausar o código até que a Serial abra
    delay(10);

  Serial.println("Adafruit MPU6050 test!");
  
  if (!mpu.begin()) // Irá parar o código se o MPU6050 não estiver conectado
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");  // MPU6050 conectado com êxito

  // Setup Acelerômetro
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G); // Escolher aqui a escala desejada
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  // Setup do Giroscópio
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG); // Escolher aqui a escala desejada
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }
  // Setup da frequência
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // Escolher aqui a escala desejada
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
    
  // Configuração da comunicação wifi
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
  ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK( esp_wifi_start());
  ESP_ERROR_CHECK( esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

  // Inicia ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  Time = 9;
  Angle = 0.00;
  AngleError = 0.00;
  AngleErrorM1 = 0.00;
  AngleErrorM2 = 0.00;
  
  // Configura o controle PID  
  Input1 = 0.00;
  Input2 = 0.00;
  Output1 = 0.00;
  Output2 = 0.00;
  Setpoint1 = 75; // Alterar conforme a precisão desejada
  Setpoint2 = 75; // Alterar conforme a precisão desejada
  motorDireitoPID.SetOutputLimits(MIN_PWM, MAX_PWM);
  motorEsquerdoPID.SetOutputLimits(MIN_PWM, MAX_PWM);
  motorDireitoPID.SetMode(AUTOMATIC);
  motorEsquerdoPID.SetMode(AUTOMATIC);
}

void loop() 
{
  // Obtém novos eventos de sensor com as leituras
  sensors_event_t a, g, temp; // a = Acelerômetro; g = Giroscópio; temp = Termômetro
  mpu.getEvent(&a, &g, &temp);

  // Seta a zona morta do giroscópio e atribui o valor da aceleração angular à variável AngleError 
  AngleError = g.gyro.z - Angle;
  if(g.gyro.z < -0.01 && g.gyro.z > -0.04) // Giroscópio usado oscilando entre -0.02 e 0.03 rad/s^2
  {
    AngleError = 0.00;
  }

  // Sobre o PID:
  // Quanto maior o erro (rads/s^2) para um lado, menos força em PWM será aplicado para o motor do lado oposto
  
  if (rcv_commands.xAxisValue >= 175 && rcv_commands.yAxisValue >= 75 && rcv_commands.yAxisValue <= 175)
  { // Move o robô para frente
    if(rcv_commands.switchPressed == 1)
    { // Move o robô para frente com PID
      if(abs(AngleError) > 0.20 && AngleError > 0)  // 0.25 é rads/s^2 (aceleração angular mínima escolhida para usar PID), AngleErro > 0 é condição com o eixo Z de cabeça para baixo
      {
        AngleErrorM1 = 0.00;
        AngleErrorM2 = abs(AngleError)*Time;
        Input1 = AngleErrorM1;
        Input2 = AngleErrorM2;
        motorDireitoPID.Compute();
        motorEsquerdoPID.Compute();
      }
      else if(abs(AngleError) > 0.20 && AngleError < 0)  // 0.25 é rads/s^2 (aceleração angular mínima escolhida para usar PID), AngleErro < 0 é condição com o eixo Z de cabeça para baixo
      {
        AngleErrorM1 = abs(AngleError)*Time;
        AngleErrorM2 = 0.00;
        Input1 = AngleErrorM1;
        Input2 = AngleErrorM2;
        motorDireitoPID.Compute();
        motorEsquerdoPID.Compute();
      }
      ledcWrite(rightMotorPWMSpeedChannel, Output1);
      digitalWrite(rightMotorPin1, LOW);
      digitalWrite(rightMotorPin2, HIGH);
      ledcWrite(leftMotorPWMSpeedChannel, Output2);
      digitalWrite(leftMotorPin1, LOW);
      digitalWrite(leftMotorPin2, HIGH);
    }
    else
    { // Move o robô para frente sem PID
      ledcWrite(rightMotorPWMSpeedChannel, MAX_MOTOR_SPEED);
      digitalWrite(rightMotorPin1, LOW);
      digitalWrite(rightMotorPin2, HIGH);
      ledcWrite(leftMotorPWMSpeedChannel, MAX_MOTOR_SPEED);
      digitalWrite(leftMotorPin1, LOW);
      digitalWrite(leftMotorPin2, HIGH);
      
      AngleErrorM1 = 0.00;
      AngleErrorM2 = 0.00;
      Input1 = AngleErrorM1;
      Input2 = AngleErrorM2;
      motorDireitoPID.Compute();
      motorEsquerdoPID.Compute();
    }
  }
  else if (rcv_commands.xAxisValue > 175 && rcv_commands.yAxisValue > 175 && rcv_commands.yAxisValue <= 254)
  { // Move o robô para frente-direita
    ledcWrite(rightMotorPWMSpeedChannel, MIN_MOTOR_SPEED);
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
    ledcWrite(leftMotorPWMSpeedChannel, MAX_MOTOR_SPEED);
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);

    AngleErrorM1 = 0.00;
    AngleErrorM2 = 0.00;
    Input1 = AngleErrorM1;
    Input2 = AngleErrorM2;
    motorDireitoPID.Compute();
    motorEsquerdoPID.Compute();
  }
  else if (rcv_commands.xAxisValue >= 175 && rcv_commands.yAxisValue >= 0 && rcv_commands.yAxisValue < 75)
  { // Move o robô para frente-esquerda
    ledcWrite(rightMotorPWMSpeedChannel, MAX_MOTOR_SPEED);
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
    ledcWrite(leftMotorPWMSpeedChannel, MIN_MOTOR_SPEED);
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);

    AngleErrorM1 = 0.00;
    AngleErrorM2 = 0.00;
    Input1 = AngleErrorM1;
    Input2 = AngleErrorM2;
    motorDireitoPID.Compute();
    motorEsquerdoPID.Compute();
  }
  else if (rcv_commands.xAxisValue == 0 && rcv_commands.yAxisValue >= 75 && rcv_commands.yAxisValue <= 175)
  { // Move o robô para trás
    if(rcv_commands.switchPressed == 1)
    { // Move o robô para trás com PID
      if(abs(AngleError) > 0.20 && AngleError > 0)  // 0.25 é rads/s^2 (aceleração angular mínima escolhida para usar PID), AngleErro > 0 é condição com o eixo Z de cabeça para baixo
      {
        AngleErrorM1 = abs(AngleError)*Time;
        AngleErrorM2 = 0.00;
        Input1 = AngleErrorM1;
        Input2 = AngleErrorM2;
        motorDireitoPID.Compute();
        motorEsquerdoPID.Compute();
      }
      else if(abs(AngleError) > 0.20 && AngleError < 0)  // 0.25 é rads/s^2 (aceleração angular mínima escolhida para usar PID), AngleErro < 0 é condição com o eixo Z de cabeça para baixo
      {
        AngleErrorM1 = 0.00;
        AngleErrorM2 = abs(AngleError)*Time;
        Input1 = AngleErrorM1;
        Input2 = AngleErrorM2;
        motorDireitoPID.Compute();
        motorEsquerdoPID.Compute();
      }
      ledcWrite(rightMotorPWMSpeedChannel, Output1);
      digitalWrite(rightMotorPin1, HIGH);
      digitalWrite(rightMotorPin2, LOW);
      ledcWrite(leftMotorPWMSpeedChannel, Output2);
      digitalWrite(leftMotorPin1, HIGH);
      digitalWrite(leftMotorPin2, LOW);
    }
    else
    { // Move o robô para trás sem PID
      ledcWrite(rightMotorPWMSpeedChannel, MAX_MOTOR_SPEED);
      digitalWrite(rightMotorPin1, HIGH);
      digitalWrite(rightMotorPin2, LOW);
      ledcWrite(leftMotorPWMSpeedChannel, MAX_MOTOR_SPEED);
      digitalWrite(leftMotorPin1, HIGH);
      digitalWrite(leftMotorPin2, LOW);

      AngleErrorM1 = 0.00;
      AngleErrorM2 = 0.00;
      Input1 = AngleErrorM1;
      Input2 = AngleErrorM2;
      motorDireitoPID.Compute();
      motorEsquerdoPID.Compute();
    }
  }
  else if (rcv_commands.xAxisValue == 0 && rcv_commands.yAxisValue > 175 && rcv_commands.yAxisValue <= 254)
  { // Move o robô para trás-direita
    ledcWrite(rightMotorPWMSpeedChannel, MIN_MOTOR_SPEED);
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
    ledcWrite(leftMotorPWMSpeedChannel, MAX_MOTOR_SPEED);
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);

    AngleErrorM1 = 0.00;
    AngleErrorM2 = 0.00;
    Input1 = AngleErrorM1;
    Input2 = AngleErrorM2;
    motorDireitoPID.Compute();
    motorEsquerdoPID.Compute();
  }
  else if (rcv_commands.xAxisValue == 0 && rcv_commands.yAxisValue >= 0 && rcv_commands.yAxisValue < 75)
  { // Move o robô para trás-esquerda
    ledcWrite(rightMotorPWMSpeedChannel, MAX_MOTOR_SPEED);
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
    ledcWrite(leftMotorPWMSpeedChannel, MIN_MOTOR_SPEED);
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);

    AngleErrorM1 = 0.00;
    AngleErrorM2 = 0.00;
    Input1 = AngleErrorM1;
    Input2 = AngleErrorM2;
    motorDireitoPID.Compute();
    motorEsquerdoPID.Compute();
  }
  else if (rcv_commands.xAxisValue > 0 && rcv_commands.xAxisValue < 254 && rcv_commands.yAxisValue == 254)
  { // Move o robô para direita
    ledcWrite(rightMotorPWMSpeedChannel, MIN_MOTOR_SPEED);
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
    ledcWrite(leftMotorPWMSpeedChannel, MIN_MOTOR_SPEED);
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);

    AngleErrorM1 = 0.00;
    AngleErrorM2 = 0.00;
    Input1 = AngleErrorM1;
    Input2 = AngleErrorM2;
    motorDireitoPID.Compute();
    motorEsquerdoPID.Compute();
  }
  else if (rcv_commands.xAxisValue > 0 && rcv_commands.xAxisValue < 254 && rcv_commands.yAxisValue == 0)
  {  // Move o robô para esquerda
    ledcWrite(rightMotorPWMSpeedChannel, MIN_MOTOR_SPEED);
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
    ledcWrite(leftMotorPWMSpeedChannel, MIN_MOTOR_SPEED);
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);

    AngleErrorM1 = 0.00;
    AngleErrorM2 = 0.00;
    Input1 = AngleErrorM1;
    Input2 = AngleErrorM2;
    motorDireitoPID.Compute();
    motorEsquerdoPID.Compute();
  }
  else                                                                                                     
  { // Para o robô
    ledcWrite(rightMotorPWMSpeedChannel, 0);
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
    ledcWrite(leftMotorPWMSpeedChannel, 0);
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
    
    AngleErrorM1 = 0.00;
    AngleErrorM2 = 0.00;
    Input1 = AngleErrorM1;
    Input2 = AngleErrorM2;
    motorDireitoPID.Compute();
    motorEsquerdoPID.Compute();
  }
  
  // Exibe valores no Monitor Serial
  Serial.print("PID: ");
  Serial.print(rcv_commands.switchPressed);
  Serial.print(" | ");
  Serial.print("Eixo X: ");
  Serial.print(rcv_commands.xAxisValue);
  Serial.print(" | ");
  Serial.print("Eixo Y: ");
  Serial.print(rcv_commands.yAxisValue);
  Serial.print(" | ");
  Serial.print("Z: ");
  Serial.print(g.gyro.z);
  Serial.print(" rads/s² | ");
  Serial.print("AngleError: ");
  Serial.print(AngleError);
  Serial.print(" | ");
  Serial.print("PWM2: ");
  Serial.print(Output2);
  Serial.print(" | ");
  Serial.print("AngleErrorM2: ");
  Serial.print(AngleErrorM2);
  Serial.print(" | ");
  Serial.print("PWM1: ");
  Serial.print(Output1);
  Serial.print(" | ");
  Serial.print("AngleErrorM1: ");
  Serial.print(AngleErrorM1);    
  Serial.println();
  
  //Serial.println(g.gyro.z);
  //Serial.println(" rads/s²");

  AngleErrorM1 = 0.00;
  AngleErrorM2 = 0.00;

  // Para o robô se o sinal recebido cair
  unsigned long now = millis();
  if (now - lastRecvTime > SIGNAL_TIMEOUT) 
  {
    ledcWrite(rightMotorPWMSpeedChannel, 0);
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
    ledcWrite(leftMotorPWMSpeedChannel, 0);
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }
}

