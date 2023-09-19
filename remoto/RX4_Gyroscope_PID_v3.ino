
// ACSO-VSSS-F_PID

// Código de controle joystick via ESP-NOW do ACSO-VSSS-F com Giroscópio Acelerômetro, controle PID e Supervisório

// Bibliotecas
#include <PID_v1.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Atribuição da biblioteca do MPU6050 ao objeto "mpu"
Adafruit_MPU6050 mpu;

// PWM mínimo e máximo para aplicação sem o PID
#define MIN_MOTOR_SPEED 55
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
unsigned long Timeold; // Parâmetro de tempo para cálcular envio de dados no Monitor Serial (default = 1000ms)
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
#define MAX_PWM 100
#define KP1 1.00
#define KP2 1.00
#define KI1 0.00
#define KI2 0.00
#define KD1 0.000
#define KD2 0.000

// Cria PID para controle
PID rightMotorPID(&Input1, &Output1, &Setpoint1, KP1, KI1, KD1, DIRECT);
PID leftMotorPID(&Input2, &Output2, &Setpoint2, KP2, KI2, KD2, DIRECT);

// Definição do tempo máximo de tolerância da ausência do sinal recebido para manter o robô funcionando
#define SIGNAL_TIMEOUT 1000  // Tempo escolhido em milisegundos
unsigned long lastRecvTime = 0;

// MAC Adress para enviar os dados no canal selecionado
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//uint8_t broadcastAddress[] = {0x70, 0xB8, 0xF6, 0x5D, 0x76, 0x64};  // MAC Adress do MR 70:B8:F6:5D:76:64

// Estrutura de envio/recebimento de dados
typedef struct struct_message
{
  byte xAxisValue; // Eixo X
  byte yAxisValue; // Eixo Y
  byte switchPressed; // Ativação/Desativação do PID
  double AngleErrorSup; // Atribuição do erro de angulação que foi resultante
  double Output2Sup; // PWM do motor esquerdo
  double AngleErrorM2Sup; //  Atribuição do erro de angulação que foi atribuído motor esquerdo
  double Output1Sup; // PWM do motor direito
  double AngleErrorM1Sup; // Atribuição do erro de angulação que foi atribuído motor direito

}struct_message;

struct_message myData;

struct_message incomingReadings;

esp_now_peer_info_t peerInfo;

// Função Callback para envio dos dados
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  //Serial.print("\r\nLast Packet Send Status:\t ");
  //Serial.println(status);
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
}
// Função Callback para recebimento dos dados
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
  incomingReadings.xAxisValue;
  incomingReadings.yAxisValue;
  incomingReadings.switchPressed;
  //Serial.println(incomingReadings.xAxisValue);
  //Serial.println(incomingReadings.yAxisValue);  
  //Serial.println(incomingReadings.switchPressed);
  
  lastRecvTime = millis();
}

void setup()
{
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
  esp_wifi_set_max_tx_power(84);

  // Inicia ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else
  {
    //Serial.println("Succes: Initialized ESP-NOW");
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  else
  {
    //Serial.println("Succes: Added peer");
  }

  // Função de Callback para envio de dados
  esp_now_register_send_cb(OnDataSent);

  // Função de Callback para recebimento de dados
  esp_now_register_recv_cb(OnDataRecv);

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

  Timeold = 0;
  Time = 5;
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
  rightMotorPID.SetOutputLimits(MIN_PWM, MAX_PWM);
  leftMotorPID.SetOutputLimits(MIN_PWM, MAX_PWM);
  rightMotorPID.SetMode(AUTOMATIC);
  leftMotorPID.SetMode(AUTOMATIC);
}

void loop() 
{
  // Obtém novos eventos de sensor com as leituras
  sensors_event_t a, g, temp; // a = Acelerômetro; g = Giroscópio; temp = Termômetro
  mpu.getEvent(&a, &g, &temp);

  // Seta a zona morta do giroscópio e atribui o valor da aceleração angular à variável AngleError 
  if(g.gyro.z < -0.01 && g.gyro.z > -0.04) // Giroscópio usado oscilando entre -0.02 e -0.03 rad/s^2
  {
    AngleError = 0.00;
  }
  else
  {
    AngleError = g.gyro.z - Angle;
  }

  // Sobre o PID:
  // Quanto maior o erro (rads/s^2) para um lado, mais força em PWM será aplicado no motor do mesmo lado, e menos para o motor do lado oposto
  
  if (incomingReadings.xAxisValue >= 175 && incomingReadings.yAxisValue >= 75 && incomingReadings.yAxisValue <= 175)
  { // Move o robô para frente
    if(incomingReadings.switchPressed == 1)
    { // Move o robô para frente com PID
      if(abs(AngleError) > 0.05 && AngleError > 0) // 0.05 é rads/s^2 (aceleração angular mínima escolhida para usar PID), AngleError > 0 é condição com o eixo Z de cabeça para baixo
      {
        AngleErrorM1 = -AngleError*Time;
        AngleErrorM2 = AngleError*Time;
        Input1 = AngleErrorM1;
        Input2 = AngleErrorM2;
        rightMotorPID.Compute();
        leftMotorPID.Compute();
      }
      else if(abs(AngleError) > 0.05 && AngleError < 0) // 0.05 é rads/s^2 (aceleração angular mínima escolhida para usar PID), AngleError < 0 é condição com o eixo Z de cabeça para baixo
      {
        AngleErrorM1 = abs(AngleError*Time);
        AngleErrorM2 = AngleError*Time;
        Input1 = AngleErrorM1;
        Input2 = AngleErrorM2;
        rightMotorPID.Compute();
        leftMotorPID.Compute();
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
      rightMotorPID.Compute();
      leftMotorPID.Compute();
    }
  }
  else if (incomingReadings.xAxisValue > 175 && incomingReadings.yAxisValue > 175 && incomingReadings.yAxisValue <= 254)
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
    rightMotorPID.Compute();
    leftMotorPID.Compute();
  }
  else if (incomingReadings.xAxisValue >= 175 && incomingReadings.yAxisValue >= 0 && incomingReadings.yAxisValue < 75)
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
    rightMotorPID.Compute();
    leftMotorPID.Compute();
  }
  else if (incomingReadings.xAxisValue == 0 && incomingReadings.yAxisValue >= 75 && incomingReadings.yAxisValue <= 175)
  { // Move o robô para trás
    if(incomingReadings.switchPressed == 1)
    { // Move o robô para trás com PID
      if(abs(AngleError) > 0.05 && AngleError > 0) // 0.05 é rads/s^2 (aceleração angular mínima escolhida para usar PID), AngleError > 0 é condição com o eixo Z de cabeça para baixo
      {
        AngleErrorM1 = AngleError*Time;
        AngleErrorM2 = -AngleError*Time;
        Input1 = AngleErrorM1;
        Input2 = AngleErrorM2;
        rightMotorPID.Compute();
        leftMotorPID.Compute();
      }
      else if(abs(AngleError) > 0.05 && AngleError < 0) // 0.05 é rads/s^2 (aceleração angular mínima escolhida para usar PID), AngleError < 0 é condição com o eixo Z de cabeça para baixo
      {
        AngleErrorM1 = -AngleError*Time;
        AngleErrorM2 = abs(AngleError)*Time;
        Input1 = AngleErrorM1;
        Input2 = AngleErrorM2;
        rightMotorPID.Compute();
        leftMotorPID.Compute();
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
      rightMotorPID.Compute();
      leftMotorPID.Compute();
    }
  }
  else if (incomingReadings.xAxisValue == 0 && incomingReadings.yAxisValue > 175 && incomingReadings.yAxisValue <= 254)
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
    rightMotorPID.Compute();
    leftMotorPID.Compute();
  }
  else if (incomingReadings.xAxisValue == 0 && incomingReadings.yAxisValue >= 0 && incomingReadings.yAxisValue < 75)
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
    rightMotorPID.Compute();
    leftMotorPID.Compute();
  }
  else if (incomingReadings.xAxisValue > 0 && incomingReadings.xAxisValue < 254 && incomingReadings.yAxisValue == 254)
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
    rightMotorPID.Compute();
    leftMotorPID.Compute();
  }
  else if (incomingReadings.xAxisValue > 0 && incomingReadings.xAxisValue < 254 && incomingReadings.yAxisValue == 0)
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
    rightMotorPID.Compute();
    leftMotorPID.Compute();
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
    rightMotorPID.Compute();
    leftMotorPID.Compute();
  }

  // Dados para envio
  myData.AngleErrorSup = AngleError; // Atribuição do erro de angulação que foi resultante
  myData.Output2Sup = Output2; // Atribuição do PWM do motor esquerdo
  myData.AngleErrorM2Sup = AngleErrorM2; // Atribuição do erro de angulação que foi atribuído motor esquerdo
  myData.Output1Sup = Output1; // Atribuição do PWM do motor direito
  myData.AngleErrorM1Sup = AngleErrorM1; // Atribuição do erro de angulação que foi atribuído motor direito

  // Declaração
  esp_err_t result;

  // Envia os dados via ESP-NOW  
  if (millis() - Timeold >= 50)
  {
    result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    if (result == ESP_OK)
    {
    //Serial.println("Sent with success");
    }
    else
    {
    Serial.println("Error sending the data");
    }
    
    // Exibe valores no Monitor Serial
    Serial.print("PID: ");
    Serial.print(incomingReadings.switchPressed);
    Serial.print(" | ");
    Serial.print("Eixo X: ");
    Serial.print(incomingReadings.xAxisValue);
    Serial.print(" | ");
    Serial.print("Eixo Y: ");
    Serial.print(incomingReadings.yAxisValue);
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
    Serial.println(AngleErrorM1);
    
    //Serial.print("AngleError: ");
    //Serial.println(AngleError);
  }

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
