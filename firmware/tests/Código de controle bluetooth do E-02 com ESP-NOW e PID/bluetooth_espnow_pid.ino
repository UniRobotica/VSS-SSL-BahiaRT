// E-02_PID

// Código de controle via bluetooth do E-02 com encoders e controle PID

// Bibliotecas
#include <PID_v1.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "BluetoothSerial.h"

// Atribuição da conexão bluetooth (SerialBT)
BluetoothSerial SerialBT;

// MAC Adress de envio dos dados em rede
uint8_t broadcast_adr0[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//  //  //  //  //  //  //  //
typedef struct struct_message{
  float v_l;
  float v_a;
} 
struct_message;

struct_message commands;

//  //  //  //  //  //  //  //
esp_now_peer_info_t peerInfo;

// CONFIGURAÇÃO DOS MOTORES
// Motor direito
int Motor1PWM = 15;
int Motor1A = 2;
int Motor1B = 4;

// Motor esquerdo
int Motor2PWM = 13;
int Motor2A = 12;
int Motor2B = 14;

// Variáveis
char Recebido; // Armazena dados recebidos via bluetooth

volatile byte Pulsos1; // Pulsos do encoder direito
volatile byte Pulsos2; // Pulsos do encoder esquerdo
unsigned long Timeold1; // Parâmetro de tempo para cálculos no encoder direito (default = 1000ms)
unsigned long Timeold2; // Parâmetro de tempo para cálculos no encoder esquerdo (default = 1000ms)
int PinoSensor1 = 5; // Pino do microcontrolador ligado ao pino OUT do sensor direito
int PinoSensor2 = 27; // Pino do microcontrolador ligado ao pino OUT do sensor esquerdo
unsigned int PulsosDisco1 = 20; // Disco encoder direito (Alterar o valor conforme fendas disco encoder)
unsigned int PulsosDisco2 = 20; // Disco encoder esquerdo (Alterar o valor conforme fendas disco encoder)
int PWM1 = 125;
int PWM2 = 175;
int PWM3 = 225;
double RPM1; // RPM do encoder direito
double RPM2; // RPM do encoder esquerdo
double Input1; // Entrada para o PID da roda direita
double Input2; // Entrada para o PID da roda esquerda
double Output1; // Saída  para o PID da roda direita
double Output2; // Saída  para o PID da roda esquerda
double Setpoint1; // Setpoint para o PID da roda direita
double Setpoint2; // Setpoint para o PID da roda esquerda

// Constantes do Controle PID
#define MIN_PWM 255
#define MAX_PWM 255
#define KP1 0.25
#define KP2 0.25
#define KI1 0.21
#define KI2 0.21
#define KD1 0.025
#define KD2 0.025

// Cria PID para controle
PID motorDireitoPID(&Input1, &Output1, &Setpoint1, KP1, KI1, KD1, DIRECT);
PID motorEsquerdoPID(&Input2, &Output2, &Setpoint2, KP2, KI2, KD2, DIRECT);

// Setup
void setup() {
  // Inicia Serial bluetooth e ESP-NOW
  Serial.begin(115200);  // Velocidade da conexão
  SerialBT.begin("E-02 BLUETOOTH"); // Nome da rede bluetooth
  Serial.println("Dispositivo conectado");  // Confirmação da serial bluetooth ativa

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_channel(13, WIFI_SECOND_CHAN_NONE));
  esp_wifi_set_max_tx_power(84);

  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  memcpy(peerInfo.peer_addr, broadcast_adr0, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) 
  {
    Serial.println("Failed to add peer");
    return;
  }

  // Configuração dos pinos do motor
  pinMode(Motor1PWM, OUTPUT);
  pinMode(Motor1A, OUTPUT);
  pinMode(Motor1B, OUTPUT);
  pinMode(Motor2PWM, OUTPUT);
  pinMode(Motor2A, OUTPUT);
  pinMode(Motor2B, OUTPUT);
 
  // Configura Interrupção
  pinMode(PinoSensor1, INPUT);
  pinMode(PinoSensor2, INPUT);
  attachInterrupt(PinoSensor1, Contador1, RISING);
  attachInterrupt(PinoSensor2, Contador2, RISING);
  Pulsos1 = 0;
  Pulsos2 = 0;
  RPM1 = 0;
  RPM2 = 0;
  Timeold1 = 0;
  Timeold2 = 0;

  // Configura controle PID
  Input1 = RPM1;
  Input2 = RPM2;
  Setpoint1 = 150;  // Alterar conforme a velocidade desejada
  Setpoint2 = 150;  // Alterar conforme a velocidade desejada
  motorDireitoPID.SetOutputLimits(MIN_PWM, MAX_PWM);
  motorEsquerdoPID.SetOutputLimits(MIN_PWM, MAX_PWM);
  motorDireitoPID.SetMode(AUTOMATIC);
  motorEsquerdoPID.SetMode(AUTOMATIC);
}

// Função executada a cada interrupção
void Contador1()
{
  Pulsos1++;  // Incrementa contador
}
// Função executada a cada interrupção
void Contador2()
{
  Pulsos2++;  // Incrementa contador
}

void loop() {
  // Calcula RPM a cada 1 Segundo
  if (millis() - Timeold1 >= 1000) {
    detachInterrupt(PinoSensor1); // Desabilita interrupção durante o cálculo para evitar sair do IF
    detachInterrupt(PinoSensor2); // Desabilita interrupção durante o cálculo para evitar sair do IF
    RPM1 = (60 * 1000 / PulsosDisco1) / (millis() - Timeold1) * Pulsos1;
    Timeold1 = millis();
    Pulsos1 = 0;
    RPM2 = (60 * 1000 / PulsosDisco2) / (millis() - Timeold2) * Pulsos2;
    Timeold2 = millis();
    Pulsos2 = 0;
    
    // Exibe valores no serial monitor
    Serial.print("PWM1: ");
    Serial.print(Output1);
    Serial.print("    ");
    Serial.print("RPM1: ");
    Serial.print(Input1);
    Serial.print("    ");
    Serial.print("PWM2: ");
    Serial.print(Output2);
    Serial.print("    ");
    Serial.print("RPM2: ");
    Serial.println(Input2);

    commands.v_l = 19;
    commands.v_a = 10;
    sendData();

    // Habilita novamente a interrupção
    attachInterrupt(PinoSensor1, Contador1, RISING);
    attachInterrupt(PinoSensor2, Contador2, RISING);
  }
    
  // Calcula o PWM para os motores conforme o ontrole PID
  Input1 = RPM1;
  Input2 = RPM2;
  motorDireitoPID.Compute();
  motorEsquerdoPID.Compute();

  // Comunicação serial do bluetooth
  if (Serial.available()) { // Se a serial estiver ativa
    SerialBT.write(Serial.read()); // Envia dados do monitor serial via bluetooth
  }
  if (SerialBT.available()) { // Se o bluetooth estiver ativo 
    Recebido = SerialBT.read(); // Lê dados recebidos via bluetooth e atribui à variável "Recebido"
    SerialBT.write(Recebido); // Escreve os dados recebidos via bluetooth
    
    // Caracteres recebidos via bluetooth
    if (Recebido == 'F') {
      frente();
    }
    else if (Recebido == 'I') {
      frente_direita();
    }
    else if (Recebido == 'G') {
    frente_esquerda();
    }    
    else if (Recebido == 'B') {
    tras();
    }
    else if (Recebido == 'J') {
    tras_direita();
    }
    else if (Recebido == 'H') {
    tras_esquerda();
    }
    else if (Recebido == 'L') {
    esquerda();
    } 
    else if (Recebido == 'R') {
    direita();
    }
    else if (Recebido == 'S') {
    parado();
    }
    else {
    parado();
    }
  }
}
  
// COMANDOS DIRECIONAIS DO ROBÔ 
void frente() {
  // Aciona os motores
  analogWrite(Motor1PWM, Output1);
  digitalWrite(Motor1A, HIGH);
  digitalWrite(Motor1B, LOW);
  analogWrite(Motor2PWM, Output2);
  digitalWrite(Motor2A, LOW);
  digitalWrite(Motor2B, HIGH);
}
  
void frente_direita() {
  // Aciona os motores
  analogWrite(Motor1PWM, PWM2);
  digitalWrite(Motor1A, HIGH);
  digitalWrite(Motor1B, LOW);
  analogWrite(Motor2PWM, PWM3);
  digitalWrite(Motor2A, LOW);
  digitalWrite(Motor2B, HIGH);
}
  
void frente_esquerda() {
  // Aciona os motores
  analogWrite(Motor1PWM, PWM3);
  digitalWrite(Motor1A, HIGH);
  digitalWrite(Motor1B, LOW);
  analogWrite(Motor2PWM, PWM2);
  digitalWrite(Motor2A, LOW);
  digitalWrite(Motor2B, HIGH);
}
 
void tras() {
  // Aciona os motores
  analogWrite(Motor1PWM, Output1);
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor1B, HIGH);
  analogWrite(Motor2PWM, Output2);
  digitalWrite(Motor2A, HIGH);
  digitalWrite(Motor2B, LOW);
}

void tras_direita() {
  // Aciona os motores
  analogWrite(Motor1PWM, PWM2);
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor1B, HIGH);
  analogWrite(Motor2PWM, PWM3);
  digitalWrite(Motor2A, HIGH);
  digitalWrite(Motor2B, LOW);
}
 
void tras_esquerda() {
  // Aciona os motores
  analogWrite(Motor1PWM, PWM3);
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor1B, HIGH);
  analogWrite(Motor2PWM, PWM2);
  digitalWrite(Motor2A, HIGH);
  digitalWrite(Motor2B, LOW);
}
 
void esquerda() {
  // Aciona os motores
  analogWrite(Motor1PWM, PWM3);
  digitalWrite(Motor1A, HIGH);
  digitalWrite(Motor1B, LOW);
  analogWrite(Motor2PWM, PWM3);
  digitalWrite(Motor2A, HIGH);
  digitalWrite(Motor2B, LOW);
}
 
void direita() {
  // Aciona os motores
  analogWrite(Motor1PWM, PWM3);
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor1B, HIGH);
  analogWrite(Motor2PWM, PWM3);
  digitalWrite(Motor2A, LOW);
  digitalWrite(Motor2B, HIGH);
}

void parado() {
  // Para os motores
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor1B, LOW);
  digitalWrite(Motor2A, LOW);
  digitalWrite(Motor2B, LOW);
}

void sendData(){
  // Delay é necessário para que os dados sejam enviados corretamente
  esp_err_t message = esp_now_send(broadcast_adr0, (uint8_t *) &commands, sizeof(commands));
  delay(3);
}

