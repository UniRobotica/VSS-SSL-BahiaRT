// Código para o recebimento de informações e controle do robô

#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define MAX_PWM 255

#define CANAL_ESPNOW 1

// Motor Direito
#define PMW_CANAL_R 1
#define PWMA 15
#define A1 4
#define B1 2

// Motor Esquerdo
#define PMW_CANAL_L 2
#define PWMB 13
#define A2 12 
#define B2 14

// Definindo variáveis ------------------------------------------

int robot_id = 0; // ATENÇÃO: Mudar o id de acordo com o robô

// Para informações recebidas

int id;
int first_mark = 0, second_mark;

float v_l, v_a;

const byte numChars = 64;
char commands[numChars];
char tempChars[numChars];

typedef struct struct_message{
  char message[64];
  } struct_message;

struct_message rcv_commands;

// Para o MPU

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

// Atualizando dados recebidos via ESP-NOW------------------------

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&rcv_commands, incomingData, sizeof(rcv_commands));
  first_mark = millis();
  strcpy(commands, rcv_commands.message);
}

// Tratamento de string ----------------------------------------

void parseData(){
    char * strtokIndx;
  
    strtokIndx = strtok(tempChars, ",");
    
    while (strtokIndx != NULL){
        id = atoi(strtokIndx);
        
        if(id == robot_id){         
          strtokIndx = strtok(NULL, ",");  
          v_l = atof(strtokIndx);       
          strtokIndx = strtok(NULL, ",");         
          v_a = atof(strtokIndx);
          strtokIndx = strtok(NULL, ","); 
       }

       else{
          strtokIndx = strtok(NULL, ",");     
          strtokIndx = strtok(NULL, ",");         
          strtokIndx = strtok(NULL, ","); 
       }
   } 
}

// Funções do MPU --------------------------------------------------

// Inicialização para o setup
void mpu_init(void) {

  Serial.println("MPU6050 init");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

// Velocidade angular
float get_theta_speed(){

	sensors_event_t a, g, temp;
	mpu.getEvent(&a, &g, &temp);
	float theta = g.gyro.z; 
	return theta;
}

// PID --------------------------------------------------

float pid(float target, float atual){
	float kp = -1.59521;
	float ki = -0.16864;
	float kd = 0.16686;

	float error = target - atual;
  error_sum += error;
 
  float P = error * kp;
  float I = error_sum * ki;
  float D = (error - last_error) * kd;
  
	float output = P+I+D;

	return output;
}

// Funções do motores -------------------------------------------

void motor_R(int speedR) {
  if (speedR > 0) {
    digitalWrite(A1, 1);
    digitalWrite(B1, 0);
  } else {
    digitalWrite(A1, 0);
    digitalWrite(B1, 1);
  }
  ledcWrite(PMW_CANAL_R, abs(speedR));
}


void motor_L(int speedL) {
  if (speedL > 0) {
    digitalWrite(A2, 1);
    digitalWrite(B2, 0);
  } else {
    digitalWrite(A2, 0);
    digitalWrite(B2, 1);
  }
  ledcWrite(PMW_CANAL_L, abs(speedL));
}

void motors_control(float linear, float angular) {
  angular = pid(angular, - get_theta_speed());

  if (linear > 0 ) linear = map(linear, 0, 255, 60, 255);
  if (linear < 0 ) linear = map(linear, 0, -255, -60, -255);


  float Vel_R = linear - angular; //ao somar o angular com linear em cada motor conseguimos a ideia de direcao do robo
  float Vel_L = linear + angular;

  if (Vel_R < 15 && Vel_R > -15) Vel_R = 0;
  if (Vel_R > MAX_PWM ) Vel_R = MAX_PWM;
  if (Vel_R < -MAX_PWM) Vel_R = -MAX_PWM;

  if (Vel_L < 15 && Vel_L > -15) Vel_L = 0;
  if (Vel_L > MAX_PWM ) Vel_L = MAX_PWM;
  if (Vel_L < -MAX_PWM) Vel_L = -MAX_PWM;

  motor_R(Vel_R); //manda para a funcao motor um valor de -255 a 255, o sinal signifca a direcao
  motor_L(Vel_L);

}

// Setup e loop --------------------------------------------------

void setup() {
  
  // Configuração de pinos para PWM
  ledcAttachPin(PWMA, PMW_CANAL_R);
  ledcAttachPin(PWMB, PMW_CANAL_L);

  ledcSetup(PMW_CANAL_R, 80000, 8);
  ledcSetup(PMW_CANAL_L, 80000, 8);

  // Configuração de pinos
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(B2, OUTPUT);
  digitalWrite(A1, 0);
  digitalWrite(A2, 0);
  digitalWrite(B1, 0);
  digitalWrite(B2, 0);

  // Configurações comunicação ESP-NOW

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
  ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK( esp_wifi_start());
  ESP_ERROR_CHECK( esp_wifi_set_channel(CANAL_ESPNOW, WIFI_SECOND_CHAN_NONE));

  if (esp_now_init() != ESP_OK) 
  {
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  second_mark = millis();

  // Necessário para proteger a informação original
  strcpy(tempChars, commands);
  parseData();

  // Protegendo o robô após a perda de informação
  if (second_mark - first_mark > 500) {
    v_l = 0.00;
    v_a = 0.00;
  }

  // Executando o controle dos motores
  motors_control(v_l, v_a);
}