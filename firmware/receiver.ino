// Código para o recebimento de informações e controle do robô

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#define PWMA 15
#define A1 2
#define B1 4

#define PWMB 13
#define A2 12 
#define B2 14

// Definindo variáveis ------------------------------------------
int robot_id = 0; // ATENÇÃO: Mudar o id de acordo com o robô

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

// Atualizando dados recebidos via ESP-NOW------------------------

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&rcv_commands, incomingData, sizeof(rcv_commands));
  first_mark = millis();
  strcpy(commands, rcv_commands.message);
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
  ledcWrite(1, abs(speedR));
}


void motor_L(int speedL) {
  if (speedL > 0) {
    digitalWrite(A2, 1);
    digitalWrite(B2, 0);
  } else {
    digitalWrite(A2, 0);
    digitalWrite(B2, 1);
  }
  ledcWrite(2, abs( speedL));
}


void motors_control(float wl, float wr) {
  if (wr > 255)
  {
    motor_R(255);
  } else if (wr < -255)
  {
    motor_R(-255);
  } else
  {
    motor_R(wr);
  }

  if (wl > 255)
  {
    motor_L(255);
  } else if (wl < -255)
  {
    motor_L(-255);
  } else
  {
    motor_L(wl);
  }

}

// Funções do motores ----------------------------------------

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

// Setup e loop --------------------------------------------------

void setup() {
  
  // Configuração de pinos para PWM
  ledcAttachPin(PWMA, 1);
  ledcAttachPin(PWMB, 2);

  ledcSetup(1, 80000, 8);
  ledcSetup(2, 80000, 8);

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
  ESP_ERROR_CHECK( esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

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
    v_l = 0.00;
  if (second_mark - first_mark > 500) {
    v_a = 0.00;
  }

  // Executando o controle dos motores
  motors_control(v_l, v_a);
}

